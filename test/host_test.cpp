// Host-side unit test for the W-Mesh protocol core (no hardware needed).
// Build:  c++ -std=c++17 -I../src host_test.cpp -o host_test && ./host_test
// SPDX-License-Identifier: MIT
#include "../core/wmesh_core.h"
#include "../core/wmesh_beacon.h"
#include "../core/wmesh_config.h"
#include "../core/wmesh_console.h"
#include "../core/openlora.h"
#include "../core/lorawan.h"
#include "../firmware/stm32wl/src/hdc3020.h"
#include "../core/gnss.h"
#include <cassert>
#include <cstdio>
#include <vector>

using namespace wmesh;

int main() {
  // --- header round-trip -----------------------------------------------
  MeshHeader h{42, 7, 3, 9};
  uint8_t buf[3];
  packHeader(h, buf);
  MeshHeader r = unpackHeader(buf);
  assert(r.origin == 42 && r.seq == 7 && r.depth == 3 && r.ttl == 9);

  // --- gradient rule: strict monotone descent ---------------------------
  assert( gradientAccepts(1, MeshHeader{5,0,2,4}));   // deeper sender -> accept
  assert(!gradientAccepts(2, MeshHeader{5,0,2,4}));   // equal depth  -> reject
  assert(!gradientAccepts(3, MeshHeader{5,0,2,4}));   // shallower    -> reject
  assert(!gradientAccepts(1, MeshHeader{5,0,2,0}));   // TTL exhausted -> reject

  // --- dedup cache -------------------------------------------------------
  DedupCache dc;
  assert(!dc.seenOrRecord(10, 1));
  assert( dc.seenOrRecord(10, 1));       // duplicate suppressed
  assert(!dc.seenOrRecord(10, 2));       // new seq passes
  assert(!dc.seenOrRecord(11, 1));       // other origin passes

  // --- forward queue + restamp ------------------------------------------
  ForwardQueue q;
  uint8_t frame[MAX_FRAME_LEN] = {0};
  packHeader(MeshHeader{9, 3, 2, 5}, frame);        // sender at depth 2
  frame[3] = 0xAB;                                   // payload marker
  assert(q.push(frame, 10));
  size_t len = 0;
  uint8_t out[MAX_FRAME_LEN];
  std::memcpy(out, q.frameAt(0, len), len);
  assert(len == 10 && out[3] == 0xAB);
  restampForForward(out, /*myDepth=*/1);
  MeshHeader f = unpackHeader(out);
  assert(f.origin == 9 && f.seq == 3);               // preserved end-to-end
  assert(f.depth == 1 && f.ttl == 4);                // re-stamped, TTL--
  // a peer at the same depth as the relay must now reject it:
  assert(!gradientAccepts(1, f));
  // ...while the next tier up accepts:
  assert( gradientAccepts(0, f));

  // --- queue capacity bound (fan-in x ports) ----------------------------
  ForwardQueue q2;
  for (size_t i = 0; i < ForwardQueue::CAPACITY; ++i) assert(q2.push(frame, 10));
  assert(!q2.push(frame, 10));                       // 57th refused

  // --- cascade phase arithmetic ------------------------------------------
  const uint32_t dur[3] = {200, 300, 400};           // H=3, deepest first
  PhasePlan p3 = planFor(3, 3, dur);                 // deepest tier
  assert(p3.txStart == 0   && p3.txEnd == 200);
  assert(p3.listenStart == 0 && p3.listenEnd == 0);  // nothing deeper to hear
  PhasePlan p2 = planFor(2, 3, dur);
  assert(p2.txStart == 200 && p2.txEnd == 500);
  assert(p2.listenStart == 0 && p2.listenEnd == 200);
  PhasePlan p1 = planFor(1, 3, dur);
  assert(p1.txStart == 500 && p1.txEnd == 900);
  assert(p1.listenStart == 200 && p1.listenEnd == 500);

  // --- Open LoRa v5: analog round-trip, battery, pulse (32-bit final) ------
  std::vector<uint16_t> readings;
  for (int i = 0; i < 96; ++i) readings.push_back(static_cast<uint16_t>(1000 + 3*i));
  uint8_t pl[openlora::HDR_LEN + 2*96];
  size_t plen = openlora::encodeAnalog(2, openlora::RecordPeriod::MINUTES, 15,
                                       readings.data(), 96, pl, sizeof pl);
  assert(plen == 5 + 2*96);                          // full-day buffer, 197 B
  assert(pl[0] == 5);                                // version
  assert((pl[1] & 0x0F) == 2 && ((pl[1] >> 4) & 3) == 1); // sensor 2, analog
  uint16_t back[96];
  uint8_t si = 0;
  uint16_t n = openlora::decodeAnalog(pl, plen, back, 96, &si);
  assert(n == 96 && si == 2);
  for (int i = 0; i < 96; ++i) assert(back[i] == readings[i]);

  // full frame must fit the SF8 ceiling with the mesh header
  assert(MESH_HDR_LEN + plen <= MAX_FRAME_LEN);

  // battery message (FPort 13): 3 bytes, exact wire image
  uint8_t bmsg[openlora::BATTERY_LEN];
  assert(openlora::encodeBattery(3600, bmsg) == 3);
  assert(bmsg[0] == 5 && bmsg[1] == 0x0E && bmsg[2] == 0x10);
  uint16_t mvb = 0;
  assert(openlora::decodeBattery(bmsg, 3, mvb) && mvb == 3600);
  bmsg[0] = 4;                                       // wrong version rejected
  assert(!openlora::decodeBattery(bmsg, 3, mvb));

  // position message (FPort 14, extension aligned with the core protocol's
  // GNSS point: lat/lon x1e7, altitude in mm, the v4 status enum)
  uint8_t pmsg2[openlora::POSITION_LEN];
  assert(openlora::encodePosition(0, openlora::GnssStatus::OK,
                                  427059372, 233277761, 609500, 28, 120,
                                  pmsg2) == 16);
  assert(pmsg2[0] == 5 && pmsg2[1] == 0);            // source 0, status OK
  assert(pmsg2[2] == 0x19 && pmsg2[3] == 0x74);      // 427059372 big-endian
  int32_t pla, plo, pam; uint8_t pnf, ph10;
  assert(openlora::decodePosition(pmsg2, 16, pla, plo, pam, pnf, ph10));
  assert(pla == 427059372 && plo == 233277761);      // Sofia bench fix
  assert(pam == 609500 && pnf == 28 && ph10 == 12);  // 609.5 m in mm
  // southern/western hemispheres survive the sign
  openlora::GnssStatus pst; uint8_t psi = 0;
  assert(openlora::encodePosition(3, openlora::GnssStatus::NO_FIX,
                                  -626353908, -603590535, 105300, 300, 2600,
                                  pmsg2) == 16);
  assert(openlora::decodePosition(pmsg2, 16, pla, plo, pam, pnf, ph10,
                                  &pst, &psi));
  assert(pla == -626353908 && plo == -603590535);    // Livingston Island
  assert(psi == 3 && pst == openlora::GnssStatus::NO_FIX);
  assert(pnf == 255 && ph10 == 255);                 // saturation
  pmsg2[0] = 4;
  assert(!openlora::decodePosition(pmsg2, 16, pla, plo, pam, pnf, ph10));

  // pulse message: 16-bit differentials + 32-bit absolute final counter
  const uint32_t pulses[3] = {70000, 70123, 70456};  // beyond 16-bit range
  uint8_t pmsg[openlora::HDR_LEN + 2*2 + 4];
  const size_t pn = openlora::encodePulse(0, openlora::RecordPeriod::MINUTES, 15,
                                          pulses, 3, pmsg, sizeof pmsg);
  assert(pn == 5 + 2*2 + 4);
  uint32_t pback[3];
  assert(openlora::decodePulse(pmsg, pn, pback, 3) == 3);
  for (int i = 0; i < 3; ++i) assert(pback[i] == pulses[i]);

  // --- Class B beacon: CRC16 (x^16+x^12+x^5+1, init 0 = XMODEM) ------------
  const uint8_t chk[] = {'1','2','3','4','5','6','7','8','9'};
  assert(beaconCrc16(chk, sizeof chk) == 0x31C3);    // standard check value

  // --- beacon build / parse / corrupt --------------------------------------
  uint8_t bcn[BEACON_LEN] = {0};
  const uint32_t t0 = 1400000000u;                   // GPS seconds, arbitrary
  restampBeacon(bcn, t0);                            // writes time + CRC1
  for (size_t i = 0; i < 7; ++i) bcn[BCN_GWS_OFF + i] = static_cast<uint8_t>(i);
  Beacon b1 = parseBeacon(bcn, BEACON_LEN);
  assert(b1.valid && b1.timeGps == t0);
  assert(!parseBeacon(bcn, BEACON_LEN - 1).valid);   // wrong length rejected
  bcn[3] ^= 0x01;                                    // corrupt the time field
  assert(!parseBeacon(bcn, BEACON_LEN).valid);       // CRC catches it
  bcn[3] ^= 0x01;                                    // restore

  // --- relay re-stamp: +1 pre-roll slot, GwSpecific untouched --------------
  restampBeacon(bcn, t0 + 2);
  Beacon b2 = parseBeacon(bcn, BEACON_LEN);
  assert(b2.valid && b2.timeGps == t0 + 2);
  for (size_t i = 0; i < 7; ++i)
    assert(bcn[BCN_GWS_OFF + i] == static_cast<uint8_t>(i));  // verbatim

  // --- pre-roll cascade: outward, one slot per tier, phases start after ----
  assert(prerollSlotStart(0, 2) == 0);               // gateway emits at t=0
  assert(prerollSlotStart(1, 2) == 2);               // tier-1 re-broadcast
  assert(prerollSlotStart(2, 2) == 4);               // tier-2 re-broadcast
  assert(prerollEnd(3, 2) == 6);                     // then phase 1 (deepest)
  // every tier's re-broadcast lands before the uplink phases begin:
  for (uint8_t d = 0; d <= 3; ++d) assert(prerollSlotStart(d, 2) <= prerollEnd(3, 2));

  // --- missed-beacon fallback: guard widens per silent day ------------------
  assert(widenedGuard(1, 1, 0) == 1);
  assert(widenedGuard(1, 1, 3) == 4);

  // --- provisioning: defaults valid, pack/unpack round-trip, CRC guard -----
  Config c0 = configDefaults();
  assert(configValidate(c0) == nullptr);
  assert(c0.windowUtcSec == 43200);                  // 12:00 UTC anchor
  c0.profile = Profile::RELAY; c0.nodeId = 21; c0.depth = 1;
  c0.meshH = 2; c0.phaseDur[0] = 300; c0.phaseDur[1] = 600;
  c0.txSlotIndex = 3; c0.whitelistLen = 2; c0.whitelist[0] = 31; c0.whitelist[1] = 32;
  assert(configValidate(c0) == nullptr);
  uint8_t img[CFG_IMAGE_LEN];
  configPack(c0, img);
  Config c1 = configDefaults();
  assert(configUnpack(img, c1));
  assert(c1.nodeId == 21 && c1.profile == Profile::RELAY && c1.meshH == 2);
  assert(c1.windowUtcSec == 43200 && c1.phaseDur[1] == 600);
  assert(c1.txSlotIndex == 3 && c1.acqListenS == 360);
  assert(c1.whitelistLen == 2 && c1.whitelist[1] == 32);
  // pre-roll sub-slots survive the nibble packing
  Config cs = c0; cs.prerollSlotS = 5; cs.prerollSub = 3; cs.parentPrerollSub = 1;
  assert(configValidate(cs) == nullptr);
  uint8_t img2[CFG_IMAGE_LEN];
  configPack(cs, img2);
  Config cs2 = configDefaults();
  assert(configUnpack(img2, cs2));
  assert(cs2.prerollSub == 3 && cs2.parentPrerollSub == 1);
  cs.prerollSub = 5;                                 // == prerollSlotS -> invalid
  assert(configValidate(cs) != nullptr);
  // GNSS backup flag: packed with prefixless, survives the round-trip
  Config cg = c0; cg.gnssBackup = 1; cg.prefixless = 1;
  assert(configValidate(cg) == nullptr);
  uint8_t img3[CFG_IMAGE_LEN];
  configPack(cg, img3);
  Config cg2 = configDefaults();
  assert(configUnpack(img3, cg2));
  assert(cg2.gnssBackup == 1 && cg2.prefixless == 1);
  cg.profile = Profile::STAKE; cg.whitelistLen = 0;
  assert(configValidate(cg) != nullptr);             // stake has GNSS already

  img[10] ^= 0xFF;                                   // corrupt
  Config c2 = configDefaults();
  assert(!configUnpack(img, c2));                    // CRC rejects
  assert(c2.nodeId == configDefaults().nodeId);      // untouched -> defaults

  // --- validation rejects broken provisioning -------------------------------
  Config cb = c0; cb.depth = 3;
  assert(configValidate(cb) != nullptr);             // depth > h
  cb = c0; cb.txSlotIndex = 100;                     // 101*12 s > own phase 600 s
  assert(configValidate(cb) != nullptr);
  cb = c0; cb.profile = Profile::LEAF;               // whitelist on a leaf
  assert(configValidate(cb) != nullptr);
  cb = c0; cb.windowUtcSec = 86400;
  assert(configValidate(cb) != nullptr);

  // --- console: the enrolment dialogue --------------------------------------
  Config cc = configDefaults();
  char rsp[512];
  ConsoleResult cr = consoleLine(cc, "set window 12:05", rsp, sizeof rsp);
  assert(cr.action == ConsoleAction::NONE && cc.windowUtcSec == 43500);
  cr = consoleLine(cc, "set profile relay", rsp, sizeof rsp);
  assert(cc.profile == Profile::RELAY);
  cr = consoleLine(cc, "set phases 300,600", rsp, sizeof rsp);
  assert(cc.meshH == 2 && cc.phaseDur[0] == 300 && cc.phaseDur[1] == 600);
  cr = consoleLine(cc, "set depth 1", rsp, sizeof rsp);
  cr = consoleLine(cc, "set slot 3", rsp, sizeof rsp);
  cr = consoleLine(cc, "set whitelist 31,32,33", rsp, sizeof rsp);
  assert(cc.whitelistLen == 3 && cc.whitelist[2] == 33);
  cr = consoleLine(cc, "set acq 300", rsp, sizeof rsp);
  assert(cc.acqListenS == 300);
  cr = consoleLine(cc, "set time 1785000000", rsp, sizeof rsp);
  assert(cr.action == ConsoleAction::SETTIME && cr.timeEpoch == 1785000000u);
  cr = consoleLine(cc, "save", rsp, sizeof rsp);
  assert(cr.action == ConsoleAction::SAVE);          // valid -> saved
  cr = consoleLine(cc, "boot", rsp, sizeof rsp);
  assert(cr.action == ConsoleAction::BOOT);
  cr = consoleLine(cc, "exit", rsp, sizeof rsp);   // the operator's instinct
  assert(cr.action == ConsoleAction::BOOT);
  // broken input never saves:
  Config cx = configDefaults(); cx.depth = 9; cx.meshH = 2;
  cr = consoleLine(cx, "save", rsp, sizeof rsp);
  assert(cr.action == ConsoleAction::NONE);          // ERR, no SAVE action
  cr = consoleLine(cx, "set window 25:00", rsp, sizeof rsp);
  assert(cx.windowUtcSec == configDefaults().windowUtcSec); // rejected
  cr = consoleLine(cx, "set whitelist 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15", rsp, sizeof rsp);
  assert(cx.whitelistLen == 0);                      // >14 rejected

  // --- bare leaf (XIAO bench node): no GNSS, no HDC3020, battery only -------
  {
    // the enrolment dialogue that provisions one of the three bench nodes
    Config cb2 = configDefaults();
    consoleLine(cb2, "set id 7", rsp, sizeof rsp);
    consoleLine(cb2, "set profile leaf", rsp, sizeof rsp);
    consoleLine(cb2, "set ports 1", rsp, sizeof rsp);       // battery IS port 0
    consoleLine(cb2, "set phases 900", rsp, sizeof rsp);
    consoleLine(cb2, "set depth 1", rsp, sizeof rsp);
    consoleLine(cb2, "set slot 2", rsp, sizeof rsp);
    assert(consoleLine(cb2, "save", rsp, sizeof rsp).action == ConsoleAction::SAVE);
    assert(cb2.ports == 1 && cb2.profile == Profile::LEAF);
    // survives the flash image round-trip
    uint8_t imgB[CFG_IMAGE_LEN];
    configPack(cb2, imgB);
    Config cb3 = configDefaults();
    assert(configUnpack(imgB, cb3) && cb3.ports == 1 && cb3.nodeId == 7);
    // ports = 0 stays impossible
    cb3.ports = 0;
    assert(configValidate(cb3) != nullptr);
    // the bare leaf's whole uplink is the 3-byte battery message (FPort 13)
    uint8_t bpl[openlora::BATTERY_LEN];
    assert(openlora::encodeBattery(4200, bpl) == openlora::BATTERY_LEN);
    uint16_t mvOne = 0;
    assert(openlora::decodeBattery(bpl, openlora::BATTERY_LEN, mvOne));
    assert(mvOne == 4200);
  }

  // --- LoRaWAN ABP: AES/CMAC vectors, key store, uplink round-trip -----------
  {
    // AES-128 known answer (FIPS-197 appendix C.1)
    const uint8_t ak[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
                            0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};
    const uint8_t apt[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
                             0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    const uint8_t act[16] = {0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
                             0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a};
    uint8_t aout[16];
    lorawan::detail::aesEncryptBlock(ak, apt, aout);
    assert(std::memcmp(aout, act, 16) == 0);
    // AES-CMAC known answer (RFC 4493, example 2: 16-byte message)
    const uint8_t ck[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
                            0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    const uint8_t cm[16] = {0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
                            0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a};
    const uint8_t cx[16] = {0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,
                            0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c};
    uint8_t mac[16];
    lorawan::detail::aesCmac(ck, cm, 16, mac);
    assert(std::memcmp(mac, cx, 16) == 0);
    // and the empty-message vector (example 1)
    const uint8_t cx0[16] = {0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,
                             0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46};
    lorawan::detail::aesCmac(ck, cm, 0, mac);
    assert(std::memcmp(mac, cx0, 16) == 0);

    // key store: pack/unpack round-trip, CRC guard
    lorawan::AbpKeys k;
    k.devAddr = 0x26011BDA;
    for (int i = 0; i < 16; ++i) { k.nwkSKey[i] = (uint8_t)i; k.appSKey[i] = (uint8_t)(0xF0 + i); }
    k.fcntUp = 42;
    uint8_t kimg[lorawan::KEYS_IMAGE_LEN];
    lorawan::keysPack(k, kimg);
    lorawan::AbpKeys k2;
    assert(lorawan::keysUnpack(kimg, k2));
    assert(k2.devAddr == k.devAddr && k2.fcntUp == 42);
    assert(std::memcmp(k2.appSKey, k.appSKey, 16) == 0);
    kimg[9] ^= 0xFF;
    assert(!lorawan::keysUnpack(kimg, k2));

    // uplink: build -> open (decrypt + MIC verify), MIC corruption rejected
    const uint8_t pay[8] = {1,2,3,4,5,6,7,8};
    uint8_t phy[64];
    const size_t sz = lorawan::buildUplink(k, 1, pay, sizeof pay, phy, sizeof phy);
    assert(sz == 13 + 8);
    assert(phy[0] == 0x40);                           // unconfirmed data up
    assert(phy[8] == 1);                              // FPort
    assert(std::memcmp(phy + 9, pay, 8) != 0);        // actually encrypted
    uint8_t back2[64]; size_t bn; uint8_t fp;
    assert(lorawan::openUplink(k, phy, sz, back2, bn, fp));
    assert(bn == 8 && fp == 1 && std::memcmp(back2, pay, 8) == 0);
    phy[sz - 1] ^= 0x01;                              // corrupt the MIC
    assert(!lorawan::openUplink(k, phy, sz, back2, bn, fp));

    // console: the ABP dialogue edits and displays the keys
    Config ca = configDefaults();
    lorawan::AbpKeys ka;
    assert(consoleLine(ca, &ka, "set devaddr 26011BDA", rsp, sizeof rsp).action == ConsoleAction::NONE);
    assert(ka.devAddr == 0x26011BDA);
    consoleLine(ca, &ka, "set nwkskey 000102030405060708090A0B0C0D0E0F", rsp, sizeof rsp);
    assert(ka.nwkSKey[1] == 0x01 && ka.nwkSKey[15] == 0x0F);
    consoleLine(ca, &ka, "set appskey F0F1F2F3F4F5F6F7F8F9FAFBFCFDFEFF", rsp, sizeof rsp);
    assert(ka.appSKey[0] == 0xF0);
    consoleLine(ca, &ka, "set fcnt 100", rsp, sizeof rsp);
    assert(ka.fcntUp == 100);
    consoleLine(ca, &ka, "show", rsp, sizeof rsp);
    assert(std::strstr(rsp, "abp devaddr 26011BDA") != nullptr);
    assert(std::strstr(rsp, "F0F1F2F3") != nullptr);  // keys are READABLE
    consoleLine(ca, &ka, "set nwkskey zz", rsp, sizeof rsp);
    assert(std::strstr(rsp, "ERR") == rsp);           // bad hex rejected
    consoleLine(ca, &ka, "set devaddr 0", rsp, sizeof rsp);
    assert(!lorawan::keysValid(ka));                  // back to raw mode
  }

  // --- HDC3020: CRC-8 vector and reply parsing ------------------------------
  const uint8_t sv[] = {0xBE, 0xEF};
  assert(hdc3020::crc8(sv, 2) == 0x92);              // datasheet check value
  uint8_t rep[6] = {0x60, 0x12, 0, 0x80, 0x00, 0};
  rep[2] = hdc3020::crc8(rep, 2);
  rep[5] = hdc3020::crc8(rep + 3, 2);
  hdc3020::Sample smp = hdc3020::parseReply(rep);
  assert(smp.valid && smp.rawT == 0x6012 && smp.rawRH == 0x8000);
  assert(hdc3020::toPercentRh(smp.rawRH) > 49.9f && hdc3020::toPercentRh(smp.rawRH) < 50.1f);
  rep[1] ^= 0x04;                                    // corrupt
  assert(!hdc3020::parseReply(rep).valid);

  // --- GNSS: checksum, GGA parse, averaging session, position words --------
  assert( gnss::nmeaChecksumOk("$A*41"));            // XOR('A') = 0x41
  assert( gnss::nmeaChecksumOk("$AB*03"));           // 0x41^0x42 = 0x03
  assert(!gnss::nmeaChecksumOk("$AB*04"));
  // Livingston-ish sentence: 62 deg 38.12345' S, 060 deg 21.54321' W
  {
    char sent[128];
    std::snprintf(sent, sizeof sent,
      "$GNGGA,120000.00,6238.12345,S,06021.54321,W,1,08,1.20,105.3,M,,M,,");
    uint8_t x = 0;
    for (const char* q = sent + 1; *q; ++q) x ^= (uint8_t)*q;
    char full[140];
    std::snprintf(full, sizeof full, "%s*%02X", sent, x);
    gnss::GgaFix fx = gnss::parseGga(full);
    assert(fx.valid);
    assert(fx.utcDaySec == 43200);                   // 12:00:00 UTC
    assert(fx.latE7 == -626353908);                  // 62 + 38.12345/60 deg, S
    assert(fx.lonE7 == -603590535);                  // 60 + 21.54321/60 deg, W
    assert(fx.altDm == 1053 && fx.nSats == 8 && fx.hdopX100 == 120);
    // corrupt checksum -> rejected
    full[std::strlen(full) - 1] ^= 0x01;
    assert(!gnss::parseGga(full).valid);
    // averaging: two fixes 2 units apart -> mean in the middle
    gnss::SessionAvg avg;
    gnss::GgaFix f2 = fx; f2.latE7 += 2; f2.lonE7 += 2;
    assert(avg.add(fx) && avg.add(f2));
    assert(avg.count() == 2 && avg.latE7() == fx.latE7 + 1);
    // the quality filter actually filters:
    gnss::GgaFix bad = fx; bad.nSats = 3;
    assert(!avg.add(bad));
    bad = fx; bad.hdopX100 = 900;
    assert(!avg.add(bad));
    assert(avg.count() == 2);
    // position words round-trip
    uint16_t w[gnss::POS_WORDS];
    gnss::packPosition(avg, w);
    int32_t la, lo; int16_t al; uint16_t nf;
    gnss::unpackPosition(w, la, lo, al, nf);
    assert(la == avg.latE7() && lo == avg.lonE7());
    assert(al == avg.altDm() && nf == 2);
    // and the frame fits OpenLoRa comfortably
    assert(6 + 2 * gnss::POS_WORDS == 22);
  }

  std::puts("ALL W-MESH CORE TESTS PASSED");
  return 0;
}
