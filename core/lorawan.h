// Minimal LoRaWAN 1.0.x ABP uplink builder — platform-independent.
//
// W-Mesh is TRANSPARENT to LoRaWAN (paper §III-C): a leaf builds a complete
// PHYPayload (MHDR | FHDR | FPort | encrypted FRMPayload | MIC), the mesh
// carries it under the 3-byte prefix, relays forward the ciphertext + MIC +
// FCnt verbatim, and the tier-1 hop strips the prefix so a standard gateway
// parses a standard ABP uplink. This header supplies exactly the pieces a
// node needs for that: AES-128 (encrypt-only), AES-CMAC (RFC 4493), the
// FRMPayload encryption of TS001 §4.3.3 and the MIC of §4.4.
//
// Session state (ABP): DevAddr + NwkSKey + AppSKey + FCntUp, stored in a
// small CRC-guarded flash blob next to the provisioning page. FCntUp must
// survive resets — the network side tracks it monotonically.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include "wmesh_beacon.h"   // beaconCrc16 for the key-store image

namespace lorawan {

// ---- AES-128, encrypt only (FIPS-197) ---------------------------------------
namespace detail {

static const uint8_t SBOX[256] = {
  0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
  0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
  0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
  0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
  0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
  0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
  0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
  0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
  0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
  0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
  0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
  0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
  0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
  0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
  0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
  0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16 };

inline uint8_t xtime(uint8_t x) {
  return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1B : 0x00));
}

inline void aesEncryptBlock(const uint8_t key[16], const uint8_t in[16],
                            uint8_t out[16]) {
  uint8_t rk[16], s[16];
  std::memcpy(rk, key, 16);
  std::memcpy(s, in, 16);
  uint8_t rcon = 0x01;
  for (int i = 0; i < 16; ++i) s[i] ^= rk[i];        // AddRoundKey 0
  for (int round = 1; round <= 10; ++round) {
    // next round key (on the fly)
    rk[0] ^= (uint8_t)(SBOX[rk[13]] ^ rcon);
    rk[1] ^= SBOX[rk[14]];
    rk[2] ^= SBOX[rk[15]];
    rk[3] ^= SBOX[rk[12]];
    rcon = xtime(rcon);
    for (int i = 4; i < 16; ++i) rk[i] ^= rk[i - 4];
    // SubBytes + ShiftRows
    uint8_t t[16];
    static const uint8_t SHIFT[16] =
        {0,5,10,15,4,9,14,3,8,13,2,7,12,1,6,11};
    for (int i = 0; i < 16; ++i) t[i] = SBOX[s[SHIFT[i]]];
    // MixColumns (skipped in the final round)
    if (round < 10) {
      for (int c = 0; c < 16; c += 4) {
        const uint8_t a0 = t[c], a1 = t[c+1], a2 = t[c+2], a3 = t[c+3];
        const uint8_t x = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
        t[c]   ^= (uint8_t)(x ^ xtime((uint8_t)(a0 ^ a1)));
        t[c+1] ^= (uint8_t)(x ^ xtime((uint8_t)(a1 ^ a2)));
        t[c+2] ^= (uint8_t)(x ^ xtime((uint8_t)(a2 ^ a3)));
        t[c+3] ^= (uint8_t)(x ^ xtime((uint8_t)(a3 ^ a0)));
      }
    }
    for (int i = 0; i < 16; ++i) s[i] = (uint8_t)(t[i] ^ rk[i]);
  }
  std::memcpy(out, s, 16);
}

// ---- AES-CMAC (RFC 4493) ----------------------------------------------------
inline void leftShift(const uint8_t in[16], uint8_t out[16]) {
  uint8_t carry = 0;
  for (int i = 15; i >= 0; --i) {
    out[i] = (uint8_t)((in[i] << 1) | carry);
    carry = (uint8_t)(in[i] >> 7);
  }
}

inline void aesCmac(const uint8_t key[16], const uint8_t* msg, size_t len,
                    uint8_t mac[16]) {
  uint8_t k1[16], k2[16], zero[16] = {0}, L[16];
  aesEncryptBlock(key, zero, L);
  leftShift(L, k1);
  if (L[0] & 0x80) k1[15] ^= 0x87;
  leftShift(k1, k2);
  if (k1[0] & 0x80) k2[15] ^= 0x87;

  const size_t nBlocks = (len == 0) ? 1 : (len + 15) / 16;
  uint8_t x[16] = {0}, block[16];
  for (size_t b = 0; b + 1 < nBlocks; ++b) {
    for (int i = 0; i < 16; ++i) x[i] ^= msg[b * 16 + i];
    aesEncryptBlock(key, x, x);
  }
  const size_t rem = len - (nBlocks - 1) * 16;
  std::memset(block, 0, 16);
  std::memcpy(block, msg + (nBlocks - 1) * 16, rem);
  if (rem == 16) {
    for (int i = 0; i < 16; ++i) block[i] ^= k1[i];
  } else {
    block[rem] = 0x80;
    for (int i = 0; i < 16; ++i) block[i] ^= k2[i];
  }
  for (int i = 0; i < 16; ++i) x[i] ^= block[i];
  aesEncryptBlock(key, x, mac);
}

} // namespace detail

// ---- ABP session state -------------------------------------------------------
constexpr size_t KEYS_IMAGE_LEN = 48;
// [ "WKY1" (4) ][ DevAddr (4 LE) ][ NwkSKey (16) ][ AppSKey (16) ]
// [ FCntUp (4 LE) ][ pad (2) ][ CRC16 (2) ]

struct AbpKeys {
  uint32_t devAddr = 0;        // 0 = ABP not provisioned -> raw OpenLoRa mode
  uint8_t  nwkSKey[16] = {0};
  uint8_t  appSKey[16] = {0};
  uint32_t fcntUp = 0;
};

inline bool keysValid(const AbpKeys& k) { return k.devAddr != 0; }

inline void keysPack(const AbpKeys& k, uint8_t img[KEYS_IMAGE_LEN]) {
  std::memset(img, 0, KEYS_IMAGE_LEN);
  std::memcpy(img, "WKY1", 4);
  img[4] = (uint8_t)k.devAddr;       img[5] = (uint8_t)(k.devAddr >> 8);
  img[6] = (uint8_t)(k.devAddr >> 16); img[7] = (uint8_t)(k.devAddr >> 24);
  std::memcpy(img + 8,  k.nwkSKey, 16);
  std::memcpy(img + 24, k.appSKey, 16);
  img[40] = (uint8_t)k.fcntUp;        img[41] = (uint8_t)(k.fcntUp >> 8);
  img[42] = (uint8_t)(k.fcntUp >> 16); img[43] = (uint8_t)(k.fcntUp >> 24);
  const uint16_t crc = wmesh::beaconCrc16(img, KEYS_IMAGE_LEN - 2);
  img[KEYS_IMAGE_LEN - 2] = (uint8_t)crc;
  img[KEYS_IMAGE_LEN - 1] = (uint8_t)(crc >> 8);
}

inline bool keysUnpack(const uint8_t img[KEYS_IMAGE_LEN], AbpKeys& k) {
  if (std::memcmp(img, "WKY1", 4) != 0) return false;
  const uint16_t stored = (uint16_t)img[KEYS_IMAGE_LEN - 2]
                        | (uint16_t)img[KEYS_IMAGE_LEN - 1] << 8;
  if (stored != wmesh::beaconCrc16(img, KEYS_IMAGE_LEN - 2)) return false;
  k.devAddr = (uint32_t)img[4] | (uint32_t)img[5] << 8
            | (uint32_t)img[6] << 16 | (uint32_t)img[7] << 24;
  std::memcpy(k.nwkSKey, img + 8,  16);
  std::memcpy(k.appSKey, img + 24, 16);
  k.fcntUp = (uint32_t)img[40] | (uint32_t)img[41] << 8
           | (uint32_t)img[42] << 16 | (uint32_t)img[43] << 24;
  return true;
}

// ---- uplink construction -----------------------------------------------------
// Unconfirmed data up (MHDR 0x40), no FOpts, no ADR. Output layout:
//   MHDR(1) | DevAddr(4 LE) | FCtrl(1)=0 | FCnt(2 LE) | FPort(1)
//   | encrypted FRMPayload(n) | MIC(4)                       = 13 + n bytes
// Uses k.fcntUp as the counter; the CALLER increments and persists it after
// a successful transmission.
inline size_t buildUplink(const AbpKeys& k, uint8_t fport,
                          const uint8_t* payload, size_t n,
                          uint8_t* out, size_t cap) {
  const size_t total = 13 + n;
  if (total > cap || fport == 0) return 0;
  out[0] = 0x40;                                     // unconfirmed data up
  out[1] = (uint8_t)k.devAddr;       out[2] = (uint8_t)(k.devAddr >> 8);
  out[3] = (uint8_t)(k.devAddr >> 16); out[4] = (uint8_t)(k.devAddr >> 24);
  out[5] = 0x00;                                     // FCtrl
  out[6] = (uint8_t)k.fcntUp;        out[7] = (uint8_t)(k.fcntUp >> 8);
  out[8] = fport;
  // FRMPayload encryption (TS001 §4.3.3): XOR with AES(AppSKey, A_i)
  uint8_t a[16] = {0}, s[16];
  a[0] = 0x01;
  a[5] = 0x00;                                       // dir = uplink
  std::memcpy(a + 6, out + 1, 4);                    // DevAddr LE
  a[10] = (uint8_t)k.fcntUp;  a[11] = (uint8_t)(k.fcntUp >> 8);
  a[12] = (uint8_t)(k.fcntUp >> 16); a[13] = (uint8_t)(k.fcntUp >> 24);
  for (size_t i = 0; i < n; ++i) {
    if (i % 16 == 0) {
      a[15] = (uint8_t)(i / 16 + 1);
      detail::aesEncryptBlock(k.appSKey, a, s);
    }
    out[9 + i] = (uint8_t)(payload[i] ^ s[i % 16]);
  }
  // MIC (TS001 §4.4): AES-CMAC(NwkSKey, B0 | MHDR..FRMPayload), first 4 B
  uint8_t b0msg[16 + 9 + 222];
  b0msg[0] = 0x49;
  std::memset(b0msg + 1, 0, 4);
  b0msg[5] = 0x00;                                   // dir = uplink
  std::memcpy(b0msg + 6, out + 1, 4);
  b0msg[10] = (uint8_t)k.fcntUp;  b0msg[11] = (uint8_t)(k.fcntUp >> 8);
  b0msg[12] = (uint8_t)(k.fcntUp >> 16); b0msg[13] = (uint8_t)(k.fcntUp >> 24);
  b0msg[14] = 0x00;
  b0msg[15] = (uint8_t)(9 + n);
  std::memcpy(b0msg + 16, out, 9 + n);
  uint8_t mac[16];
  detail::aesCmac(k.nwkSKey, b0msg, 16 + 9 + n, mac);
  std::memcpy(out + 9 + n, mac, 4);
  return total;
}

// Host-side check: decrypt an uplink built by buildUplink (encryption is an
// XOR keystream, so this is buildUplink's inner loop run in reverse) and
// verify the MIC. Gateway model for the tests.
inline bool openUplink(const AbpKeys& k, const uint8_t* frame, size_t len,
                       uint8_t* payload, size_t& n, uint8_t& fport) {
  if (len < 13 || frame[0] != 0x40) return false;
  const uint32_t addr = (uint32_t)frame[1] | (uint32_t)frame[2] << 8
                      | (uint32_t)frame[3] << 16 | (uint32_t)frame[4] << 24;
  if (addr != k.devAddr) return false;
  const uint16_t fcnt16 = (uint16_t)frame[6] | (uint16_t)frame[7] << 8;
  AbpKeys tmp = k;
  tmp.fcntUp = fcnt16;                               // 16-bit window model
  uint8_t rebuilt[256];
  n = len - 13;
  fport = frame[8];
  // decrypt = re-encrypt of the ciphertext
  uint8_t a[16] = {0}, s[16];
  a[0] = 0x01; std::memcpy(a + 6, frame + 1, 4);
  a[10] = frame[6]; a[11] = frame[7];
  for (size_t i = 0; i < n; ++i) {
    if (i % 16 == 0) {
      a[15] = (uint8_t)(i / 16 + 1);
      detail::aesEncryptBlock(k.appSKey, a, s);
    }
    payload[i] = (uint8_t)(frame[9 + i] ^ s[i % 16]);
  }
  const size_t rl = buildUplink(tmp, fport, payload, n, rebuilt, sizeof rebuilt);
  return rl == len && std::memcmp(rebuilt, frame, len) == 0;   // incl. MIC
}

} // namespace lorawan
