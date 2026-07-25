// W-Mesh protocol core — platform-independent (no Arduino dependencies).
// Reference implementation accompanying the paper:
//   "W-Mesh: a Duty-Cycled LoRa Mesh Extension for Windowed Buffered Telemetry"
// Section references (§) point to the paper.
//
// Frame layout on air:
//   [ MeshHeader (3 B) ][ Open LoRa v5 payload / LoRaWAN PHYPayload ]
//   MeshHeader: origin (1 B) | seq (1 B) | depth:4 bits, ttl:4 bits (1 B)   §III-A
//
// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace wmesh {

constexpr size_t MESH_HDR_LEN   = 3;
constexpr size_t MAX_FRAME_LEN  = 222;              // EU868 ceiling at SF7/SF8 (RP002)
constexpr size_t MAX_PAYLOAD    = MAX_FRAME_LEN - MESH_HDR_LEN;
constexpr uint8_t MAX_TTL       = 15;               // 4-bit field

struct MeshHeader {
  uint8_t origin;    // node id of the frame's originator (1..255; 0 = gateway)
  uint8_t seq;       // per-origin sequence number, wraps
  uint8_t depth;     // sender depth at transmission time (0..15)
  uint8_t ttl;       // remaining hops (0..15)
};

// --- header pack/unpack ----------------------------------------------------
inline void packHeader(const MeshHeader& h, uint8_t* buf) {
  buf[0] = h.origin;
  buf[1] = h.seq;
  buf[2] = static_cast<uint8_t>((h.depth << 4) | (h.ttl & 0x0F));
}
inline MeshHeader unpackHeader(const uint8_t* buf) {
  MeshHeader h;
  h.origin = buf[0];
  h.seq    = buf[1];
  h.depth  = buf[2] >> 4;
  h.ttl    = buf[2] & 0x0F;
  return h;
}

// --- gradient rule (§III-C: loop freedom by construction) -------------------
// A node at depth `myDepth` accepts a frame for forwarding ONLY if the sender
// was strictly deeper. Frames travel monotonically down the depth gradient,
// so the forwarding graph is a DAG and routing loops are impossible.
inline bool gradientAccepts(uint8_t myDepth, const MeshHeader& h) {
  if (h.ttl == 0) return false;          // hop limit exhausted (belt-and-braces)
  return h.depth > myDepth;              // strict inequality — the invariant
}

// --- duplicate cache (§III-C) -----------------------------------------------
// Suppresses copies arriving via multiple parents within one window.
// Fixed-size, cleared at window start; linear scan is fine at this scale.
class DedupCache {
public:
  static constexpr size_t CAPACITY = 64; // >= fan-in bound (14) x ports (4)
  void clear() { count_ = 0; }
  // returns true if (origin, seq) was already seen; records it otherwise
  bool seenOrRecord(uint8_t origin, uint8_t seq) {
    for (size_t i = 0; i < count_; ++i)
      if (entries_[i].origin == origin && entries_[i].seq == seq) return true;
    if (count_ < CAPACITY) entries_[count_++] = {origin, seq};
    return false;
  }
  size_t size() const { return count_; }
private:
  struct Entry { uint8_t origin, seq; };
  Entry entries_[CAPACITY];
  size_t count_ = 0;
};

// --- store-and-forward queue (§III-B) ---------------------------------------
// Holds foreign frames accepted during the listening phase, for retransmission
// in the node's own phase. Bounded by the regulatory fan-in (§V): 14
// descendants x 4 ports = 56 frames worst case.
class ForwardQueue {
public:
  static constexpr size_t CAPACITY   = 56;
  static constexpr size_t SLOT_BYTES = MAX_FRAME_LEN;

  void clear() { count_ = 0; }
  bool full()  const { return count_ >= CAPACITY; }
  size_t size() const { return count_; }

  // Store a received frame (header re-stamped by caller before TX).
  bool push(const uint8_t* frame, size_t len) {
    if (full() || len > SLOT_BYTES) return false;
    std::memcpy(slot_[count_], frame, len);
    len_[count_] = static_cast<uint16_t>(len);
    ++count_;
    return true;
  }
  const uint8_t* frameAt(size_t i, size_t& len) const {
    len = len_[i];
    return slot_[i];
  }
private:
  uint8_t  slot_[CAPACITY][SLOT_BYTES];
  uint16_t len_[CAPACITY];
  size_t   count_ = 0;
};

// --- forwarding: re-stamp the header before relaying (§III-C) ----------------
// The relay rewrites the sender-depth field to its own depth and decrements
// TTL; origin and seq are preserved end to end.
inline void restampForForward(uint8_t* frame, uint8_t myDepth) {
  MeshHeader h = unpackHeader(frame);
  h.depth = myDepth;
  h.ttl   = static_cast<uint8_t>(h.ttl - 1);
  packHeader(h, frame);
}

// --- cascade phase arithmetic (§III-B) ---------------------------------------
// The window [0, W) is split into H phases ordered by DECREASING depth:
// phase index p = H - d for a node of depth d (its TX phase);
// its listening phase is the one before its TX phase (deeper tier's TX).
struct PhasePlan {
  uint32_t txStart, txEnd;         // seconds from window start
  uint32_t listenStart, listenEnd; // zero-length if leaf has no listen duty
};

// phaseBounds: H phases with given per-phase durations (seconds), cumulative.
inline PhasePlan planFor(uint8_t myDepth, uint8_t H,
                         const uint32_t* phaseDur /*[H], deepest first*/) {
  PhasePlan p{0,0,0,0};
  uint32_t t = 0;
  // phases run deepest-first: phase i covers tier (H - i)
  for (uint8_t i = 0; i < H; ++i) {
    const uint8_t tier = static_cast<uint8_t>(H - i);
    const uint32_t start = t, end = t + phaseDur[i];
    if (tier == myDepth)                 { p.txStart = start;     p.txEnd = end; }
    if (tier == myDepth + 1)             { p.listenStart = start; p.listenEnd = end; }
    t = end;
  }
  return p;
}

} // namespace wmesh
