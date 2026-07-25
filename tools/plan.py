#!/usr/bin/env python3
# W-Mesh network planner — from a network description to per-node provisioning.
#
# Input: a JSON plan (see examples/livingston.json): the window anchor, the
# node list with parent links, and sizing goals (margins). Output:
#   1. a validation report — every rule the firmware and the design impose,
#      checked across the WHOLE network (things no single node can check
#      about itself: fan-in, slot uniqueness, channel-per-subtree, whitelist
#      completeness);
#   2. a ready-to-run enroll script: one tools/enroll.py line per node.
#
# The planner derives everything derivable so the human specifies only the
# topology and the goals:
#   depth        <- parent chain (parent 0 = the gateway)
#   channel      <- one per tier-1 subtree, round-robin from `channels`
#   whitelist    <- ALL DESCENDANTS of a relay (the firmware filters on the
#                   ORIGIN id, so a relay must admit its whole subtree,
#                   not just its direct children)
#   psub/ppsub   <- beacon sub-slots per tier, parent's sub-slot for children
#   slot/slotdur <- per-tier slots; slotdur sized to the tier's heaviest
#                   node (own + forwarded airtime) x margin
#   phases       <- per-tier durations from the slotted load x margin,
#                   deepest first (or taken verbatim if given)
#
# Rounding is deliberately upward everywhere: the slack absorbs a drifted
# clock or a late frame lands in slack, not on a neighbour.
#
# Usage:
#   python3 tools/plan.py examples/livingston.json            # report only
#   python3 tools/plan.py examples/livingston.json -o out.sh  # + enroll script
#
# SPDX-License-Identifier: MIT
import argparse
import json
import math
import sys

T_STD = 0.6   # s, final-hop frame airtime (standard preamble), rounded up
T_EXT = 0.8   # s, inter-tier frame airtime (CAD preamble), rounded up
FANIN = {1: 14}          # regulatory descendants cap: 14 at tier 1...
FANIN_DEEP = 10          # ...10 deeper (forwards carry the CAD preamble)
MAX_WHITELIST = 14       # config image capacity


def fail(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def load(path):
    with open(path) as f:
        plan = json.load(f)
    for req in ("window", "nodes"):
        if req not in plan:
            fail(f"plan is missing '{req}'")
    return plan


def build(plan):
    nodes = {n["id"]: dict(n) for n in plan["nodes"]}
    if len(nodes) != len(plan["nodes"]):
        fail("duplicate node ids")
    # depth from parent chain
    for n in nodes.values():
        d, seen, cur = 0, set(), n
        while cur["parent"] != 0:
            if cur["id"] in seen:
                fail(f"parent loop at node {n['id']}")
            seen.add(cur["id"])
            if cur["parent"] not in nodes:
                fail(f"node {cur['id']} has unknown parent {cur['parent']}")
            cur = nodes[cur["parent"]]
            d += 1
        n["depth"] = d + 1
    H = max(n["depth"] for n in nodes.values())

    # children / descendants
    for n in nodes.values():
        n["children"] = [m["id"] for m in nodes.values() if m["parent"] == n["id"]]

    def descendants(nid):
        out = []
        for c in nodes[nid]["children"]:
            out.append(c)
            out.extend(descendants(c))
        return out
    for n in nodes.values():
        n["desc"] = descendants(n["id"])

    # roles sanity
    for n in nodes.values():
        if n["children"] and n["profile"] != "relay":
            fail(f"node {n['id']} has children but profile '{n['profile']}'")

    # channel per tier-1 subtree
    channels = plan.get("channels", [868100000, 868300000, 868500000,
                                     868700000, 867100000, 867300000,
                                     867500000, 867700000])
    tier1 = sorted(n["id"] for n in nodes.values() if n["depth"] == 1)
    if len(tier1) > len(channels):
        fail(f"{len(tier1)} tier-1 subtrees but only {len(channels)} channels")
    chan = {}
    for i, t in enumerate(tier1):
        chan[t] = channels[i]
        for d in nodes[t]["desc"]:
            chan[d] = channels[i]
    for n in nodes.values():
        n["freq"] = chan[n["id"]]

    # beacon sub-slots: relays per tier get 0,1,2...; children listen at parent's
    preroll = int(plan.get("preroll_slot", 5))
    for depth in range(1, H + 1):
        relays = sorted(n["id"] for n in nodes.values()
                        if n["depth"] == depth and n["profile"] == "relay")
        for i, r in enumerate(relays):
            nodes[r]["psub"] = i
            if i >= preroll:
                fail(f"tier {depth} has {len(relays)} relays but the pre-roll "
                     f"interval holds only {preroll} sub-slots — raise preroll_slot")
    for n in nodes.values():
        n["ppsub"] = 0 if n["parent"] == 0 else nodes[n["parent"]].get("psub", 0)

    # airtime per node: own ports + everything it forwards
    m_slot = float(plan.get("slot_margin", 2.0))
    for n in nodes.values():
        T = T_STD if n["depth"] == 1 else T_EXT
        own = n.get("ports", 3) * T
        fwd = sum(nodes[d].get("ports", 3) for d in n["desc"]) * T
        n["airtime"] = own + fwd

    # per-tier slots: slotdur = heaviest node in tier x margin, rounded up
    for depth in range(1, H + 1):
        tier = sorted((n for n in nodes.values() if n["depth"] == depth),
                      key=lambda n: -n["airtime"])   # heaviest first: slot 0
        dur = max(3, math.ceil(m_slot * max(n["airtime"] for n in tier)))
        for i, n in enumerate(tier):
            n["slot"], n["slotdur"] = i, dur

    # phases: deepest first, sized to the tier's slotted load x margin
    if isinstance(plan.get("phases"), list):
        phases = [int(x) for x in plan["phases"]]
        if len(phases) != H:
            fail(f"phases has {len(phases)} entries but the network depth is {H}")
    else:
        m_phase = float(plan.get("phase_margin", 1.5))
        phases = []
        for depth in range(H, 0, -1):
            tier = [n for n in nodes.values() if n["depth"] == depth]
            need = max(n["slot"] for n in tier) * tier[0]["slotdur"] + \
                   max(n["slotdur"] for n in tier)
            phases.append(max(30, math.ceil(m_phase * need)))
    return nodes, H, phases, preroll


def validate(nodes, H, phases, preroll, plan):
    errors, warnings = [], []
    win_total = sum(phases) + H * preroll
    for n in nodes.values():
        cap = FANIN.get(n["depth"], FANIN_DEEP)
        if n["profile"] == "relay" and len(n["desc"]) > cap:
            errors.append(f"relay {n['id']} (tier {n['depth']}) has "
                          f"{len(n['desc'])} descendants > fan-in cap {cap}")
        if n["profile"] == "relay" and len(n["desc"]) > MAX_WHITELIST:
            errors.append(f"relay {n['id']}: whitelist needs {len(n['desc'])} "
                          f"entries, config holds {MAX_WHITELIST}")
        if n["depth"] > 15:
            errors.append(f"node {n['id']}: depth {n['depth']} exceeds 15")
        # slot fits its phase (phases list is deepest-first)
        phase = phases[H - n["depth"]]
        if (n["slot"] + 1) * n["slotdur"] > phase:
            errors.append(f"node {n['id']}: slot {n['slot']}x{n['slotdur']}s "
                          f"exceeds its {phase}s phase")
    # duty cycle: relay TX seconds per window <= 36 s/h budget
    for n in nodes.values():
        if n["airtime"] > 36:
            errors.append(f"node {n['id']}: {n['airtime']:.0f}s TX per window "
                          f"exceeds the 36 s/h duty budget")
    if win_total > 900:
        warnings.append(f"window needs {win_total}s > the 900s reference window")
    return errors, warnings, win_total


def emit(nodes, H, phases, preroll, plan, out):
    window = plan["window"]
    lines = ["#!/bin/sh", "# generated by tools/plan.py — one line per node;",
             "# replace PORT with the node's serial device, run, reset the node.",
             "set -e", ""]
    for n in sorted(nodes.values(), key=lambda n: (n["depth"], n["slot"])):
        cmd = (f"python3 tools/enroll.py PORT --id {n['id']} "
               f"--profile {n['profile']} --depth {n['depth']} "
               f"--phases {','.join(str(p) for p in phases)} "
               f"--preroll {preroll} --slot {n['slot']} --slotdur {n['slotdur']} "
               f"--ports {n.get('ports', 3)} --freq {n['freq']} "
               f"--window {window} --ppsub {n['ppsub']}")
        if n["profile"] == "relay":
            cmd += f" --psub {n['psub']} --whitelist {','.join(str(d) for d in n['desc'])}"
            if n.get("gnssbk"):
                cmd += " --gnssbk 1"
        cmd += " --boot"
        name = n.get("name", "")
        lines.append(f"# {name} (tier {n['depth']}, "
                     f"{len(n['desc'])} descendants)" if n["profile"] == "relay"
                     else f"# {name} (tier {n['depth']})")
        lines.append(cmd)
        lines.append("")
    with open(out, "w") as f:
        f.write("\n".join(lines))


def timeline(nodes, H, phases, preroll, out_png):
    """One window as a Gantt chart: every node's beacon, listen and TX slots."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    t0 = H * preroll                      # uplink phases start after the pre-roll
    def phase_start(depth):               # phases list is deepest-first
        return t0 + sum(phases[:H - depth])

    chans = sorted(set(n["freq"] for n in nodes.values()))
    cmap = plt.get_cmap("tab10")
    ccolor = {c: cmap(i % 10) for i, c in enumerate(chans)}

    rows = sorted(nodes.values(), key=lambda n: (n["freq"], n["depth"], n["slot"]))
    fig, ax = plt.subplots(figsize=(11, 0.34 * len(rows) + 1.8), dpi=150)
    for y, n in enumerate(rows):
        col = ccolor[n["freq"]]
        # beacon: RX tick at the parent's sub-slot; relays also TX their own copy
        ax.add_patch(Rectangle(((n["depth"] - 1) * preroll + n["ppsub"], y + 0.2),
                               0.8, 0.6, fc="#b03030", ec="none"))
        if n["profile"] == "relay":
            ax.add_patch(Rectangle((n["depth"] * preroll + n.get("psub", 0), y + 0.2),
                                   0.8, 0.6, fc="#b03030", ec="none", alpha=0.55))
            # listening: the whole phase of the children's tier, hatched
            if n["depth"] < H:
                ls = phase_start(n["depth"] + 1)
                ax.add_patch(Rectangle((ls, y + 0.2), phases[H - n["depth"] - 1], 0.6,
                                       fc="none", ec=col, lw=0.7, hatch="////", alpha=0.6))
        # TX slot: light = allocated slot, dark = actual airtime
        s = phase_start(n["depth"]) + n["slot"] * n["slotdur"]
        ax.add_patch(Rectangle((s, y + 0.15), n["slotdur"], 0.7, fc=col, alpha=0.25, ec="none"))
        ax.add_patch(Rectangle((s, y + 0.15), n["airtime"], 0.7, fc=col, alpha=0.95, ec="none"))
    # phase boundaries
    x = t0
    ax.axvline(0, color="0.3", lw=0.8)
    ax.text(t0 / 2, len(rows) + 0.35, "pre-roll" + chr(10) + "(beacons)", ha="center", fontsize=7)
    for i, ph in enumerate(phases):
        ax.axvline(x, color="0.5", lw=0.7, ls="--")
        ax.text(x + ph / 2, len(rows) + 0.35, f"phase {i + 1}" + chr(10) + f"(tier {H - i} TX)",
                ha="center", fontsize=7)
        x += ph
    ax.axvline(x, color="0.3", lw=0.8)
    ax.set_yticks([y + 0.5 for y in range(len(rows))])
    ax.set_yticklabels([f"{n.get('name', n['id'])} d{n['depth']} "
                        f"{n['freq'] / 1e6:.1f}MHz" for n in rows], fontsize=6.5)
    ax.set_xlabel("seconds from window start (beacon = t0)", fontsize=8)
    ax.set_xlim(-1, x + 3)
    ax.set_ylim(0, len(rows) + 1.4)
    ax.set_title("W-Mesh window timeline: beacon (red), listen (hatched), "
                 "TX slot (light) and actual airtime (solid)", fontsize=8.5)
    ax.tick_params(axis="x", labelsize=7)
    fig.tight_layout()
    fig.savefig(out_png)


def main():
    p = argparse.ArgumentParser(description="W-Mesh network planner")
    p.add_argument("plan", help="JSON network description")
    p.add_argument("-o", "--out", help="write the enroll script here")
    p.add_argument("-t", "--timeline", help="write a window-timeline PNG here")
    a = p.parse_args()
    plan = load(a.plan)
    nodes, H, phases, preroll = build(plan)
    errors, warnings, win_total = validate(nodes, H, phases, preroll, plan)

    print(f"network: {len(nodes)} nodes, depth H={H}, "
          f"phases (deepest first) = {phases}, pre-roll {H}x{preroll}s")
    print(f"window load: {win_total}s of the 900s window "
          f"({100 * win_total / 900:.0f}%)")
    for depth in range(1, H + 1):
        tier = [n for n in nodes.values() if n["depth"] == depth]
        print(f"  tier {depth}: {len(tier)} nodes, slotdur {tier[0]['slotdur']}s, "
              f"channels {sorted(set(n['freq'] for n in tier))}")
    for w in warnings:
        print(f"WARNING: {w}")
    for e in errors:
        print(f"ERROR: {e}")
    if errors:
        sys.exit(1)
    print("plan OK")
    if a.out:
        emit(nodes, H, phases, preroll, plan, a.out)
        print(f"enroll script written to {a.out}")
    if a.timeline:
        timeline(nodes, H, phases, preroll, a.timeline)
        print(f"timeline written to {a.timeline}")


if __name__ == "__main__":
    main()
