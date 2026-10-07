"""Summarise a run log and verify event order: RECOVERY_CONFIRMED(k) precedes TARGET_GENERATED(k+1)."""
import json
import re
import sys

evs = []
for line in open(sys.argv[1], encoding="utf-8"):
    m = re.search(r"(\{.*\})\s*$", line)
    if m:
        try:
            evs.append(json.loads(m.group(1)))
        except json.JSONDecodeError:
            pass

order = [(e["ev"], e["cycle"]) for e in evs if e["ev"] in
         ("HOME_CONFIRMED", "TARGET_GENERATED", "REACH_CONFIRMED", "RECOVERY_STARTED", "RECOVERY_CONFIRMED", "FAILED")]
expect = ["TARGET_GENERATED", "REACH_CONFIRMED", "RECOVERY_STARTED", "RECOVERY_CONFIRMED"]
ok = bool(order) and order[0][0] == "HOME_CONFIRMED"
problems = []
k_done = 0
seq = order[1:]
i = 0
while i < len(seq):
    k = seq[i][1]
    chunk = [e for e, c in seq[i:i + 4]]
    cyc = [c for e, c in seq[i:i + 4]]
    if chunk == expect and all(c == k for c in cyc):
        if k != k_done + 1:
            problems.append(f"cycle numbering gap at {k}")
        k_done = k
        i += 4
    elif "FAILED" in chunk:
        problems.append(f"FAILED in cycle {k}")
        break
    else:
        # last cycle may be cut by the user stop (incomplete, not an order violation)
        if i + len(chunk) == len(seq) and chunk == expect[:len(chunk)]:
            break
        problems.append(f"order violation at cycle {k}: {chunk}")
        break
ok = ok and not problems

reach = [e for e in evs if e["ev"] == "REACH_CONFIRMED"]
rec = [e for e in evs if e["ev"] == "RECOVERY_CONFIRMED"]
gen = [e for e in evs if e["ev"] == "TARGET_GENERATED"]
rej = {}
for e in evs:
    if e["ev"] == "CANDIDATE_REJECTED":
        rej[e["reason"]] = rej.get(e["reason"], 0) + 1
ncand = sum(e.get("candidates", 1) for e in gen)
start = next((e for e in evs if e["ev"] == "START"), {})
shut = next((e for e in evs if e["ev"] == "SHUTDOWN"), {})
summary = {
    "order_ok": ok, "problems": problems, "seed": start.get("seed"), "completed_cycles": k_done,
    "targets_generated": len(gen), "failed": any(e["ev"] == "FAILED" for e in evs),
    "reach_pos_err_max_mm": round(max((e["pos_err"] for e in reach), default=0) * 1000, 3),
    "reach_ori_err_max_deg": round(max((e["ori_err_deg"] for e in reach), default=0), 3),
    "reach_qd_max": max((e["max_qd"] for e in reach), default=0),
    "recover_q_err_max_rad": max((e["max_q_err"] for e in rec), default=0),
    "recover_qd_max": max((e["max_qd"] for e in rec), default=0),
    "torque_sat_steps_total": sum(e.get("sat_steps", 0) for e in reach + rec),
    "candidates_total": ncand, "acceptance_rate": round(len(gen) / ncand, 3) if ncand else None,
    "reject_reasons": rej, "gen_wall_max_s": max((e.get("gen_wall", 0) for e in gen), default=0),
    "sim_t_end": shut.get("sim_t"), "shutdown": {k: shut.get(k) for k in ("reason", "exit_code", "close_s")},
}
print(json.dumps(summary, indent=2))
sys.exit(0 if ok else 1)
