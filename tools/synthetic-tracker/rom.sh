#!/usr/bin/env bash
# Range-of-motion pass for the synthetic tracker. Split out of the firmware repo's
# sim/sweep.sh; the topology half of that sweep stays there.
#
#   tools/synthetic-tracker/rom.sh                    # validate the pose generator, no server
#   tools/synthetic-tracker/rom.sh --host 127.0.0.1   # also stream ROM at a running server
set -uo pipefail
cd "$(dirname "$0")"

HOST=""
RATE=100
DURATION=0
while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST=$2; shift 2;;
    --rate) RATE=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    *) echo "unknown option: $1"; exit 2;;
  esac
done

fail=0
echo "=== Range of motion ==="
# Each ROM pose visits every joint's extremes; rom-all combines them, which is where
# inter-bone constraints break rather than any single joint.
for pose in rom rom-all; do
  echo "--- $pose ---"
  if [ -n "$HOST" ]; then
    dur=${DURATION:-0}
    [ "$dur" = "0" ] && dur=95     # one full cycle of pose_rom: 15 parts x 3 axes x 2 s
    python3 synthetic_tracker.py --host "$HOST" --pose "$pose" \
      --rate "$RATE" --duration "$dur" || fail=1
  else
    # No server: validate the pose generator itself rather than pretending to stream.
    python3 - "$pose" <<'PY' || fail=1
import math, sys
sys.path.insert(0, ".")
import synthetic_tracker as st

pose = sys.argv[1]
fn = st.POSES[pose]
samples = 0
extremes = {}
for i in range(8000):
    t = i * 0.0125
    rot = st.build_rotations(fn, t)
    for part, q in rot.items():
        n = math.sqrt(sum(c * c for c in q))
        assert abs(n - 1.0) < 1e-9, f"{part} quaternion not unit: {n}"
        samples += 1
    for part, ang in fn(t).items():
        lo, hi = extremes.get(part, (999.0, -999.0))
        m = max(abs(a) for a in ang)
        extremes[part] = (min(lo, m), max(hi, m))
print(f"  {samples} quaternions, all unit length")
print(f"  {len(extremes)} parts driven; peak excursions "
      f"{min(v[1] for v in extremes.values()):.0f}-"
      f"{max(v[1] for v in extremes.values()):.0f} deg")
PY
  fi
done

echo
[ $fail -eq 0 ] && echo "ROM PASSED" || echo "FAILURES ABOVE"
exit $fail
