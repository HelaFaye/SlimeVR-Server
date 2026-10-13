# Synthetic tracker

Fake SlimeVR trackers driven by a posable body, for testing the server end to end with no
firmware and no hardware. It speaks the UDP tracker protocol directly and covers
discovery, handshake, sensor enumeration, tracker assignment, the skeleton solve and palm
forward kinematics.

Written for the I2SPI work in [HelaFaye/SlimeVR-Tracker-ESP](https://github.com/HelaFaye/SlimeVR-Tracker-ESP)
(chained sensors over RJ45), whose `sim/` covers everything below the network. The
hardware lives in [HelaFaye/SlimeVR-Tracker-ESP-Hardware](https://github.com/HelaFaye/SlimeVR-Tracker-ESP-Hardware).
Python 3 standard library only.

```sh
python3 tools/synthetic-tracker/synthetic_tracker.py --list-poses
python3 tools/synthetic-tracker/synthetic_tracker.py --pose walk --rate 100
python3 tools/synthetic-tracker/synthetic_tracker.py --host 192.168.1.20 --pose palm-twist
tools/synthetic-tracker/rom.sh [--host 127.0.0.1]
```

It stands up the **whole 18-point build**: five hub devices, each with its own UDP socket,
MAC and packet counter, carrying three sensors: the hub IMU as sensor 0, I2SPI nodes 1 and 2
as sensors 1 and 2. That makes fifteen sensors, the same topology as the firmware repo's
`docs/dev/EXAMPLE-18POINT-BUILD.md`.

## Wire format

Taken from `src/network/packets.h` and `connection.cpp` on the sending side, then checked
against `server/core/.../udp/UDPPacket.kt` here, which is the authoritative consumer. Header is
4 bytes of big-endian packet type followed by 8 of packet number; `Handshake` (3),
`SensorInfo` (15), `RotationData` (17) and `HeartBeat` (0) are implemented. Positions come
from `TrackerPosition.kt` ids.

## Poses

| Pose | What it is for |
|---|---|
| `t-pose` | Everything identity. Baseline for assignment |
| `arms-down` | Neutral standing |
| `sitting` | Hips and knees flexed. Exercises leg constraints |
| `walk` | Counter-swinging limbs. Foot and hip solve stability |
| `wave` | Right arm, palm leading the forearm |
| `palm-twist` | **The one worth running.** Only the palms rotate |

`palm-twist` is the diagnostic. If the palms turn and nothing else moves, the arm chain is
parented correctly and `forceArmsFromHMD` is doing what we think it is. If the forearms
swing with them, it isn't — which is the exact confusion that produced a wrong answer in
this project's own notes before it was caught.

## What it proves and what it doesn't

Proves: the server accepts the topology, fifteen sensors across five devices enumerate,
assignment works, the skeleton solves, and palm position is produced by FK from measured
arm segments.

Doesn't: any firmware behaviour whatsoever. It fabricates rotations; it does not run a
line of tracker code. Pair it with `run.sh`, which runs the real bus sources but no
network.

---

# Sensor power staging

`scenarioStagedPowerUp` and `scenarioNodeRefusesPower` cover the gated VCC rail. Sensors
come up with their pass MOSFET **off** (`SensorPowerDefaultOn = false`) and
`ATTinyCSBus::init()` brings them online one at a time, settling between each.

The simulation checks the property that matters: never more than one sensor ramping
simultaneously. Staging isn't cosmetic — a shorted sensor becomes attributable to a node
instead of taking the rail down anonymously, and inrush is spread over milliseconds
instead of summed, which at the far end of a metre of thin cable is the difference between
a droop and a brownout.

The gate polarity is also modelled: the P-channel FET is off when its gate is high, and
the gate has a pull-up, so an unprogrammed or held-in-reset ATtiny leaves its sensor
**unpowered**. That is the safe direction — an unpowered IMU cannot drive MISO and cannot
fight the bus.


---

# Full sweep (`sweep.sh`)

Two exhaustive passes, covering opposite halves of the system.

```sh
sim/sweep.sh                    # every topology, then range of motion offline
sim/sweep.sh --nodes 10         # 3^10 instead of 3^15, for a quick pass
sim/sweep.sh --host 127.0.0.1   # also stream ROM at a running server
```

## Topology: 3^15 configurations, exhaustive

Each of the 15 node ids is independently **absent, an SPI sensor, or an I2C sensor**, so
the space is 3^15 = **14,348,907** configurations. The sweep walks all of it — not a
sample, not pairwise. It takes about 150 seconds at ~94,000 configurations per second and
performs 143 million node polls.

Every I2C sensor in the sweep is deliberately given the **same address, 0x68**. That is
the case gating exists to make safe, so it is the case worth sweeping rather than avoiding.

Per configuration it checks:

- exactly one chip select asserted at any moment, never two
- a poll reaches the sensor it addressed and no other
- I2C sensors sharing an address never both answer
- absent nodes report absent rather than returning plausible data
- nothing answers at 0x68 while the chain is disarmed

Exhaustive was worth doing because it was cheap. If it ever stops being cheap, sample —
but check the timing before assuming, which is how this ended up exhaustive.

## Range of motion

Two poses, both coverage-oriented rather than realistic:

| Pose | What it does |
|---|---|
| `rom` | Each joint through its full range, one axis at a time, one part at a time |
| `rom-all` | Every joint sweeping simultaneously at mutually offset rates |

`rom-all` is the harsher of the two: it puts the whole skeleton into unusual combinations
at once, which is where inter-bone constraints and proportion assumptions break rather
than any single joint. `rom` is the one to watch if you want to see which specific joint
misbehaves.

Ranges are declared per part in `synthetic_tracker.py`'s `ROM` table — wide enough to
reach the constraint limits and the gimbal-adjacent regions, not anatomical gospel. The
sweep uses a triangle wave rather than a sine so the extremes are held briefly instead of
passed through instantaneously.

Without `--host` it validates the generator itself: 120,000 quaternions per pose, all
checked unit-length. With `--host` it streams at a real server. Measured end to end
through a loopback listener, the worst quaternion norm error across a full ROM stream is
4.1e-08.
