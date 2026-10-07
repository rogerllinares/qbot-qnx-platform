# QBot Platform on QNX

A real-time C API for the **Quanser QBot** mobile robot, running on **QNX Neutrino**, plus a LiDAR-triggered
obstacle-avoidance manoeuvre built on top of it. Bachelor's thesis (TFG) project, Industrial Electronics and
Automation Engineering — ESEIAAT, Universitat Politècnica de Catalunya (UPC), 2026.

The work adapts an earlier robot-control setup built around a Festo Robotino to the QBot, and was tested on the
real robot in the university lab.

## What it does

- **`apibasica` library** — the only part of the program that talks to the robot. An internal thread
  (`SCHED_RR`, priority 50) reads the Quanser driver over TCP (`:18888`) every 50 ms and keeps the robot state
  behind a mutex. The application only uses getters/setters: wheel positions and speeds, IMU, battery, body
  velocity (`v`, `ω`) and odometry.
- **Events over a POSIX message queue** — `initSystem()` takes a bitmask of the events the application wants
  (low battery, LiDAR obstacle) and returns an `mq` descriptor; the application blocks on it instead of polling.
- **LiDAR inside the library** — a second internal thread reads a TCP bridge (`:18889`) that runs on the robot's
  Jetson Orin Nano and sends one line per scan with the closest distance in a front sector. Below the threshold
  it posts `LIDAR_OBSTACLE` once, with hysteresis so the same obstacle is not reported twice.
- **Clean start and stop** — if any step of `initSystem()` fails, everything opened so far is closed again;
  `closeSystem()` sends the stop frame, shuts the sockets down and joins both threads.
- **P4 manoeuvre (`QBot_P4.c`)** — an event-driven Supervisor plus a periodic Monitor (priority 15, 50 ms):
  drive forward, and when the LiDAR sees an obstacle at 0.40 m, turn 90° left, arc 180° around it and turn 90°
  back, leaving on the original heading.

## Architecture

```
Jetson Orin Nano (Ubuntu)                      QNX Neutrino
-------------------------                      ------------
M10P --USB--> pont_lidar.py --TCP :18889-->  LiDAR thread  --mq_send(LIDAR_OBSTACLE)--+
QBot   --Quanser driver ----TCP :18888-->  robot thread  --mq_send(BATERIA_BAIXA)---+
                                                                                      v
                                                     application (Supervisor + Monitor)
```

Diagrams (in Catalan, as in the thesis): [layers](docs/Capes.svg) · [topology](docs/Topologia.svg) ·
[manoeuvre states](docs/Estats.svg) · [sequence](docs/Sequencia.svg).

## Repository layout

| Path | Contents |
|---|---|
| `src/apibasica.h`, `src/apibasica1.c` | The library: public interface and implementation |
| `src/QBot_P4.c` | LiDAR-triggered obstacle-avoidance manoeuvre |
| `src/test_lidar_api.c` | Listens to the event queue for 60 s and prints every LiDAR event (does not move the robot) |
| `src/QBot_Platform.c` | Minimal Supervisor + Monitor demo |
| `src/test_api.c`, `src/test_odometria.c` | Test bench for the API and the odometry |
| `jetson/pont_lidar.py` | LiDAR bridge on the Jetson: M10P → closest distance in the front sector → TCP |
| `data/` | Lab logs of five P4 runs (18 Sep 2026) |

Code comments are in Catalan/Spanish.

## Build and run

Cross-compiled from Windows with **QNX Momentics IDE** (QNX SDP) to a QNX Neutrino target. Each program is one
project: its main `.c` file + `apibasica1.c` + `apibasica.h`, linked with `-lm`.

1. On the Jetson: `python3 pont_lidar.py --desfase 92 --escala 1.0`
2. On QNX, as root, with the robot switched on: `./test_lidar_api` to check the events, then `./QBot_P4` with
   free space in front of the robot.

The robot IP (`QBOT_IP`) is set at the top of each program: read it from the robot's LCD.

## Status

Tested on the real robot in the lab: the bridge on 17 Sep 2026 (92° offset, scale 1.0, ~12 Hz) and the full v7
(the LiDAR inside the library + P4) on 18 Sep 2026. Known limitation: the LiDAR scans a single plane, so it does
not see obstacles lower than that plane.

## Credits

The starting point is the QBot example code provided by Roberto at the UPC lab; this repository is the API,
threading model, LiDAR integration and applications built from it. Published with permission.

## License

MIT, see [LICENSE](LICENSE).
