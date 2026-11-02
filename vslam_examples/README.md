# Mapping, localization and visual SLAM examples

Mapping and localization with the car's own sensors: ultrasonic sensor, ESP32 camera and gyro. Everything runs in a pure-MATLAB simulator first, then on the real car.

| File | What it is |
|---|---|
| `elegooMappingTemplate.m` | **Student template.** Five tasks in one script, with comments in ASD-STE100 Simplified Technical English. Set `TASK` and `USE_SIMULATOR`, then click Run. |
| `visualSlamDemo.m` | Full visual SLAM demo with a dashboard (`monovslam`), replay and MP4 export. |
| `ElegooSimCar.m` | Simulator: a furnished 5 m x 4 m living room. It has the same commands as `ElegooRobotCar`. |
| `simAssets/` | Room textures for the simulator (AWS RoboMaker Small House World, MIT-0). |
| `calibrateElegooCamera.m` | One-time checkerboard calibration of the ESP32 camera. |
| `STUDENT_GUIDE.md`, `STUDENT_GUIDE.html` | **Student guide** (the HTML file is generated from the Markdown by `tools/build_guide_html.py`, with the figures embedded, so students can open it in any browser): each task, each labeled figure with an example image and how to read it, the expected results, a results table and a glossary. |
| `STUDENT_README.txt` | Short student instructions. In the student kit it becomes `README.txt`. |
| `docs/images/` | Example figures from the simulator, used by the guide. |

## The five tasks in `elegooMappingTemplate.m`

| Task | Car motion | Position from | Map from |
|---|---|---|---|
| 1. Read the sensors | One 0.8 s turn | – | – |
| 2. Calibrate the motion | up to 8 steps forward and back, then ~400° in place | Sonar (step length), camera (turn angle) | – |
| 3. Sonar map from one spot | None (servo scan) | Known | Sonar occupancy grid |
| 4. Square + dead reckoning | Square, camera-feedback turns | Step count + camera visual odometry | Sonar occupancy grid |
| 5. Visual SLAM | Map-start sweep, ~1.25 circles, scale drive | `monovslam` (loop closure) | Camera map points, sonar scale |

Every figure has a label such as **Task 3, Figure 3.1**, and its panels are lettered (a), (b) and so on. The guide explains each one: [STUDENT_GUIDE.md](STUDENT_GUIDE.md).

## Quick start

```matlab
cd vslam_examples
% In elegooMappingTemplate.m set TASK = 1 and USE_SIMULATOR = true, then:
elegooMappingTemplate

visualSlamDemo("sim")      % full demo in the simulator
visualSlamDemo()           % full demo on the real car
```

The vSLAM files find `ElegooRobotCar.m` in `../basic_wireless_control` automatically.

## Simulator results (MATLAB R2026b)

- Task 2: 5.0–5.2 cm per step and 16–18° per turn step (it changes each run).
- Task 4: the square almost closes; the calculated path stays within about 10 cm of the true path.
- Task 5: the map starts after 220–250 photos. Complete runs have 61–67 key frames and the loop closes; the path is 20–40 cm RMS from the truth and the sonar scale error is −40% to +20% (it changes each run). About 2 of 3 runs complete (4 of 6 in our tests, as for visualSlamDemo); the others stop early with lost tracking, which the template reports as "Task 5 stopped early".

These results are from the simulator only. The code is not yet tested on the real car.

## Before students use the real car (TA checklist)

The simulator always answers, has no Wi-Fi delay, and cannot show some failures. Check these on one car first, with the wheels off the floor where it says so:

1. **Reply formats.** Close the dashboard and any other connection first (the ESP32 bridge likely serves one TCP client). Then check `car.readDistance()` with nothing in range and with a hand at 1 cm: is the field `distance` in cm, and what is the "no echo" value (0, 150 or more)? The template treats 0 and a missing value as "no value" and stops task 5 after 3 in a row. Also check `car.readIMU().gyro` (size, units) and `size(car.capture())` (the template prints the photo size when it connects).
2. **Watchdog (wheels lifted).** `car.drive(100,100)` and send nothing: do the wheels stop after about 1 s? Repeat while calling `readDistance` in a loop: do non-drive commands reset the watchdog? Press Ctrl+C during task 4: the template's `onCleanup` should stop the wheels at once.
3. **Pulse size and latency.** `tic; car.ping; toc` a few times, then `stepCar`-size pulses on the course floor: does `S.turnLeft = [-130 130 0.10]` turn the car at all on carpet? Wi-Fi delay adds to every pulse.
4. **Camera.** Is it upside down (set `S.flipImage`)? Is it 640 x 480? Take a photo right after a turn step and 1 s later: if they differ, the ESP32 returns a stale frame and every camera turn lags one step.
5. **`S.circle`.** Drive 20 circle steps and check that the car curves left, and how many degrees per step. Task 5 stops with an error if it does not turn left after 40 circle photos.
6. **Car network names.** Label each car with its `ELEGOO-...` network name, so students connect to their own car.

## Rebuild the student kit

```matlab
% after any change of STUDENT_GUIDE.md:  python tools/build_guide_html.py   (needs: pip install markdown)
cd tools
makeStudentKit        % -> dist/ElegooMappingKit.zip (no top folder: one folder after unzip)
```
