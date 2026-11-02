# Mapping, localization and visual SLAM examples

Mapping and localization with the car's own sensors: ultrasonic sensor, ESP32 camera and gyro. Everything runs in a pure-MATLAB simulator first, then on the real car.

| File | What it is |
|---|---|
| `elegooMappingTemplate.m` | **Student template.** Five tasks in one script, with comments in ASD-STE100 Simplified Technical English. Set `TASK` and `USE_SIMULATOR`, then click Run. |
| `visualSlamDemo.m` | Full visual SLAM demo with a dashboard (`monovslam`), replay and MP4 export. |
| `ElegooSimCar.m` | Simulator: a furnished 5 m x 4 m living room. It has the same commands as `ElegooRobotCar`. |
| `simAssets/` | Room textures for the simulator (AWS RoboMaker Small House World, MIT-0). |
| `calibrateElegooCamera.m` | One-time checkerboard calibration of the ESP32 camera. |
| `STUDENT_GUIDE.md` | **Student guide:** each task, each labeled figure with an example image and how to read it, the expected results, a results table and a glossary. |
| `STUDENT_README.txt` | Short student instructions. In the student kit it becomes `README.txt`. |
| `docs/images/` | Example figures from the simulator, used by the guide. |

## The five tasks in `elegooMappingTemplate.m`

| Task | Car motion | Position from | Map from |
|---|---|---|---|
| 1. Read the sensors | One 0.8 s turn | – | – |
| 2. Calibrate the motion | 8 steps forward and back, then ~400° in place | Sonar (step length), camera (turn angle) | – |
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

- Task 2: 5.1 cm per step and 17.2° per turn step (the simulator's true value is about 17.5°).
- Task 4: the calculated path stays within about 7 cm of the true path.
- Task 5: the map starts after 220–250 photos. Complete runs have 63–67 key frames, the loop closes, the path is 25–35 cm RMS from the truth, and the sonar scale error is −15% to +20% (it changes each run). Two of six runs stopped early with lost tracking before the "lost again here: drive on" rule was restored; the two runs after it completed.

These results are from the simulator only. The code is not yet tested on the real car.
