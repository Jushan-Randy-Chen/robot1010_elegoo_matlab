# ROBT 1010: ELEGOO Smart Robot Car V4 in MATLAB

MATLAB code for the ELEGOO Smart Robot Car V4 used in ROBT 1010 Intro to Robotics Engineering (RPI). It covers Wi-Fi control and camera streaming, and then mapping, localization and visual SLAM with the car's ultrasonic sensor, ESP32 camera and gyro.

## Repository layout

```
basic_wireless_control/   ElegooRobotCar client, dashboard, simple drive scripts (start here)
sensor_tests/             Sensor checks; ir_line_sensor/ tests the line-tracking IR sensors
edge_mapping/             Table-edge mapper UIs (IR + sonar)
camera_recording/         Save camera frames as a GIF or an MP4
control_simulations/      Feedback-control simulations (Live Scripts), no car needed
vslam_examples/           Simulator, student mapping template, visual SLAM demo
tools/                    makeStudentKit.m (student zip in dist/, not in git)
```

| Folder | Contents |
|---|---|
| [`basic_wireless_control/`](basic_wireless_control/) | `ElegooRobotCar` client class (all folders use it), dashboard UI, simple drive scripts, basic-control tutorials (on-off stop, turn-rate measurement, gyro bias, saturation) |
| [`sensor_tests/ir_line_sensor/`](sensor_tests/ir_line_sensor/) | Guided test of the three IR line sensors (motors off), live monitor, IR simulator, unit tests |
| [`edge_mapping/`](edge_mapping/) | `edgeHuggingMapperUI`, `tableMapperVideoUI` |
| [`camera_recording/`](camera_recording/) | `make_gif`, `make_video` |
| [`control_simulations/`](control_simulations/) | `sim_distance_control.mlx`: P-control of the distance to a wall |
| [`vslam_examples/`](vslam_examples/) | `elegooMappingTemplate.m` (5-task student template), `visualSlamDemo.m`, `ElegooSimCar.m` simulator, camera calibration |

## Requirements

- MATLAB R2024a or later (tested with R2026b) on Windows or macOS. MATLAB Online runs the simulator only, because it cannot reach the car's local Wi-Fi.
- Computer Vision Toolbox and Image Processing Toolbox (for `vslam_examples`)
- For the real car: an ELEGOO Smart Robot Car V4 with the course's custom firmware. `ElegooRobotCar` sends JSON commands over TCP port 100, and the stock firmware does not answer them. The firmware is not in this repository.

## Quick start

**Simulator (no car needed)**

```matlab
cd vslam_examples
% Open elegooMappingTemplate.m, set TASK = 1 and USE_SIMULATOR = true, then Run.
visualSlamDemo("sim")          % full visual SLAM demo
```

**Real car:** follow the checklist below.

## Real-car test checklist

Before you go offline:

- Start MATLAB while the computer still has internet (license check). Open this folder.
- Print the calibration board: `cd vslam_examples`, then `calibrateElegooCamera("print")`.
- Note the car's Wi-Fi name and password: `wifiName` and `wifiPassword` in the ESP32 sketch (`ElegooCarESP32S3.ino`).

On the car:

1. **Car:** batteries charged, both boards flashed, ESP32-UNO wires connected. Power ON, **motor switch OFF**.
2. **Wi-Fi:** join the car's network. Turn off any VPN. On a Mac, allow MATLAB in System Settings > Privacy & Security > Local Network. Check: `http://192.168.4.1/capture` shows a photo in a browser.
3. **Hardware check** (5 min). Answer its y/n questions. It asks before the motors turn and tells you when to lift the car.
   ```matlab
   cd basic_wireless_control
   report = checkCar      % prints SwapSides, PhotoVFlip, PhotoMirror
   save carCheck report
   ```
   Write the three printed values at the top of `ElegooRobotCar.m`. Run `checkCar` again until it says "The file is already correct".
4. **Drive:** run `elegooCarDashboard`. Try a short pulse in each direction and a timed drive (turn > 0 turns left). **Close the dashboard when you are done:** the car accepts one connection at a time.
5. **IR line sensors** (before line tracking or edge mapping): in `sensor_tests/ir_line_sensor/`, run `runIRSensorTest`.
6. **Obstacle stop:** face a wall 1 m away, then run `driveUntilObstacle(ElegooRobotCar("192.168.4.1"), 25, 60, 10)`.
7. **Camera calibration (optional for a first test):** in the `vslam_examples/` folder, run `calibrateElegooCamera(25)`. Use your measured square size in mm instead of 25.
8. **Template:** in `elegooMappingTemplate.m`, set `USE_SIMULATOR = false`. Run tasks 1 to 4 in order. Write the task 2 results in SETTINGS. Then run task 5 on 2.5 m x 2.5 m of clear floor with furniture around it.
9. **Bring back:** `carCheck.mat`, the task 2 values and the Task 5 figures.

To stop the car: press Ctrl+C in MATLAB. The motors stop 1 s after MATLAB stops sending. If they do not, lift the car and switch it off.

If the photos are colored stripes, charge the battery. For this and other problems, see [If something goes wrong](vslam_examples/README.md#if-something-goes-wrong).

## Student kit

Students get one flat folder with only the files the template needs:

```matlab
cd tools
makeStudentKit        % -> dist/ElegooMappingKit.zip
```

The kit has `elegooMappingTemplate.m`, `ElegooRobotCar.m`, `ElegooSimCar.m`, `simAssets/`, `calibrateElegooCamera.m`, `docs/images/` and `README.md`. The README of [vslam_examples](vslam_examples/README.md) is the only student guide. It uses ASD-STE100 Simplified Technical English. Students do each task in the simulator first, then on the real car. Before the first real-car session, do the real-car steps in that README.

## Safety

The mapping code moves the car in short stop-and-go steps. Every script that drives the car sets a 1 s motor watchdog, so the motors stop if MATLAB stops sending commands. Keep the car away from stairs and table edges.

## License

MIT, see [LICENSE](LICENSE). The simulator textures keep their own MIT No Attribution license.

## Credits

- Simulator textures in `vslam_examples/simAssets/` come from the [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) under the MIT No Attribution license (see `simAssets/LICENSE_aws-robomaker-small-house-world.txt`).
- The bookshelf in the simulator uses the book-cover images that ship with the Computer Vision Toolbox. They load at run time and are not in this repository.
