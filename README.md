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
tools/                    makeStudentKit.m (student zip in dist/, not in git), build_guide_html.py
```

| Folder | Contents |
|---|---|
| [`basic_wireless_control/`](basic_wireless_control/) | `ElegooRobotCar` client class (all folders use it), dashboard UI, simple drive scripts |
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

**Real car**

1. Join the car's Wi-Fi network (`ELEGOO-...`). The car is at `192.168.4.1`.
2. Run `elegooCarDashboard` in `basic_wireless_control/`, or set `USE_SIMULATOR = false` in the template.
3. Before line tracking or edge mapping, run `runIRSensorTest` in `sensor_tests/ir_line_sensor/` (it also runs in a simulator).

## Student kit

Students get one flat folder with only the files the template needs:

```matlab
cd tools
makeStudentKit        % -> dist/ElegooMappingKit.zip
```

The kit has `elegooMappingTemplate.m`, `ElegooRobotCar.m`, `ElegooSimCar.m`, `simAssets/`, `calibrateElegooCamera.m`, the student guide (`STUDENT_GUIDE.html` with embedded figures, `STUDENT_GUIDE.md`, `docs/images/`) and a short `README.txt`, all in ASD-STE100 Simplified Technical English. Students run each task in the simulator first, then on the real car. After a change of `STUDENT_GUIDE.md`, run `python tools/build_guide_html.py` (needs `pip install markdown`) before `makeStudentKit`. See the TA checklist in [vslam_examples/README.md](vslam_examples/README.md) before the first real-car session.

## Safety

The mapping code moves the car in short stop-and-go steps. Every script that drives the car sets a 1 s motor watchdog, so the motors stop if MATLAB stops sending commands. Keep the car away from stairs and table edges.

## License

MIT, see [LICENSE](LICENSE). The simulator textures keep their own MIT No Attribution license.

## Credits

- Simulator textures in `vslam_examples/simAssets/` come from the [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) under the MIT No Attribution license (see `simAssets/LICENSE_aws-robomaker-small-house-world.txt`).
- The bookshelf in the simulator uses the book-cover images that ship with the Computer Vision Toolbox. They load at run time and are not in this repository.
