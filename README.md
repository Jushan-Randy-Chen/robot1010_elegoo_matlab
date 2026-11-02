# ROBT 1010: ELEGOO Smart Robot Car V4 in MATLAB

MATLAB code for the ELEGOO Smart Robot Car V4 used in ROBT 1010 Intro to Robotics Engineering (RPI). It covers Wi-Fi control and camera streaming, and then mapping, localization and visual SLAM with the car's ultrasonic sensor, ESP32 camera and gyro.

## Repository layout

```
basic_wireless_control/   Drive the car over Wi-Fi and stream the camera (start here)
vslam_examples/           Simulator, student mapping template, visual SLAM demo
tools/makeStudentKit.m    Builds the self-contained student zip (dist/, not in git)
```

| Folder | Contents |
|---|---|
| [`basic_wireless_control/`](basic_wireless_control/) | `ElegooRobotCar` client class, dashboard UI, simple drive and recording scripts |
| [`vslam_examples/`](vslam_examples/) | `elegooMappingTemplate.m` (5-task student template), `visualSlamDemo.m`, `ElegooSimCar.m` simulator, camera calibration |

## Requirements

- MATLAB R2024a or later (tested with R2026b)
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

## Student kit

Students get one flat folder with only the files the template needs:

```matlab
cd tools
makeStudentKit        % -> dist/ElegooMappingKit.zip
```

The kit has `elegooMappingTemplate.m`, `ElegooRobotCar.m`, `ElegooSimCar.m`, `simAssets/`, `calibrateElegooCamera.m` and a `README.txt` in ASD-STE100 Simplified Technical English. Students run each task in the simulator first, then on the real car.

## Safety

The car moves in short stop-and-go steps, and every script sets a 1 s motor watchdog. Keep the car away from stairs and table edges.

## Credits

- Simulator textures in `vslam_examples/simAssets/` come from the [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) under the MIT No Attribution license (see `simAssets/LICENSE_aws-robomaker-small-house-world.txt`).
- The bookshelf in the simulator uses the book-cover images that ship with the Computer Vision Toolbox. They load at run time and are not in this repository.
