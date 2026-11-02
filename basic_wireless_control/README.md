# Basic wireless control

Drive the ELEGOO Smart Robot Car V4 over Wi-Fi from MATLAB and stream its camera at the same time.

| File | What it does |
|---|---|
| `ElegooRobotCar.m` | Client class for the car. It sends JSON commands over TCP port 100 to the course firmware, and reads the ESP32 camera over HTTP (`/capture`, `/stream`). |
| `elegooCarDashboard.m` | Dashboard UI: teleoperation, timed (v, ω, T) drives, live camera, sonar, IR and IMU, pan scan, data logging and snapshots. |
| `driveUntilObstacle.m` | Drive forward until the ultrasonic sensor sees an object. |
| `driveCircle.m` | Drive in a circle and log the IMU. |
| `edgeHuggingMapperUI.m` | UI that crawls along a table edge with the IR line sensors. |
| `tableMapperVideoUI.m` | UI that maps a table edge (IR + sonar) and records the camera video. |
| `make_gif.m`, `make_video.m` | Save camera frames as a GIF or an MP4. |

## Quick start

1. Join the car's Wi-Fi network (`ELEGOO-...`). The car is at `192.168.4.1`.
2. In MATLAB, open this folder and run:

```matlab
robot = ElegooRobotCar("192.168.4.1");
robot.setWatchdog(1000);   % the motors stop if MATLAB stops talking for 1 s
robot.drive(100, 100); pause(0.3); robot.stop();
imshow(robot.capture());
robot.readDistance()
```

Or run `elegooCarDashboard` for the full UI.

`ElegooRobotCar` needs the course's custom car firmware (commands `N=7` drive, `N=10` distance, `N=12` IMU, `N=30` pan, `N=102` watchdog). The stock ELEGOO firmware does not answer these commands.
