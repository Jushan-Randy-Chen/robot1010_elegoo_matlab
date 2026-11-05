# Basic wireless control

Drive the ELEGOO Smart Robot Car V4 over Wi-Fi from MATLAB and stream its camera at the same time.

| File | What it does |
|---|---|
| `ElegooRobotCar.m` | Client class for the car. It sends JSON commands over TCP port 100 to the course firmware, and reads the ESP32 camera over HTTP (`/capture`, `/stream`). The four constants at the top set the wheel sides and the photo size and direction. |
| `checkCar.m` | Five-minute hardware check (sensors, timing, photo direction, wheel sides, watchdog). It prints the constants for `ElegooRobotCar.m`. Run it once per car type. |
| `elegooCarDashboard.m` | Dashboard UI: teleoperation, timed (v, ω, T) drives, live camera, sonar, IR and IMU, pan scan, data logging and snapshots. |
| `driveUntilObstacle.m` | Drive forward until the ultrasonic sensor sees an object. |
| `driveCircle.m` | Drive in a circle and log the IMU. |

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

Or run `elegooCarDashboard` for the full UI. Run `checkCar` first on a new car.

Notes: the sonar sends `distance: 0` when no echo comes back (nothing within about 5 m). Only drive commands reset the 1 s watchdog. The ESP32 accepts one TCP connection at a time.

The other folders find `ElegooRobotCar.m` here: the table-edge mapper UIs are in [`../edge_mapping`](../edge_mapping/), the GIF and MP4 recorders in [`../camera_recording`](../camera_recording/), and the IR sensor test in [`../sensor_tests/ir_line_sensor`](../sensor_tests/ir_line_sensor/).

`ElegooRobotCar` needs the course's custom car firmware (commands `N=7` drive, `N=10` distance, `N=12` IMU, `N=30` pan, `N=102` watchdog). The stock ELEGOO firmware does not answer these commands.
