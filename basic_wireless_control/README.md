# Basic wireless control

Use MATLAB to drive the ELEGOO Smart Robot Car V4 over Wi-Fi. You can also stream the car camera at the same time.

This README uses ASD-STE100 Simplified Technical English.

| File | What it does |
|---|---|
| `ElegooRobotCar.m` | Client class for the car. It sends JSON commands over TCP port 100 to the course firmware. It reads the ESP32 camera over HTTP (`/capture`, `/stream`). The four constants at the top set the wheel sides and the photo size and direction. |
| `checkCar.m` | Five-minute hardware check (sensors, timing, photo direction, wheel sides, watchdog). It prints the constants for `ElegooRobotCar.m`. Run it one time for each car type. |
| `elegooCarDashboard.m` | Dashboard UI: teleoperation, timed (v, turn, T) drives, live camera, sonar, IR and IMU, pan scan, data logging and snapshots. |
| `driveUntilObstacle.m` | `driveUntilObstacle(robot, stopCm, speed, T)`: drive forward. Stop when the sonar sees an object at `stopCm` or nearer. |
| `driveCircle.m` | `driveCircle(robot, v, turn, T)`: drive on a circle. Plot the gyro turn rate and measure the deg/s for 1 PWM of turn. |

## Safety

> **WARNING:** Keep the car away from stairs and table edges. The car can fall and cause injury.

> **CAUTION:** Lift the car or set the motor switch to OFF before you connect MATLAB. An old command can make the wheels turn.

- All the scripts in this folder set a 1 s motor watchdog. If MATLAB stops and sends no drive command for 1 s, the motors stop.
- To stop the car, push Ctrl+C in MATLAB. The motors stop after 1 s.
- If the motors do not stop, lift the car. Then set the power switch to OFF.

## Words and units

| Word | Meaning |
|---|---|
| PWM | The motor command. The range is -255 (full speed backward) to 255 (full speed forward). |
| `v` | The forward speed command, in PWM. |
| `turn` | The PWM difference between the two sides. `turn` > 0 turns the car to the left. `turn` is **not** an angular velocity in rad/s. |
| turn rate | The angular velocity of the car, in deg/s. The gyro measures it. |

The wheel commands are:

```
left wheels  = v - turn
right wheels = v + turn
```

The same `turn` value gives different turn rates on different cars, floors and battery levels. Tutorial 2 shows how to measure the turn rate for your car.

## Procedure 1: Connect to the car

1. Set the car power switch to ON. Keep the motor switch at OFF.
2. Connect the computer to the car Wi-Fi network (`ELEGOO-...`). The car address is `192.168.4.1`.
3. In MATLAB, open this folder.
4. Type these commands:

   ```matlab
   robot = ElegooRobotCar("192.168.4.1");
   robot.setWatchdog(1000);   % the motors stop if MATLAB stops for 1 s
   imshow(robot.capture());
   robot.readDistance()
   ```

5. Make sure that a photo shows and that MATLAB shows a distance.
6. For a new car, run `checkCar` before you continue.

## Procedure 2: Drive with the dashboard

1. Run `elegooCarDashboard`.
2. Set the motor switch to ON.
3. Push one arrow button. The car moves for one pulse (0.2 s), then it stops.
4. Set `v`, `turn` and `T`. Then push **Run Timed Drive**.
5. Close the dashboard when you complete the work. The car accepts only one connection at a time.

## Tutorials

Do each tutorial in the simulator first. Then do it on the real car. To use the simulator, type these commands:

```matlab
addpath ../vslam_examples
robot = ElegooSimCar();
```

To use the real car, type this command:

```matlab
robot = ElegooRobotCar("192.168.4.1");
```

### Tutorial 1: Stop at an obstacle (on-off control)

`driveUntilObstacle` uses on-off control. The car drives at one speed. When the sonar reading is equal to or less than the stop distance, the car stops. There is no speed between "drive" and "stop".

**The sonar reading 0.** The sonar sends 0 when it receives no echo. Two different conditions give 0:

- Nothing is within approximately 5 m. The path is clear.
- An object is nearer than approximately 3 cm, or its surface is soft or at an angle. The path is **not** clear.

Thus, the code uses 0 as "unknown". It uses the last good reading (more than 0) to make a decision:

| Last good reading | Decision | `result.reason` |
|---|---|---|
| Equal to or less than 2 x stop distance | Stop. This is the safe decision. | `"noEchoNear"` |
| More than 2 x stop distance | Continue to drive. | (no stop) |
| No good reading yet | Continue to drive. | (no stop) |

The simulator sends 150 cm, not 0, when there is no echo. Thus, you can only see the 0 condition on the real car.

**Procedure:**

1. Put the car 1 m in front of a wall. Make sure that the path is clear.
2. Type this command:

   ```matlab
   result = driveUntilObstacle(robot, 25, 60, 10)
   ```

3. Measure the distance from the sonar to the wall with a ruler.
4. Write the result in the table. Do the test again with speed 100 and speed 150.

| Speed (PWM) | `result.distanceCm` | Ruler distance (cm) | Difference (cm) |
|---|---|---|---|
| 60 | | | |
| 100 | | | |
| 150 | | | |

**Questions:**

1. The stop distance is 25 cm. Why does the car stop nearer than 25 cm?
   Hint: each loop sends two Wi-Fi commands and waits 0.1 s. The car moves during this time.
2. Is the difference larger at a higher speed? Calculate the distance that the car moves in one loop.
3. Put a soft object (a pillow or a sweater) in front of the car. Does the sonar see it? Which line of `driveUntilObstacle.m` stops the car?

### Tutorial 2: Drive on a circle and measure the turn rate

`driveCircle` sends the same `left` and `right` commands for `T` seconds. This is open-loop control: the code does not use the gyro to change the commands. The gyro only measures the result.

The function gives these values:

| Field | Meaning |
|---|---|
| `log.turnDeg` | The total angle that the car turned (the integral of the turn rate). |
| `log.meanRateDeg` | The mean turn rate in the second half of the run. In the first half, the speed of the car increases from 0. |
| `log.degPerSecPerPWM` | The turn rate for 1 PWM of `turn`. Use this value to convert `turn` to deg/s. |

**Procedure:**

1. Make sure that there is 1.5 m of clear floor around the car.
2. Type this command:

   ```matlab
   log = driveCircle(robot, 100, 30, 5)
   ```

3. Write `log.meanRateDeg` and `log.degPerSecPerPWM` in the table.
4. Do the test again with `turn` = 15 and `turn` = 60.

| `turn` (PWM) | `log.meanRateDeg` (deg/s) | `log.degPerSecPerPWM` |
|---|---|---|
| 15 | | |
| 30 | | |
| 60 | | |

**Questions:**

1. Is `log.degPerSecPerPWM` the same for the three tests? If it is not the same, give a reason.
2. You want the car to turn at 45 deg/s. Use your table to calculate `turn`. Do the test and compare.
3. Make a plot of `log.meanRateDeg` against `turn`. Is the line straight? Does the line touch the point (0, 0)?

This procedure is a system identification. You measure the relation between the command and the result of the car. A controller needs this relation.

### Tutorial 3: Remove the gyro bias

When the car does not move, the gyro does not read exactly 0. This value is the bias. `driveCircle` adds the bias to each sample. Thus, `log.turnDeg` has a small error that increases with time.

**Procedure:**

1. Put the car on the floor. Do not move the car.
2. Type these commands:

   ```matlab
   for k = 1:50
       imu = robot.readIMU();
       g(k) = imu.gyro(3);
   end
   bias = mean(g)
   ```

3. Open `driveCircle.m`. Find this line:

   ```matlab
   rateDeg = gyroZ / rawPerDegS;
   ```

4. Change it so that it subtracts `bias` from `gyroZ`. Add `bias` as an input of the function.
5. Do Tutorial 2 again. Compare `log.turnDeg` before and after the change.

**Question:** The bias is 50 raw (0.38 deg/s). Calculate the angle error after 60 s.

### Tutorial 4: Saturation

The motor driver accepts commands from -255 to 255 only. `driveCircle` limits `left` and `right` to this range. When `v + turn` is more than 255, the right wheels cannot go faster. Thus, the car turns less than the command. This is actuator saturation.

**Procedure:**

1. Type this command:

   ```matlab
   log = driveCircle(robot, 240, 60, 4)
   ```

2. Read the warning in the Command Window.
3. Compare `log.meanRateDeg` with the result for `v` = 100 and `turn` = 60 from Tutorial 2.

**Question:** The commands are `v` = 240 and `turn` = 60. Calculate the `left` and `right` values after the limit. What is the real `turn` value?

## Notes

- Only drive commands reset the 1 s watchdog. Sensor commands do not reset it.
- The ESP32 accepts one TCP connection at a time.
- `ElegooRobotCar` needs the course firmware (commands `N=7` drive, `N=10` distance, `N=12` IMU, `N=30` pan, `N=102` watchdog). The stock ELEGOO firmware does not answer these commands.
- The other folders use `ElegooRobotCar.m` from this folder:
  - the table-edge mapper UIs in [`../edge_mapping`](../edge_mapping/)
  - the GIF and MP4 recorders in [`../camera_recording`](../camera_recording/)
  - the IR sensor test in [`../sensor_tests/ir_line_sensor`](../sensor_tests/ir_line_sensor/).
