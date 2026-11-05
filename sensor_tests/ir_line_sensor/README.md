# IR line sensor test

Test the three IR line-tracking sensors of the ELEGOO Smart Robot Car V4 before you use them for line tracking or table-edge mapping. **The motors stay off for the whole test.**

You need a light, flat floor and a strip of matte black electrical tape.

| File | What it does |
|---|---|
| `runIRSensorTest.m` | Start here. Set `USE_SIMULATOR`, then run. It connects and runs the full test. |
| `irSensorTest.m` | The guided test (9 steps, below). Returns a report with one PASS/FAIL/SKIP row per step. |
| `irLiveMonitor.m` | Live view: raw readings, the firmware's `onLine` flags and the line-tracker decision. |
| `lineTrackerDecision.m` | The steering rule: `[L M R]` → forward / left / right / stop. |
| `ElegooSimIRCar.m` | Simulator of the IR sensors with the same IR commands as `ElegooRobotCar`. No car needed. |
| `testIRSensorTest.m` | Unit tests on the simulator, healthy and faulty sensors: `runtests("testIRSensorTest")`. |

## Quick start

```matlab
cd sensor_tests/ir_line_sensor
runIRSensorTest                        % USE_SIMULATOR = true: no car needed
```

On the real car, join the car's Wi-Fi network (`ELEGOO-...`), set `USE_SIMULATOR = false` and run again. Do each step that the Command Window tells you to do, then push Enter. You can also call the functions yourself:

```matlab
addpath ../../basic_wireless_control
robot  = ElegooRobotCar("192.168.4.1");
report = irSensorTest(robot, LiveSeconds=20, SaveFile="irTest.mat");
irLiveMonitor(robot, 30);              % slide tape under the sensors and watch
```

## How the sensors work

The car has three ITR20001 reflective IR sensors at the front: left on `A2`, middle on `A1`, right on `A0`. The Arduino reads each one as a 10-bit ADC count (0 to 1023). `readIR` (firmware command `N=11`) returns:

```json
{"ir":[L,M,R], "onLine":[l,m,r]}
```

The firmware sets `onLine = ir > irThreshold`. The default threshold is 600, and `setIRThreshold` (`N=16`) changes it until the car is switched off. A light floor reflects IR and reads low. Black tape reads higher. A lifted car, or a car at a table edge, gets almost no reflection and reads near the top. This is why the edge mappers in `edge_mapping/` see a table edge as `onLine = 1`.

## Test steps

| Step | Pass when |
|---|---|
| 1 Response format | `readIR` gives 3 ADC counts (0 to 1023) and 3 `onLine` flags (0 or 1) |
| 2 Read rate | at least `MinRateHz` readings per second (default 5) over Wi-Fi |
| 3 onLine rule | the firmware's `onLine` agrees with `ir > irThreshold` on every reading |
| 4 Light floor | the noise (std) of each sensor is at most `MaxNoise` (default 25 counts), and no sensor is stuck at 0 or 1023 |
| 5 Black line | the same checks as step 4, with tape under all three sensors |
| 6 Contrast | black reads at least `MinContrast` (default 150 counts) higher than light on each sensor |
| 7 Threshold | one threshold separates light and black on all three sensors, with 4 std of noise clear on each side |
| 8 Lifted car | each sensor reads above the threshold when the car is lifted about 5 cm |
| 9 Firmware update | `setIRThreshold` stores the recommended threshold, and `onLine` follows it |

After the test, `report.recommendedThreshold` holds the threshold for this floor and `report.perSensorThreshold` holds the midpoint for each sensor. The test also plots the mean reading of each sensor on each surface, with both thresholds drawn as lines.

## If a step fails

- **Contrast or threshold**: use matte (not shiny) black tape on a light floor. Clean the sensors. A sensor that gives the same value on every surface has a loose cable. If one threshold cannot serve all three sensors, apply `report.perSensorThreshold` in MATLAB to the raw `ir` values.
- **Noise**: hold the car still and keep it out of direct sunlight, because sunlight contains IR.
- **Lifted car**: the line tracker cannot tell "lifted" from "on the line", so do not run it near a table edge.
- **Response format**: the car must have the course firmware. The stock ELEGOO firmware does not answer `N=11` in this format.

The live check in `irLiveMonitor` uses the same rule as the line tracker (`111`/`010` forward, `100`/`110` left, `001`/`011` right, `000`/`101` stop). When the tape is between two sensors, you can see a short `000` (stop). The line tracker must handle this.
