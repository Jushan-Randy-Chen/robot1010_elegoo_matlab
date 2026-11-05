# Edge mapping

UIs that use the IR line sensors and the ultrasonic sensor to find and follow the edge of a table. Both UIs connect to the car at `192.168.4.1` and find `ElegooRobotCar.m` in `../basic_wireless_control`.

| File | What it does |
|---|---|
| `edgeHuggingMapperUI.m` | Crawls along a table edge: drive to the edge, stop, back up, turn a little, repeat. |
| `tableMapperVideoUI.m` | Maps a table edge (IR + sonar) and records the camera video. |

A table edge reads as `onLine = 1`, because the IR sensors get no reflection past the edge. Before you use these UIs on a table, run the lifted-car step of `sensor_tests/ir_line_sensor/` to make sure that this is true for your car. Keep a hand near the car.
