# Camera recording

Scripts that save frames from the car's ESP32 camera. Both use a `robot` variable in the workspace:

```matlab
addpath ../basic_wireless_control
robot = ElegooRobotCar("192.168.4.1");
make_gif      % 30 frames -> test.gif
make_video    % frames -> test.mp4 (MPEG-4)
```
