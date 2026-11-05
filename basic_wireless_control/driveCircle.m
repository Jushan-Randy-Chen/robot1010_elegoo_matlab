function log = driveCircle(robot, v, w, T)
% DRIVECIRCLE  Drive on a circle for T seconds and log the gyro.
%   log = driveCircle(robot)             % v = 100, w = 30, T = 5
%   log = driveCircle(robot, v, w, T)
% Differential drive in PWM units: left = v - w, right = v + w.
% w > 0 turns left. Clear 1.5 m of floor around the car first.
arguments
    robot
    v (1,1) double = 100
    w (1,1) double = 30
    T (1,1) double {mustBePositive} = 5
end

robot.setWatchdog(1000);
stopAtEnd = onCleanup(@() robot.stop());
left = round(min(max(v - w, -255), 255));
right = round(min(max(v + w, -255), 255));
t = [];
gyroZ = [];
timer = tic;
while toc(timer) < T
    robot.drive(left, right);            % send again each loop: the watchdog stays fed
    imu = robot.readIMU();
    t(end+1) = toc(timer); %#ok<AGROW>
    gyroZ(end+1) = imu.gyro(3); %#ok<AGROW>
end
robot.stop();

rawPerDegS = 131;                        % MPU-6050: 131 raw = 1 deg/s
if isa(robot, "ElegooSimCar")
    rawPerDegS = 1;                      % the simulator gyro gives deg/s
end
rateDeg = gyroZ / rawPerDegS;
log = struct("t", t, "gyroZ", gyroZ, "turnDeg", trapz(t, rateDeg));
figure("Name", "driveCircle", "NumberTitle", "off");
plot(t, rateDeg, ".-");
grid on
xlabel("time (s)");
ylabel("turn rate (deg/s)");
title(sprintf("left %d, right %d PWM: turned %.0f deg in %.1f s", left, right, log.turnDeg, T));
end
