function log = driveCircle(robot, v, turn, T)
% DRIVECIRCLE  Drive on a circle for T seconds and log the gyro.
%   log = driveCircle(robot)                % v = 100, turn = 30, T = 5
%   log = driveCircle(robot, v, turn, T)
% Differential drive in PWM units (-255 to 255):
%   left wheels  = v - turn
%   right wheels = v + turn
% turn > 0 turns left. turn is a PWM difference, NOT an angular velocity
% in rad/s. This function measures the angular velocity with the gyro, and
% gives log.degPerSecPerPWM: the turn rate that 1 PWM unit of turn gives.
% Make sure that there is 1.5 m of clear floor around the car.
arguments
    robot
    v (1,1) double = 100
    turn (1,1) double = 30
    T (1,1) double {mustBePositive} = 5
end

robot.setWatchdog(1000);
stopAtEnd = onCleanup(@() robot.stop());
% Saturation: the motor driver accepts -255 to 255 only.
left = round(min(max(v - turn, -255), 255));
right = round(min(max(v + turn, -255), 255));
if abs(v - turn) > 255 || abs(v + turn) > 255
    warning("driveCircle:Saturated", ...
        "v +/- turn is out of -255..255. The car turns less than you asked.");
end
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

% Steady-state turn rate: use the second half of the run only. In the
% first half, the speed of the car increases from 0.
steady = t >= T / 2;
meanRateDeg = mean(rateDeg(steady));
actualTurn = (right - left) / 2;         % the turn value after saturation
degPerSecPerPWM = meanRateDeg / actualTurn;   % Inf or NaN when turn = 0

log = struct("t", t, "gyroZ", gyroZ, "rateDeg", rateDeg, ...
    "turnDeg", trapz(t, rateDeg), "meanRateDeg", meanRateDeg, ...
    "degPerSecPerPWM", degPerSecPerPWM);
figure("Name", "driveCircle", "NumberTitle", "off");
plot(t, rateDeg, ".-");
hold on
yline(meanRateDeg, "--", sprintf("steady %.1f deg/s", meanRateDeg));
hold off
grid on
xlabel("time (s)");
ylabel("turn rate (deg/s)");
title(sprintf("left %d, right %d PWM: turned %.0f deg in %.1f s", left, right, log.turnDeg, T));
fprintf("Steady turn rate %.1f deg/s. 1 PWM of turn = %.2f deg/s.\n", ...
    meanRateDeg, degPerSecPerPWM);
end
