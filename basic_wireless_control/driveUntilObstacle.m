function result = driveUntilObstacle(robot, stopDistanceCm, speed, maxDurationSec)
% Drive forward at low speed until the ultrasonic sensor sees a close object.
% Example: result = driveUntilObstacle(robot, 25, 100, 20)

if nargin < 2 || isempty(stopDistanceCm)
    stopDistanceCm = 25;
end
if nargin < 3 || isempty(speed)
    speed = 100;
end
if nargin < 4 || isempty(maxDurationSec)
    maxDurationSec = 20;
end

validateattributes(stopDistanceCm, {'numeric'}, {'scalar', 'positive', 'finite'});
validateattributes(speed, {'numeric'}, {'scalar', 'integer', '>', 0, '<=', 255});
validateattributes(maxDurationSec, {'numeric'}, {'scalar', 'positive', 'finite'});

% Ensure an old command cannot leave the car moving, then enable a fail-safe.
robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); %#ok<NASGU>

timer = tic;
result = struct("reason", "timeout", "distanceCm", NaN, "elapsedSec", 0);

while toc(timer) < maxDurationSec
    reading = robot.readDistance();
    if ~isstruct(reading) || ~isfield(reading, "distance")
        error("driveUntilObstacle:BadSensorData", ...
            "Ultrasonic response did not contain a distance field.");
    end

    distanceCm = double(reading.distance);
    result.distanceCm = distanceCm;
    result.elapsedSec = toc(timer);
    fprintf("Distance: %.0f cm\n", distanceCm);

    % A zero/invalid echo is treated as a sensor fault and stops the car.
    if ~isfinite(distanceCm) || distanceCm <= 0
        error("driveUntilObstacle:InvalidDistance", ...
            "Ultrasonic reading was invalid (%g cm); stopping for safety.", distanceCm);
    end

    if distanceCm <= stopDistanceCm
        result.reason = "obstacle";
        robot.stop();
        fprintf("Stopping at %.0f cm (threshold %.0f cm).\n", ...
            distanceCm, stopDistanceCm);
        return
    end

    robot.drive(speed, speed);
    pause(0.1);
end

result.elapsedSec = toc(timer);
robot.stop();
fprintf("Stopped after reaching the %.1f-second time limit.\n", maxDurationSec);
end

function stopRobotSafely(robot)
try
    robot.stop();
catch
    % Best effort if Wi-Fi has already disconnected.
end
end
