function result = driveUntilObstacle(robot, stopDistanceCm, speed, maxDurationSec)
% DRIVEUNTILOBSTACLE  Drive forward until the sonar sees a near object.
%   result = driveUntilObstacle(robot)                 % 25 cm, speed 100, 20 s
%   result = driveUntilObstacle(robot, 25, 100, 20)

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

nearLimitCm = 2 * stopDistanceCm;   % a last good reading below this is "near"
lastGoodCm = NaN;                   % NaN: no good reading yet
timer = tic;
result = struct("reason", "timeout", "distanceCm", NaN, ...
    "lastGoodCm", NaN, "elapsedSec", 0);

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

    if distanceCm > 0
        % Good reading: compare it with the stop distance.
        lastGoodCm = distanceCm;
        result.lastGoodCm = lastGoodCm;
        if distanceCm <= stopDistanceCm
            result.reason = "obstacle";
            robot.stop();
            fprintf("Stopping at %.0f cm (threshold %.0f cm).\n", ...
                distanceCm, stopDistanceCm);
            return
        end
    elseif ~isnan(lastGoodCm) && lastGoodCm <= nearLimitCm
        % No echo, and the last good reading was near: the object can be
        % too near for the sonar.
        result.reason = "noEchoNear";
        robot.stop();
        fprintf("Stopping: no echo, and the last good reading was %.0f cm.\n", ...
            lastGoodCm);
        return
    end
    % Else: no echo and the last good reading was far (or none yet): continue.

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
