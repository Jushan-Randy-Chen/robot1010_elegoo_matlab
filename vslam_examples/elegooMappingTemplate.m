%% ELEGOO SMART ROBOT CAR V4: MAP AND POSITION TEMPLATE
% ROBT 1010 Intro to Robotics Engineering.
% The comments in this file use ASD-STE100 Simplified Technical English.
%
% =========================================================================
% SUMMARY
% =========================================================================
% PURPOSE
%   This file shows the two basic functions of a mobile robot:
%     - Localization: the robot calculates its position and its heading.
%     - Mapping: the robot makes a map of the objects near it.
%   The car does these functions with its sensors. It does not use a GPS.
%   Task 1 to task 5 start with one sensor and end with visual SLAM
%   (Simultaneous Localization And Mapping). Visual SLAM makes a map and
%   finds the car in the map at the same time.
%
% SENSORS
%   Sensor              Part on the car       Measures                     Tasks
%   ------------------  --------------------  ---------------------------  ---------
%   Ultrasonic sensor   HC-SR04 module, on    Distance to the nearest      1 2 3 4 5
%   (sonar)             the pan servo head    object, 2 cm to 150 cm
%   Camera              ESP32 camera module,  Photos (400 x 296 pixels).   1 2 4 5
%                       on the pan servo head The computer finds corners
%                                             in the photos.
%   Gyro                MPU-6050 IMU          Turn rate of the car around  1
%                                             the vertical axis
%
% PARTS THAT MOVE THE CAR (ACTUATORS)
%   Motors (4)          DC gear motors. The PWM value sets the motor power.
%   Pan servo           Turns the sensor head (sonar and camera) to the
%                       left and to the right.
%
% WHAT EACH TASK DOES
%   1  Read the sensors. Find the range and the noise of each sensor.
%   2  Calibrate the motion. Measure one forward step (sonar) and one turn
%      step (camera).
%   3  Make a sonar map from one position (occupancy grid).
%   4  Move the car in a square. Calculate the position from the steps and
%      the camera (dead reckoning). Make a sonar map.
%   5  Make a camera map with visual SLAM. The sonar measures the scale.
%   Do the tasks in sequence. Task 4 uses the results of task 2.
%
% =========================================================================
% HOW TO USE THIS FILE
% =========================================================================
%   1. Set the MATLAB current folder to the folder of this file.
%      If MATLAB shows a message with "Change Folder" and "Add to Path"
%      when you click Run, click "Change Folder".
%   2. Set TASK (below) to the number of the task.
%   3. Do the task in the simulator first: set USE_SIMULATOR = true.
%      The simulator is a living room in MATLAB. The car is not necessary.
%   4. Click Run. You can also push F5 (some laptops and Macs: Fn+F5).
%   5. Read the results. Answer the questions in the task section.
%   6. Then do the same task on the real car:
%      a. Connect the computer to the Wi-Fi network of YOUR car. Each car
%         has a different network name ("ELEGOO-..."). Ask your TA.
%      b. Set USE_SIMULATOR = false.
%      c. Click Run.
%   7. Compare the results of the simulator and the real car.
%
% NOTE: README.md shows example figures (for example "Task 3, Figure 3.1").
% NOTE: The simulator car and the real car are not the same. Do task 2 on
%       the real car before task 4 on the real car. Write the results of
%       the real car in SETTINGS.
%
% CAUTION: KEEP THE CAR AWAY FROM STAIRS AND TABLE EDGES.
%          THE CAR CAN FALL AND BREAK.
% CAUTION: OPERATE THE CAR ON THE FLOOR ONLY. DO NOT OPERATE IT ON A TABLE.
% CAUTION: KEEP A CLEAR FLOOR AREA OF 1 m x 1 m (2.5 m x 2.5 m FOR TASK 5).
%          IF THE FLOOR IS NOT CLEAR, THE CAR CAN HIT AN OBJECT.
% CAUTION: THE CAR MOVES BACK WITHOUT A SENSOR. KEEP THE FLOOR BEHIND THE
%          CAR CLEAR.
% CAUTION: TO STOP THE CAR, PUSH CTRL+C IN THE COMMAND WINDOW. IF THE CAR
%          DOES NOT STOP, LIFT THE CAR. SET THE POWER SWITCH TO OFF.
%
% NOTE: The car moves in short steps: motor pulse, stop, measure.
%       The car has no wheel encoders. Photos are not clear when the car moves.
% NOTE: Keep these files in the same folder as this file:
%         ElegooRobotCar.m         Commands for the real car (Wi-Fi)
%         ElegooSimCar.m           The simulator car and room
%         simAssets folder         Photos of the surfaces in the simulated room
%         calibrateElegooCamera.m  Camera calibration for task 5 (real car)
% NOTE: This file uses the Computer Vision Toolbox and the Image
%       Processing Toolbox.

TASK = 1;                  % Task number: 1, 2, 3, 4 or 5
USE_SIMULATOR = true;      % true = simulated room. false = real car.

%% SETTINGS
% Change these values for your car.
% Task 2 measures S.cmPerStep and S.degPerTurn. Write the results here.

S.host = "192.168.4.1";         % IP address of the car

% One step is [left motor PWM, right motor PWM, time in seconds].
% PWM is the motor power: -255 (full reverse) to 255 (full forward).
S.forward  = [100 100 0.30];    % One step forward (approximately 5 cm to 8 cm)
S.backward = [-100 -100 0.30];  % One step back
S.turnLeft = [-130 130 0.10];   % One turn step to the left, in place (approximately 10 to 20 degrees)
S.circle   = [135 160 0.15];    % One step on a large circle to the left (task 5)
S.settleSec = 0.2;              % Time to wait after a stop. The car must not move when it takes a photo.
% NOTE: The Wi-Fi delay adds time to each motor pulse. Thus the size of a
%       step can change a little from step to step.

% Results of task 2
S.cmPerStep  = 5;               % Distance of one forward step (cm)
S.degPerTurn = 15;              % Angle of one turn step (degrees)

% Sensors
S.panLeftIsBigger = true;       % true if pan(150) turns the sensor head to the LEFT (task 3)
S.sonarMaxCm = 150;             % No object in range (the real sonar sends 0): the car uses this value
S.sensorAheadCm = 9;            % Distance from the wheel axle to the sensor head (cm)
S.cameraFovDeg = 60;            % Camera field of view, left edge to right edge (degrees)
S.cameraHeightCm = 12;          % Height of the camera above the floor (cm)
% NOTE: The photo direction (upside down, mirror) is set in ElegooRobotCar.m.

%% CONNECT TO THE CAR AND DO THE TASK
% The results go into the variable "result" in the Workspace.
clear car                       % Close the old connection
TASK_NAMES = ["Read the sensors", "Calibrate the motion", "Make a sonar map from one position", ...
    "Move in a square, calculate the position and make a map", "Make a camera map with visual SLAM"];
assert(isnumeric(TASK) && isscalar(TASK) && any(TASK == 1:5), "Set TASK to 1, 2, 3, 4 or 5 (a number).");
fprintf("\n========== TASK %d: %s ==========\n", TASK, upper(TASK_NAMES(TASK)));
[car, S] = connectCar(USE_SIMULATOR, S, TASK);
result = runTask(car, S, TASK);  % The motors stop at the end, after an error, and after Ctrl+C
fprintf("Task %d is finished. Look at the figures (Task %d, Figure %d.1 ...).\n", TASK, TASK, TASK);
fprintf("The results are in the variable ""result"" in the Workspace.\n");


%% ========================================================================
%  TASK 1: READ THE SENSORS
%  ========================================================================
function result = runTask(car, S, task)
% Do one task. onCleanup stops the motors when the task ends, when an
% error occurs, and when you push Ctrl+C.
stopAtEnd = onCleanup(@() safeStop(car));
switch task
    case 1, result = task1_readSensors(car, S);
    case 2, result = task2_calibrate(car, S);
    case 3, result = task3_sonarMap(car, S);
    case 4, result = task4_squareMap(car, S);
    case 5, result = task5_visualSlam(car, S);
end
end


function result = task1_readSensors(car, S)
% PURPOSE
%   This task shows what each sensor measures. It also shows the accuracy
%   of each sensor.
%
% PROCEDURE
%   1. Run the task.
%   2. During the first 10 seconds, move your hand in front of the sonar.
%      Move your hand near the sensor and then away from the sensor.
%   3. The car takes a photo. Then it turns left for 0.8 seconds.
%   4. Look at the four plots.
%
% QUESTIONS
%   a. Put a book 30 cm in front of the sonar. Use a ruler.
%      What value does the sonar show?
%   b. What value does the sonar show when no object is in front of it?
%   c. Turn the car until the sonar is at an angle of more than 45 degrees
%      to a wall. What occurs? Why? (Help: where does the sound go?)
%   d. Turn the camera to a blank wall. How many corners does the computer
%      find? Then turn the camera to a bookshelf. Visual SLAM must have
%      corners.
%   e. Is the gyro value positive or negative when the car turns left?
%   f. The car turned for 0.8 seconds. Measure the angle that the car turned.
%      Compare it with the gyro angle. What are the units of the gyro?

% FIGURE 1.1: four panels, (a) to (d)
labeledFigure(car, 1, 1, "Sensor readings", [2 2], [80 80 1100 650]);

% 1. Sonar for 10 seconds
ax = nexttile;
h = plot(ax, NaN, NaN, "b.");
grid(ax, "on");
xlabel(ax, "time (s)");
ylabel(ax, "distance (cm)");
ylim(ax, [0 160]);
title(ax, "(a) Sonar distance. Move your hand in front of the car.");
sonarTime = [];
sonarCm = [];
timer = tic;
while toc(timer) < 10
    sonarTime(end+1) = toc(timer); %#ok<AGROW>
    sonarCm(end+1) = readSonar(car, S, 1); %#ok<AGROW>
    set(h, "XData", sonarTime, "YData", sonarCm);
    drawnow limitrate
    pause(0.05);
end

% 2. One photo and its corners (ORB features)
photo = takePhoto(car);
corners = detectORBFeatures(im2gray(photo));
strongest = corners.selectStrongest(200);
ax = nexttile;
imshow(photo, "Parent", ax);
hold(ax, "on");
plot(ax, strongest.Location(:, 1), strongest.Location(:, 2), "g+");
title(ax, sprintf("(b) Camera photo: %d corners found (green +, 200 strongest)", corners.Count));

% 3. Gyro: turn left for 0.8 seconds and record the turn rate
gyroTime = [];
gyroZ = [];
timer = tic;
car.drive(S.turnLeft(1), S.turnLeft(2));
while toc(timer) < 0.8
    imu = car.readIMU();
    gyroTime(end+1) = toc(timer); %#ok<AGROW>
    gyroZ(end+1) = imu.gyro(3); %#ok<AGROW>   % The z axis points up. It measures the turn.
    pause(0.02);
end
car.stop();
ax = nexttile;
plot(ax, gyroTime, gyroZ, "r.-");
grid(ax, "on");
xlabel(ax, "time (s)");
ylabel(ax, "gyro z (raw units)");
ylim(ax, [min(0, min(gyroZ)), max(0, max(gyroZ))] * 1.2 + [-1 1]);   % Show the zero line
title(ax, "(c) Gyro: turn rate while the car turns left");

% 4. Angle = sum of (turn rate x time)
gyroAngle = [0 cumsum(gyroZ(2:end) .* diff(gyroTime))];
ax = nexttile;
plot(ax, gyroTime, gyroAngle, "m.-");
grid(ax, "on");
xlabel(ax, "time (s)");
ylabel(ax, "angle (raw units x s)");
title(ax, sprintf("(d) Gyro angle = sum of (rate x time) = %.0f", gyroAngle(end)));

fprintf("Sonar: %d readings, %.0f cm to %.0f cm.\n", numel(sonarCm), min(sonarCm), max(sonarCm));
fprintf("Camera: %d corners in the photo.\n", corners.Count);
fprintf("Gyro: angle = %.0f (raw units x s) after the 0.8 s turn.\n", gyroAngle(end));
result = struct("sonarTime", sonarTime, "sonarCm", sonarCm, "photo", photo, ...
    "numCorners", corners.Count, "gyroTime", gyroTime, "gyroZ", gyroZ, "gyroAngle", gyroAngle);
end


%% ========================================================================
%  TASK 2: CALIBRATE THE MOTION
%  ========================================================================
function result = task2_calibrate(car, S)
% PURPOSE
%   The car has no wheel encoders. To calculate its position, the car counts
%   its steps. This task measures the length of one forward step and the
%   angle of one turn step.
%
% PROCEDURE
%   1. Put the car approximately 1 m from a wall or a large box.
%   2. Run the task.
%   3. The car turns until the sonar finds an object 70 cm to 130 cm away.
%   4. The car moves 8 steps forward and 8 steps back. The change of the
%      sonar distance gives the length of one step.
%   5. The car turns in place approximately 400 degrees, one turn step at a
%      time. The camera measures each turn step. The photo that matches the
%      first photo shows the step where the car turned 360 degrees.
%   6. Write the two results in SETTINGS: S.cmPerStep and S.degPerTurn.
%
% QUESTIONS
%   a. Do the task three times. How much do the results change? Why?
%   b. Is one step back the same length as one step forward?
%   c. Do the turn steps change more than the forward steps? Why?
%      (Help: the four wheels slide to the side when the car turns.)
%   d. Do the results change on a carpet? On a tile floor?

% 1. Find an object 70 cm to 130 cm in front of the car
for i = 1:30
    cm = readSonar(car, S, 3);
    if cm > 70 && cm < 130, break; end
    stepCar(car, S.turnLeft, S);
end
if ~(cm > 70 && cm < 130)
    error("No object is 70 cm to 130 cm in front of the car. Put the car approximately " + ...
        "1 m from a wall or a large box. Then do the task again.");
end

% 2. Length of one step: the sonar is the ruler.
%    The car stops if it comes nearer than 30 cm to the object.
nSteps = 8;
before = readSonar(car, S, 5);             % Median of 5 values: less noise
nForward = 0;
for i = 1:nSteps
    if ~(readSonar(car, S) >= 30), break; end
    stepCar(car, S.forward, S);
    nForward = nForward + 1;
end
after = readSonar(car, S, 5);
for i = 1:nForward
    stepCar(car, S.backward, S);
end
back = readSonar(car, S, 5);
if nForward < 4
    error("The car moved only %d steps before it came near the object. Put the car " + ...
        "approximately 1 m from the object. Then do the task again.", nForward);
end
cmPerStep = (before - after) / nForward;
cmPerStepBack = (back - after) / nForward;
fprintf("Sonar: %g cm -> %g cm -> %g cm\n", before, after, back);
fprintf("One step forward = %.1f cm. One step back = %.1f cm.\n", cmPerStep, cmPerStepBack);

% 3. Angle of one turn step. The car turns in place, one step at a time.
%    a. The camera measures the angle of each step (as in task 4).
%       The car stops when the sum of the angles is approximately 400 degrees.
%    b. Near 360 degrees, find the photo that matches the first photo best.
%       At this photo, the car is at the first view again.
%    The sum in step a can have a small error. Step b does not have this error.
first = takePhoto(car);
previous = first;
photos = {};
sumDeg = [];
matches = [];
total = 0;
k = 0;
nNoMatch = 0;                           % Turn steps with too few corners
while total < 400 && k < 80
    k = k + 1;
    stepCar(car, S.turnLeft, S);
    photos{k} = takePhoto(car); %#ok<AGROW>
    turn = turnFromPhotos(previous, photos{k}, S);
    if isnan(turn)
        turn = S.degPerTurn;            % Not enough corners: use the value in SETTINGS
        nNoMatch = nNoMatch + 1;
    end
    total = total + turn;
    sumDeg(k) = total; %#ok<AGROW>
    if k == 3 && total < -10
        error("The camera shows a turn to the RIGHT, but the car must turn LEFT. " + ...
            "Ask your TA to run checkCar.m (wheel sides and photo mirror).");
    elseif k == 3 && abs(total) < 0.3 * 3 * S.degPerTurn
        error("The car does not turn. Increase the PWM or the time of S.turnLeft in SETTINGS.");
    end
    [~, matches(k)] = turnFromPhotos(first, photos{k}, S); %#ok<AGROW>
    previous = photos{k};
    fprintf("Turn step %2d: sum = %5.1f deg, %4d corners match the first photo\n", k, total, matches(k));
end
if nNoMatch > 0.2 * k
    error("The camera found too few corners in %d of %d turn steps. Put the car in a " + ...
        "place with more objects (furniture, posters, boxes). Then do the task again.", nNoMatch, k);
end
nearFull = find(sumDeg > 300 & sumDeg < 420);   % Steps near 360 degrees
if isempty(nearFull)
    error("The car did not turn approximately 360 degrees. Do the task again.");
end
[~, i] = max(matches(nearFull));
N = nearFull(i);                                % The step at the first view
extraDeg = turnFromPhotos(first, photos{N}, S); % Angle past the first view
if isnan(extraDeg)
    degPerTurn = total / k;                     % No match: use the sum of the angles
    fprintf("NOTE: The first view was not found. The result uses the sum of the angles.\n");
else
    degPerTurn = (360 + extraDeg) / N;
    fprintf("At turn step %d, the photo matches the first photo. The car turned %.1f degrees in %d steps.\n", ...
        N, 360 + extraDeg, N);
end
fprintf("One turn step = %.1f degrees.\n", degPerTurn);
if ~(cmPerStep > 2 && cmPerStep < 15) || ~(degPerTurn > 3 && degPerTurn < 60)
    warning("The results are not usual (usual: 2 cm to 15 cm, 3 to 60 degrees). Do the task again.");
end

% FIGURE 2.1: two panels, (a) and (b)
tl = labeledFigure(car, 2, 1, "Turn calibration with the camera", [1 2], [80 80 1100 450]);
subtitle(tl, string(tl.Subtitle.String) + sprintf(". Results: one forward step = %.1f cm, one turn step = %.1f deg.", ...
    cmPerStep, degPerTurn));
ax = nexttile;
plot(ax, 1:k, sumDeg, "bo-", "DisplayName", "sum of the camera turn angles");
yline(ax, 360, "k--", "DisplayName", "360 deg (one full turn)");
grid(ax, "on");
xlabel(ax, "turn step");
ylabel(ax, "sum of camera angles (deg)");
legend(ax, "Location", "northwest");
title(ax, "(a) Turn angle from the camera");
ax = nexttile;
plot(ax, 1:k, matches, "bo-", "DisplayName", "corners that match the first photo");
hold(ax, "on");
plot(ax, N, matches(N), "r*", "MarkerSize", 12, "DisplayName", sprintf("first view again: step %d", N));
grid(ax, "on");
xlabel(ax, "turn step");
ylabel(ax, "number of matched corners");
legend(ax, "Location", "northwest");
title(ax, "(b) Matches with the first photo");

fprintf("\nWrite these values in SETTINGS:\n");
fprintf("  S.cmPerStep  = %.1f;\n", cmPerStep);
fprintf("  S.degPerTurn = %.1f;\n", degPerTurn);
result = struct("cmPerStep", cmPerStep, "cmPerStepBack", cmPerStepBack, ...
    "degPerTurn", degPerTurn, "turnSteps", N, "matches", matches);
end


%% ========================================================================
%  TASK 3: MAKE A SONAR MAP FROM ONE POSITION
%  ========================================================================
function result = task3_sonarMap(car, S)
% PURPOSE
%   The sonar is on a servo (the "pan" servo). The servo turns the sensor
%   head from the right to the left. The sonar measures the distance at
%   each 5 degrees. This is a "scan". Each value gives one point:
%     x = distance * cos(angle),   y = distance * sin(angle)
%   An occupancy grid divides the floor into cells of 5 cm x 5 cm.
%   Each cell is free (white), occupied (black) or unknown (gray).
%
% PROCEDURE
%   1. Put the car less than 1 m from walls, boxes or furniture.
%      The sonar range is approximately 150 cm.
%   2. Run the task.
%   3. At the start, the sensor head turns to pan(150) for 1 second.
%      Look at the head. If it turns to the RIGHT, set
%      S.panLeftIsBigger = false in SETTINGS. Then do the task again.
%
% QUESTIONS
%   a. Measure the distance to one wall with a tape measure.
%      Is the wall at the same distance on the map?
%   b. Thin chair legs and walls at a large angle are not on the map. Why?
%   c. The sonar beam is approximately 15 degrees wide. It is not a thin
%      line. How does this change the shape of small objects on the map?
%   d. Change the scan to 2-degree increments. Is the map better?
%      How much more time is necessary for the scan?

% 1. Show the turn direction of the pan servo
car.pan(150);
pause(1);

% 2. Scan from the right (pan 15) to the left (pan 165)
pans = 15:5:165;
ranges = nan(size(pans));
car.pan(pans(1));
pause(0.6);                             % The first move is long: wait for the servo
for i = 1:numel(pans)
    car.pan(pans(i));
    pause(0.15);                        % Wait until the servo is at the angle
    ranges(i) = readSonar(car, S, 3);
end
car.pan(90);
angles = sensorAngle(pans, S);          % 0 = forward, positive = left

% 3. Points and occupancy grid. The origin is the wheel axle of the car.
isEcho = ranges < S.sonarMaxCm - 5;     % 150 cm = no object in range
x = S.sensorAheadCm + ranges .* cosd(angles);
y = ranges .* sind(angles);
map = newMap(400, 5);                   % 4 m x 4 m, cells of 5 cm
for i = 1:numel(angles)
    map = addSonarRay(map, S.sensorAheadCm, 0, angles(i), ranges(i), S);
end

% FIGURE 3.1: two panels, (a) and (b)
labeledFigure(car, 3, 1, "Sonar map from one position", [1 2], [80 80 1250 520]);
ax = nexttile;
h = plot(ax, x(isEcho), y(isEcho), "r.", "MarkerSize", 15, "DisplayName", "sonar echo (object)");
hold(ax, "on");
h(2) = plot(ax, 0, 0, "b^", "MarkerFaceColor", "b", "DisplayName", "car (points to +x)");
h = [h, drawSimRoom(car, truePose(car), ax)];
axis(ax, "equal");
grid(ax, "on");
xlabel(ax, "x = forward (cm)");
ylabel(ax, "y = left (cm)");
legend(ax, h, "Location", "southoutside", "NumColumns", 3);
title(ax, "(a) Scan points");
ax = nexttile;
showMap(map, ax);
hold(ax, "on");
h = gridLegendItems(ax);
h(end+1) = plot(ax, 0, 0, "b^", "MarkerFaceColor", "b", "DisplayName", "car (points to +x)");
h = [h, drawSimRoom(car, truePose(car), ax)];
axis(ax, mapLimits(map));               % Show only the area of the grid
legend(ax, h, "Location", "southoutside", "NumColumns", 3);
title(ax, "(b) Occupancy grid");

result = struct("panDeg", pans, "angleDeg", angles, "rangeCm", ranges, "map", map);
end


%% ========================================================================
%  TASK 4: MOVE IN A SQUARE, CALCULATE THE POSITION AND MAKE A MAP
%  ========================================================================
function result = task4_squareMap(car, S)
% PURPOSE
%   The car moves in a square. After each step, it calculates its pose
%   (position x, y and heading). This is "dead reckoning":
%     forward step:  x = x + cmPerStep * cos(heading)
%                    y = y + cmPerStep * sin(heading)
%     turn step:     heading = heading + (turn angle from the camera)
%   The camera measures each turn: the corners in the photo move to the
%   side when the car turns. This is "visual odometry".
%   The car adds each sonar value to the map at the calculated pose.
%   If the calculated pose is not correct, the map is not correct.
%
% PROCEDURE
%   1. Do task 2 first. Write its results in SETTINGS.
%   2. Put the car less than 1 m from walls or furniture. Make sure that
%      the car has 1 m of clear floor in front of it and on its left side.
%   3. Put a piece of tape on the floor at the start position.
%   4. Run the task.
%   5. At the end, measure the position of the car from the tape.
%      Compare it with the calculated position.
%
% QUESTIONS
%   a. Where does the car stop? Where does the car calculate that it stops?
%   b. Find a wall that shows two times on the map. Which error causes this?
%   c. Set useCamera = false (below). The car then counts the turn steps.
%      Is the map better or worse?
%   d. How can the car correct its pose when it returns to the start?
%      (Task 5 shows one method.)

assert(S.cmPerStep > 0 && S.degPerTurn > 0, "Do task 2 first. Write S.cmPerStep and S.degPerTurn in SETTINGS.");
useCamera = true;                       % false = count the turn steps only
side = 10;                              % Forward steps on each side of the square
map = newMap(500, 5);                   % 5 m x 5 m, cells of 5 cm
x = 0;                                  % Calculated pose: x (cm),
y = 0;                                  %   y (cm),
heading = 0;                            %   heading (degrees, 0 = start direction)
estimatedPath = [x y];
start = truePose(car);                  % Simulator only: the true start pose
truePath = [0 0];

for corner = 1:4
    % 1. Scan at the corner: the sensor head turns from the right to the left
    car.pan(15);
    pause(0.6);                         % The first move is long: wait for the servo
    for pan = 15:10:165
        car.pan(pan);
        pause(0.15);
        map = addReading(map, x, y, heading + sensorAngle(pan, S), readSonar(car, S, 3), S);
    end
    car.pan(90);
    pause(0.3);

    % 2. Move along one side. Add one sonar value at each step.
    for i = 1:side
        range = readSonar(car, S, 3);
        map = addReading(map, x, y, heading, range, S);
        if ~(range >= 25), break; end   % An object is near (or no sonar value): stop this side
        stepCar(car, S.forward, S);
        x = x + S.cmPerStep * cosd(heading);
        y = y + S.cmPerStep * sind(heading);
        estimatedPath(end+1, :) = [x y]; %#ok<AGROW>
        p = truePose(car, start);
        truePath(end+1, :) = p(1:2); %#ok<AGROW>
    end

    % 3. Turn 90 degrees to the left. The camera measures each turn step.
    %    The car turns until the calculated heading is near the target
    %    heading. This is feedback control.
    %    The car does not turn more than 2 times the usual number of steps.
    target = 90 * corner;
    photo = takePhoto(car);
    maxTurnSteps = 2 * round(90 / S.degPerTurn) + 2;
    for i = 1:maxTurnSteps
        if heading >= target - S.degPerTurn / 2
            break
        end
        stepCar(car, S.turnLeft, S);
        newPhoto = takePhoto(car);
        turn = turnFromPhotos(photo, newPhoto, S);
        if ~useCamera || isnan(turn)
            turn = S.degPerTurn;        % Not enough corners: use the step count
        end
        heading = heading + turn;
        photo = newPhoto;
    end
    if heading < target - S.degPerTurn / 2
        car.stop();
        error("The car did not turn 90 degrees. Do task 2 again. If the car turns to the " + ...
            "right, ask your TA to run checkCar.m.");
    end
    fprintf("Corner %d: calculated pose x = %.0f cm, y = %.0f cm, heading = %.0f deg\n", ...
        corner, x, y, heading);
end

% FIGURE 4.1: one panel
labeledFigure(car, 4, 1, "Square path and sonar map", [1 1], [80 80 900 650]);
ax = nexttile;
showMap(map, ax);
hold(ax, "on");
h = plot(ax, estimatedPath(:, 1), estimatedPath(:, 2), "b.-", "LineWidth", 1.5, ...
    "DisplayName", "calculated path (dead reckoning)");
if isa(car, "ElegooSimCar")
    h(end+1) = plot(ax, truePath(:, 1), truePath(:, 2), "m--", "LineWidth", 1, ...
        "DisplayName", "true path (simulator only)");
end
h(end+1) = plot(ax, 0, 0, "go", "MarkerFaceColor", "g", "DisplayName", "start");
h = [h, gridLegendItems(ax), drawSimRoom(car, start, ax)];
axis(ax, mapLimits(map));               % Show only the area of the grid
legend(ax, h, "Location", "eastoutside");
title(ax, "Sonar map at the calculated poses");

fprintf("Calculated end position: x = %.0f cm, y = %.0f cm.\n", estimatedPath(end, 1), estimatedPath(end, 2));
if isa(car, "ElegooSimCar")
    fprintf("True end position (simulator only): x = %.0f cm, y = %.0f cm.\n", truePath(end, 1), truePath(end, 2));
else
    disp("Measure the true end position from the tape on the floor.");
end
result = struct("estimatedPath", estimatedPath, "truePath", truePath, ...
    "endHeadingDeg", heading, "map", map);
end


%% ========================================================================
%  TASK 5: MAKE A CAMERA MAP WITH VISUAL SLAM
%  ========================================================================
function result = task5_visualSlam(car, S)
% PURPOSE
%   Visual SLAM makes a map and finds the car in the map at the same time.
%   MATLAB "monovslam" does these steps with one camera:
%     - It matches corners in the photos (as in task 4).
%     - It calculates the 3-D position of the corners (map points).
%     - It calculates the camera pose at "key frames" (selected photos).
%     - It identifies a place that it saw before. Then it corrects the map.
%       This is "loop closure".
%   One camera cannot measure size. In one photo, a small object near the
%   camera and a large object far from the camera can look the same. At the
%   end, the sonar measures the scale of the map.
%
% PROCEDURE
%   1. Find 2.5 m x 2.5 m of clear floor. Put the car 0.5 m from the right
%      edge of this area, at the middle of the edge. Point the car along the
%      edge. The car moves on a circle to its left. The circle is
%      approximately 1.6 m wide. The room must have furniture, posters or
%      boxes. A blank wall or a floor that
%      reflects light gives the camera no corners.
%   2. For the real car, calibrate the camera one time. Type
%      help calibrateElegooCamera. Do the procedure in the help text. The
%      function saves elegooCameraIntrinsics.mat in the folder of this file.
%      A calibrated camera gives a more accurate map.
%   3. Run the task. It takes approximately 5 minutes in the simulator. The
%      real car can need more time. The car does
%      these steps:
%        a. It moves 4 steps forward and 4 steps back until the map starts.
%           The camera must see the room from two positions to measure depth.
%           This step can use 200 to 250 photos (simulator).
%        b. It moves on a large circle (approximately 1.25 turns).
%        c. It moves back until an object is 120 cm in front of it. Then it
%           moves straight to the object. The sonar measures the scale.
%   4. Sometimes SLAM cannot find the position of the car (status
%      "TrackingLost"). The first time, the car moves back along its path
%      until SLAM finds the position again. If SLAM loses the position
%      again on the circle, the car continues on the circle past that
%      place. Then SLAM can identify a known place later.
%
% QUESTIONS
%   a. Compare the SLAM path with the path of task 4. Which path returns
%      nearer to the start?
%   b. Where are there many map points? Where are there no points? Why?
%   c. What occurs to the map if the scale has an error of 20%?
%   d. Set S.circle = [125 160 0.15] (smaller circle) or
%      [145 160 0.15] (larger circle). Which circle gives a better map?
%
% NOTE: Your TA has a full version with a dashboard (visualSlamDemo.m).

% 1. Camera model
photo = takePhoto(car);
imageSize = size(photo, 1:2);
if isa(car, "ElegooSimCar")
    camera = car.cameraIntrinsics();            % The simulator gives the exact camera
elseif isfile(calibrationFile())
    saved = load(calibrationFile());            % From calibrateElegooCamera (best)
    camera = saved.intrinsics;
else
    f = (imageSize(2) / 2) / tand(S.cameraFovDeg / 2);   % An approximate camera
    camera = cameraIntrinsics([f f], imageSize([2 1]) / 2 + 0.5, imageSize);
    disp("NOTE: No camera calibration. The map is less accurate. Type: help calibrateElegooCamera");
end
assert(isequal(camera.ImageSize, imageSize), ...
    "Calibrate the camera at the photo size (%d x %d).", imageSize(2), imageSize(1));
% The photos are corrected for lens distortion. SLAM then uses a perfect camera.
pinhole = cameraIntrinsics(camera.FocalLength, camera.PrincipalPoint, camera.ImageSize);
vslam = monovslam(pinhole, MaxNumPoints = 1000, TrackFeatureRange = [30 120], ...
    LoopClosureThreshold = 40);

% 2. Values that the loop records
maxPhotos = 500;                % The car stops after this number of photos
phase = "start";                % start -> circle -> scale
range = readSonar(car, S);
ranges = nan(maxPhotos, 1);     % Sonar value at each photo
phases = strings(maxPhotos, 1);  % Phase at each photo
statuses = strings(maxPhotos, 1);% SLAM status at each photo
trueCamera = nan(maxPhotos, 3); % Simulator only: true camera pose
resetAt = 0;                    % Photo number of the last SLAM restart
lostCount = 0;                  % Number of photos in sequence with no position
wasLost = false;                % true while SLAM cannot find the position
foundAgain = 0;                 % Number of times that SLAM found the position again
backStepsLeft = 8;              % Steps back on the path that are permitted for one loss
skipStepsLeft = 25;             % Circle steps without a position that are permitted for one loss
history = zeros(0, 3);          % Steps after the map started
heading = NaN;                  % Camera heading from SLAM (degrees)
turned = 0;                     % Angle of the circle until now (degrees)
scaleSteps = 0;
backedUp = false;
stopReason = "";                % Not empty if the car stopped before the end
circlePhotos = 0;               % Photos in the circle phase
noSonar = 0;                    % Sonar readings in sequence with no value

% FIGURE 5.1: live view during the run, two panels, (a) and (b)
labeledFigure(car, 5, 1, "Visual SLAM live view (changes during the run)", [1 2], [80 80 1250 520]);
axCam = nexttile;
axTop = nexttile;

for k = 1:maxPhotos
    % 3. Select the next step
    isBackStep = false;
    if wasLost && lostCount == 0
        % SLAM found the position again. After each new loss, the car can
        % again do 8 steps back and 25 circle steps.
        wasLost = false;
        foundAgain = foundAgain + 1;
        backStepsLeft = 8;
        skipStepsLeft = 25;
        fprintf("SLAM found the position again (%d time(s)).\n", foundAgain);
        if foundAgain > 6
            stopReason = "SLAM loses the position again and again.";
            disp(stopReason + " The car stops.");
            break
        end
    end
    if phase ~= "start" && lostCount >= 2
        % SLAM cannot find the position of the car.
        %  - The first time: move back along the path to the last place
        %    where SLAM found the position.
        %  - If SLAM loses the position again on the circle, this place has
        %    too few corners. Continue on the circle past this place. Later,
        %    SLAM can identify a known place.
        wasLost = true;
        driveOn = phase == "circle" && (foundAgain >= 1 || backStepsLeft == 0 || isempty(history));
        if ~driveOn && backStepsLeft > 0 && ~isempty(history)
            move = [-history(end, 1:2) history(end, 3)];   % Do the last step in the opposite direction
            history(end, :) = [];
            backStepsLeft = backStepsLeft - 1;
            isBackStep = true;
        elseif driveOn && skipStepsLeft > 0
            move = S.circle;
            skipStepsLeft = skipStepsLeft - 1;
        else
            stopReason = "SLAM cannot find the position of the car.";
            disp(stopReason + " The car stops.");
            break
        end
    elseif phase == "start"
        % 4 steps forward, then 4 steps back. Change the view after 24 photos.
        if mod(k - 1, 8) < 4 && range > 25
            move = S.forward;
        else
            move = S.backward;
        end
        if mod(k, 24) == 0, move = S.circle; end
        if k > 300
            error("The map did not start after 300 photos. The room has too few corners. " + ...
                "Put furniture, posters or boxes near the car. Calibrate the camera.");
        end
    elseif phase == "circle"
        circlePhotos = circlePhotos + 1;
        if circlePhotos == 40 && turned < 20
            error("The car does not turn left on the circle. Make S.circle(1) smaller " + ...
                "than S.circle(2). If the car turns right, ask your TA to run checkCar.m.");
        end
        if circlePhotos > 250
            disp("NOTE: The circle is not complete after 250 photos. The sonar measures the scale now.");
            phase = "scale";
        end
        if ~(range >= 45)
            move = S.backward;          % An object is near (or no sonar value): move back
        else
            move = S.circle;
        end
    else
        % Scale: move back until the object is 120 cm away (10 steps or less).
        % Then move to the object until it is 50 cm away (30 steps in total).
        scaleSteps = scaleSteps + 1;
        if ~backedUp && range < 120 && scaleSteps <= 10
            move = S.backward;
        elseif range > 50 && scaleSteps <= 30
            backedUp = true;
            move = S.forward;
        else
            break
        end
    end

    % 4. Move, take a photo, read the sonar
    stepCar(car, move, S);
    photo = takePhoto(car);
    if any(camera.RadialDistortion) || any(camera.TangentialDistortion)
        photo = undistortImage(photo, camera, OutputView = "same");
    end
    range = readSonar(car, S);
    noSonar = (noSonar + 1) * isnan(range);
    if noSonar >= 3
        error("The sonar gives no value. Examine the sonar and its cable.");
    end
    ranges(k) = range;
    phases(k) = phase;
    if backedUp, phases(k) = "scale in"; end
    if isa(car, "ElegooSimCar"), trueCamera(k, :) = car.cameraPose(); end
    if phase ~= "start" && ~isBackStep, history(end+1, :) = move; end %#ok<AGROW>

    % 5. Give the photo to SLAM
    % Wait until SLAM finishes this photo. Then the result below is complete.
    addFrame(vslam, photo);
    waitTimer = tic;
    while ~isDone(vslam) && toc(waitTimer) < 20
        pause(0.02);
    end
    if phase == "start"
        if numel(poses(vslam)) > 1
            phase = "circle";
            disp("The map started. The car moves on a circle.");
        elseif k - resetAt >= 4
            % No map after 4 photos: start again with a new SLAM object.
            % (Do not use reset(vslam): after it, each photo waits 20 s.)
            delete(vslam);
            vslam = monovslam(pinhole, MaxNumPoints = 1000, TrackFeatureRange = [30 120], ...
                LoopClosureThreshold = 40);
            resetAt = k;
        end
    end
    status = "map not started";
    if phase ~= "start"
        status = string(checkStatus(vslam));
        lostCount = (lostCount + 1) * (status == "TrackingLost");
    end
    statuses(k) = status;

    % 6. Heading from the newest key frame. Add the angle of the circle.
    if phase ~= "start" && hasNewKeyFrame(vslam)
        camPoses = poses(vslam);
        R = camPoses(end).R;            % Camera axes: x right, y down, z forward
        newHeading = atan2d(-R(1, 3), R(3, 3));
        if phase == "circle" && ~isnan(heading)
            turned = turned + mod(newHeading - heading + 180, 360) - 180;
        end
        heading = newHeading;
        if phase == "circle" && turned >= 450
            phase = "scale";
            disp("The circle is complete. The sonar measures the scale.");
        end
        % Top view: key frames (blue) and map points (gray)
        C = vertcat(camPoses.Translation);
        X = mapPoints(vslam);
        plot(axTop, X(:, 3), -X(:, 1), ".", "Color", [0.6 0.6 0.6], "MarkerSize", 3, ...
            "DisplayName", "map points (corners in 3-D)");
        hold(axTop, "on");
        plot(axTop, C(:, 3), -C(:, 1), "b.-", "DisplayName", "key frames (camera path)");
        hold(axTop, "off");
        grid(axTop, "on");
        xlabel(axTop, "forward (SLAM units)");
        ylabel(axTop, "left (SLAM units)");
        legend(axTop, "Location", "southoutside", "NumColumns", 2);
        % Some map points have large errors and are very far away. Set the axis
        % limits from the middle 96% of the points and the key frames.
        P = sort([X(:, 3), -X(:, 1); C(:, 3), -C(:, 1)]);
        lo = P(max(1, round(0.02 * size(P, 1))), :);
        hi = P(max(1, round(0.98 * size(P, 1))), :);
        pad = 0.1 * max(hi - lo) + 0.01;
        xlim(axTop, [lo(1) - pad, hi(1) + pad]);
        ylim(axTop, [lo(2) - pad, hi(2) + pad]);
        daspect(axTop, [1 1 1]);
        title(axTop, sprintf("(b) SLAM top view: %d key frames, %d map points (no scale yet)", ...
            size(C, 1), size(X, 1)));
    end
    imshow(photo, "Parent", axCam);
    title(axCam, sprintf("(a) Photo %d | phase: %s | SLAM: %s | circle: %.0f deg", k, phase, status, turned));
    drawnow limitrate
end
car.stop();
disp("The car stopped. SLAM completes the map ...");
waitTimer = tic;
while ~isDone(vslam) && toc(waitTimer) < 60
    pause(0.2);
end

% 7. Scale. When the car moves straight to an object, the sonar distance
%    decreases by the distance that the car moves. A line fit of
%    (sonar distance) against (SLAM distance) gives cm per SLAM unit.
[camPoses, viewIds] = poses(vslam);
if numel(camPoses) < 3
    error("SLAM did not make a map. Add objects with texture, calibrate the camera, or move to a different room.");
end
keyPhoto = viewIds + resetAt;           % Photo number of each key frame
C = vertcat(camPoses.Translation);
use = find(phases(keyPhoto) == "scale in" & ranges(keyPhoto) > 8 & ranges(keyPhoto) < 140);
cmPerUnit = NaN;
if numel(use) >= 3
    slamDistance = vecnorm(C(use, :) - C(use(1), :), 2, 2);
    if max(slamDistance) > 0
        fit = polyfit(slamDistance, ranges(keyPhoto(use)), 1);
        if fit(1) < 0                   % The sonar distance must decrease
            cmPerUnit = -fit(1);
        end
    end
end
scaleKnown = isfinite(cmPerUnit);
if scaleKnown
    unit = "cm";
    scaleText = sprintf("Scale: %.0f cm for each SLAM unit.", cmPerUnit);
else
    unit = "SLAM units";                % Show the map without a scale
    cmPerUnit = 1;
    scaleText = "NO SCALE: the map is in SLAM units.";
    disp("NOTE: The sonar did not measure the scale. The map stays in SLAM units.");
end
fprintf("Map: %d key frames. %s\n", numel(camPoses), scaleText);
if stopReason ~= ""
    warning("Task 5 stopped early: %s The map is not complete. Do the task again.", stopReason);
    scaleText = scaleText + " STOPPED EARLY.";
end

% 8. Top view of the map in cm. Show the map in the frame of the first key
%    frame: the map starts at (0, 0) and the car points forward (+x).
R1 = camPoses(1).R;
t1 = camPoses(1).Translation;
toFirstFrame = @(P) (P - t1) * R1;              % World points to the first camera frame
C1 = toFirstFrame(C);
pathCm = [C1(:, 3), -C1(:, 1)] * cmPerUnit;     % forward = camera z, left = -camera x
X = toFirstFrame(mapPoints(vslam)) * cmPerUnit;
heightCm = S.cameraHeightCm - X(:, 2);          % Camera y points down
isObstacle = heightCm > 4 & heightCm < 80;      % Not the floor, not the ceiling
if ~scaleKnown
    isObstacle = true(size(heightCm));          % No scale: the height is not known
end
% FIGURE 5.2: the final map, one panel
tl = labeledFigure(car, 5, 2, "Visual SLAM map (final)", [1 1], [80 80 950 650]);
subtitle(tl, string(tl.Subtitle.String) + sprintf(". %d key frames. ", numel(camPoses)) + scaleText);
ax = nexttile;
pointsText = "map points 4 cm to 80 cm above the floor (objects)";
if ~scaleKnown
    pointsText = "all map points (no scale: floor and ceiling points also)";
end
h = plot(ax, X(isObstacle, 3), -X(isObstacle, 1), "r.", "MarkerSize", 4, "DisplayName", pointsText);
hold(ax, "on");
h(2) = plot(ax, pathCm(:, 1), pathCm(:, 2), "b-", "LineWidth", 2, "DisplayName", "SLAM path (key frames)");
h(3) = plot(ax, 0, 0, "go", "MarkerFaceColor", "g", "DisplayName", "start of the map");
truePathCm = [];
if isa(car, "ElegooSimCar") && scaleKnown           % The true path is in cm
    first = trueCamera(keyPhoto(1), :);
    first = [100 * first(1:2), rad2deg(first(3))];
    truePathCm = toStartFrame(100 * trueCamera(keyPhoto, 1:2), first);
    h(4) = plot(ax, truePathCm(:, 1), truePathCm(:, 2), "k--", "DisplayName", "true path (simulator only)");
    h = [h, drawSimRoom(car, first, ax)];
end
axis(ax, "equal");
grid(ax, "on");
halfWidth = 250;                                % cm
if ~scaleKnown
    halfWidth = 1.5 * max(max(pathCm, [], 1) - min(pathCm, [], 1)) + eps;   % SLAM units
end
xlim(ax, halfWidth * [-1 1] + mean(pathCm(:, 1)));
ylim(ax, halfWidth * [-1 1] + mean(pathCm(:, 2)));
xlabel(ax, "forward (" + unit + ")");
ylabel(ax, "left (" + unit + ")");
legend(ax, h, "Location", "southoutside", "NumColumns", 2);
title(ax, "Top view of the map");

nPhotos = find(phases ~= "", 1, "last");       % Photos that the car took
if ~scaleKnown
    cmPerUnit = NaN;
end
result = struct("vslam", vslam, "cmPerUnit", cmPerUnit, "unit", unit, "path", pathCm, ...
    "mapPoints", X, "truePathCm", truePathCm, "sonarCm", ranges(1:nPhotos), ...
    "phase", phases(1:nPhotos), "status", statuses(1:nPhotos), ...
    "completed", stopReason == "", "stopReason", stopReason);
end


%% ========================================================================
%  HELPER FUNCTIONS (all tasks use these functions)
%  ========================================================================
function [car, S] = connectCar(useSimulator, S, task)
% Connect to the real car or to the simulator. Both have the same commands.
% In the simulator, tasks 1 to 4 start near furniture (the sonar range is short).
% Task 5 starts in the open middle of the room (the car moves on a circle).
if useSimulator
    if task == 5
        car = ElegooSimCar("Pose", [2.6 1.15 0]);   % [x m, y m, heading rad] in the room
    else
        car = ElegooSimCar("Pose", [3.4 1.3 0]);
    end
    S.cameraFovDeg = car.HorizontalFovDeg;   % The simulator camera is known exactly
else
    % Find ElegooRobotCar.m. In the student kit, it is in this folder. In the
    % course repository, it is in the basic_wireless_control folder.
    if exist("ElegooRobotCar", "file") ~= 2
        addpath(fullfile(fileparts(mfilename("fullpath")), "..", "basic_wireless_control"));
    end
    try
        car = ElegooRobotCar(S.host);
    catch problem
        error("MATLAB cannot connect to the car at %s. Make sure that the computer is on the " + ...
            "Wi-Fi network of your car (ELEGOO-...) and that the car is ON. Disconnect from a VPN. " + ...
            "On a Mac, allow MATLAB in System Settings > Privacy & Security > Local Network. " + ...
            "(MATLAB message: %s)", S.host, problem.message);
    end
end
car.stop();
car.setWatchdog(1000);          % Safety: the motors stop if MATLAB stops for 1 s
car.pan(90);                    % Sensor head points forward
pause(0.5);
fprintf("Connected. The sonar shows %g cm.\n", readSonar(car, S));
photo = takePhoto(car);
fprintf("The camera gives photos of %d x %d pixels.\n", size(photo, 2), size(photo, 1));
if ~useSimulator && isfile(calibrationFile())
    % Use the calibrated field of view to measure the turns (task 2 and task 4).
    saved = load(calibrationFile());
    S.cameraFovDeg = 2 * atand(saved.intrinsics.ImageSize(2) / 2 / saved.intrinsics.FocalLength(1));
    fprintf("Camera calibration found: field of view = %.1f degrees.\n", S.cameraFovDeg);
end
end

function safeStop(car)
% Stop the motors. If the connection has a problem, do not show a second error.
try
    car.stop();
catch
end
end

function stepCar(car, move, S)
% Do one step: motor pulse, stop, then wait until the car does not move.
% move = [left PWM, right PWM, time in seconds]
car.drive(move(1), move(2));
pause(move(3));
car.stop();
pause(S.settleSec);
end

function cm = readSonar(car, S, n)
% Read the sonar n times. Give the median distance (cm). NaN = no answer.
% The real sonar sends 0 when no echo comes back (no object in range).
% This function then gives S.sonarMaxCm, as the simulator does.
if nargin < 3, n = 1; end
values = nan(1, n);
for i = 1:n
    reply = car.readDistance();
    if isfield(reply, "distance") && isnumeric(reply.distance) && isscalar(reply.distance)
        values(i) = double(reply.distance);
    end
end
values(values == 0 | values > S.sonarMaxCm) = S.sonarMaxCm;
cm = median(values, "omitnan");
end

function photo = takePhoto(car)
% Take one photo. If the Wi-Fi connection has a short problem, take the
% photo again one time.
try
    photo = car.capture();
catch
    pause(0.3);
    photo = car.capture();
end
end

function [deg, nMatches, p1, p2] = turnFromPhotos(photo1, photo2, S)
% Calculate the turn of the car between two photos (visual odometry).
% deg > 0: the car turned to the LEFT. deg = NaN: not enough corners match.
% Method: find the same corners in the two photos. The direction to a corner is
%   atan(pixels from the image center / focal length).
% When the car turns 10 degrees, all directions change by 10 degrees.
g1 = im2gray(photo1);
g2 = im2gray(photo2);
[f1, c1] = extractFeatures(g1, detectORBFeatures(g1));
[f2, c2] = extractFeatures(g2, detectORBFeatures(g2));
% The photos of the car have noise. With these MatchThreshold and MaxRatio
% values, the function finds more matches.
pairs = matchFeatures(f1, f2, Unique = true, MatchThreshold = 40, MaxRatio = 0.8);
p1 = c1.Location(pairs(:, 1), :);       % One corner in photo 1 ...
p2 = c2.Location(pairs(:, 2), :);       % ... and the same corner in photo 2
nMatches = size(pairs, 1);
width = size(photo1, 2);
focal = (width / 2) / tand(S.cameraFovDeg / 2);    % Focal length (pixels)
dir1 = atan2d(p1(:, 1) - width / 2, focal);         % Direction to each corner
dir2 = atan2d(p2(:, 1) - width / 2, focal);
deg = median(dir2 - dir1);              % Incorrect matches have almost no effect on the median
if nMatches < 10
    deg = NaN;
end
end

function a = sensorAngle(pan, S)
% Change the pan servo angle to the sensor direction: 0 = forward, positive = left.
a = pan - 90;
if ~S.panLeftIsBigger
    a = -a;
end
end

function map = newMap(sizeCm, cellCm)
% Make an empty occupancy grid. The start position of the car is the center.
% Each cell records two counts:
%   hits   = number of sonar beams that stopped in the cell (an object)
%   misses = number of sonar beams that went through the cell (free)
n = ceil(sizeCm / cellCm);
map.cellCm = cellCm;
map.cornerCm = -n * cellCm / 2;         % x and y of the lower-left corner
map.hits = zeros(n, n);                 % Rows = y, columns = x
map.misses = zeros(n, n);
end

function map = addReading(map, x, y, headingDeg, rangeCm, S)
% Add one sonar value at the calculated pose. The sensor is ahead of the axle.
sx = x + S.sensorAheadCm * cosd(headingDeg);
sy = y + S.sensorAheadCm * sind(headingDeg);
map = addSonarRay(map, sx, sy, headingDeg, rangeCm, S);
end

function map = addSonarRay(map, xCm, yCm, angleDeg, rangeCm, S)
% Add one sonar beam to the occupancy grid.
% The beam is approximately 15 degrees wide. The cells in the beam are free.
% The cell at the end of the beam center is occupied. If no object is in
% range, all cells in the beam are free.
if isnan(rangeCm), return; end
freeCm = min(rangeCm, S.sonarMaxCm) - map.cellCm;
[d, a] = meshgrid(0:map.cellCm / 2:freeCm, angleDeg + (-7.5:2.5:7.5));   % Points in the beam
[row, col] = cellOf(map, xCm + d .* cosd(a), yCm + d .* sind(a));
ok = ~isnan(row);
idx = unique(sub2ind(size(map.misses), row(ok), col(ok)));
map.misses(idx) = map.misses(idx) + 1;
if rangeCm < S.sonarMaxCm - 5           % The sound came back from an object
    [row, col] = cellOf(map, xCm + rangeCm * cosd(angleDeg), yCm + rangeCm * sind(angleDeg));
    if ~isnan(row)
        map.hits(row, col) = map.hits(row, col) + 1;
    end
end
end

function [row, col] = cellOf(map, x, y)
% Find the cell that contains the point (x, y). NaN = the point is outside the map.
n = size(map.hits, 1);
col = floor((x - map.cornerCm) / map.cellCm) + 1;
row = floor((y - map.cornerCm) / map.cellCm) + 1;
outside = col < 1 | col > n | row < 1 | row > n;
col(outside) = NaN;
row(outside) = NaN;
end

function showMap(map, ax)
% Show the occupancy grid: white = free, black = occupied, gray = unknown.
seen = map.hits + map.misses;
occupied = map.hits ./ seen;            % Part of the beams that stopped in the cell
occupied(seen == 0) = 0.5;              % No beam: unknown (gray)
n = size(map.hits, 1);
centers = map.cornerCm + map.cellCm * ((1:n) - 0.5);
imagesc(ax, centers, centers, 1 - occupied, [0 1]);
colormap(ax, gray);
axis(ax, "xy", "equal", "tight");
xlabel(ax, "x = forward (cm)");
ylabel(ax, "y = left (cm)");
end

function pose = truePose(car, start)
% SIMULATOR ONLY: the true pose of the car [x cm, y cm, heading deg].
%   truePose(car)          gives the pose in the room.
%   truePose(car, start)   gives the pose from the start pose (x = start direction).
% The real car gives NaN. Use a tape measure for the real car.
if ~isa(car, "ElegooSimCar")
    pose = [NaN NaN NaN];
    return
end
pose = [100 * car.Pose(1:2), rad2deg(car.Pose(3))];
if nargin == 2
    pose = [toStartFrame(pose(1:2), start), mod(pose(3) - start(3) + 180, 360) - 180];
end
end

function xy = toStartFrame(xy, start)
% Change room points (cm) to the frame of the start pose: x = start direction, y = left.
d = xy - start(1:2);
c = cosd(start(3));
s = sind(start(3));
xy = [d(:, 1) * c + d(:, 2) * s, -d(:, 1) * s + d(:, 2) * c];
end

function h = drawSimRoom(car, start, ax)
% SIMULATOR ONLY: draw the true walls and furniture (green) to check the map.
% h is one line for the legend. On the real car, h is empty.
h = gobjects(1, 0);
if ~isa(car, "ElegooSimCar"), return; end
tmp = figure("Visible", "off");
car.showFloorPlan(axes(tmp));           % Room outline in meters
outlines = findobj(tmp, "Type", "line");
hold(ax, "on");
for i = 1:numel(outlines)
    xy = toStartFrame(100 * [outlines(i).XData(:), outlines(i).YData(:)], start);
    outline = plot(ax, xy(:, 1), xy(:, 2), "-", "Color", [0.1 0.7 0.1], "LineWidth", 1, ...
        "HandleVisibility", "off");
    if i == 1
        outline.DisplayName = "true walls and furniture (simulator only)";
        outline.HandleVisibility = "on";
        h = outline;
    end
end
close(tmp);
end

function tl = labeledFigure(car, task, number, titleText, layout, position)
% Make one figure with a clear label, for example
%   "Task 3, Figure 3.1: Sonar map from one position"
% The subtitle shows the data source (simulator or real car). Each panel
% in the figure has a letter: (a), (b), (c), (d).
source = "real car";
if isa(car, "ElegooSimCar")
    source = "simulator";
end
label = sprintf("Task %d, Figure %d.%d: %s", task, task, number, titleText);
figure("Name", label, "NumberTitle", "off", "Position", position, "Color", "w");
tl = tiledlayout(layout(1), layout(2), "TileSpacing", "compact", "Padding", "compact");
title(tl, label, "FontWeight", "bold", "FontSize", 13);
subtitle(tl, "Data from the " + source);
end

function file = calibrationFile()
% Find elegooCameraIntrinsics.mat (from calibrateElegooCamera). Look first in
% the folder of this file, then in the MATLAB current folder.
file = fullfile(fileparts(mfilename("fullpath")), "elegooCameraIntrinsics.mat");
if ~isfile(file)
    file = fullfile(pwd, "elegooCameraIntrinsics.mat");
end
end

function limits = mapLimits(map)
% Give the axis limits of the occupancy grid: [xMin xMax yMin yMax] (cm).
widthCm = size(map.hits, 1) * map.cellCm;
limits = map.cornerCm + [0 widthCm 0 widthCm];
end

function h = gridLegendItems(ax)
% Make legend items for the three cell colors of the occupancy grid.
h = patch(ax, NaN, NaN, "w", "EdgeColor", "k", "DisplayName", "free cell (white)");
h(2) = patch(ax, NaN, NaN, "k", "DisplayName", "occupied cell (black)");
h(3) = patch(ax, NaN, NaN, [0.5 0.5 0.5], "EdgeColor", "none", "DisplayName", "unknown cell (gray)");
end
