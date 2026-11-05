function report = checkCar(host)
% CHECKCAR  Five-minute hardware check of one ELEGOO car over Wi-Fi.
%   report = checkCar()                 % car at 192.168.4.1
%   report = checkCar("192.168.4.1")
% The function asks yes/no questions in the Command Window. At the end it
% prints the four setup values for the top of ElegooRobotCar.m.
% Keep the motor switch OFF until the function asks for it.

if nargin < 1, host = "192.168.4.1"; end
car = ElegooRobotCar(host);
stopAtEnd = onCleanup(@() car.stop());
car.stop();
car.setWatchdog(1000);
car.pan(90);
report = struct();

%% 1. Connection and timing
fprintf("\n1. CONNECTION\n");
assert(car.ping().ok == 1, "The Arduino does not answer. Check the UART wires.");
report.pingSec = medianTime(@() car.ping(), 5);
report.photoSec = medianTime(@() car.capture(), 5);
photo = car.capture();
report.photoSize = [size(photo, 2) size(photo, 1)];
fprintf("   Command round trip %.0f ms. One photo %.0f ms (%d x %d).\n", ...
    1000 * report.pingSec, 1000 * report.photoSec, report.photoSize);
fprintf("   Battery %.2f V (shield divider, approximate).\n", car.readBattery().battery);

%% 2. Sonar and gyro (motor switch can stay OFF)
fprintf("\n2. SENSORS\n");
ask("Hold your hand 20 cm in front of the sonar, then push Enter.", false);
report.sonarHandCm = car.readDistance().distance;
ask("Point the car at open space (nothing within 2 m), then push Enter.", false);
report.sonarOpenCm = car.readDistance().distance;
fprintf("   Hand: %d cm. Open space: %d cm (0 = no echo, the code uses 150).\n", ...
    report.sonarHandCm, report.sonarOpenCm);
report.gyroAtRest = car.readIMU().gyro(:)';
fprintf("   Gyro at rest: [%d %d %d] raw (131 raw = 1 deg/s).\n", report.gyroAtRest);

%% 3. Photo direction
fprintf("\n3. PHOTO DIRECTION\n");
figure("Name", "checkCar: photo", "NumberTitle", "off");
ask("Hold a page with printed text in front of the camera, then push Enter.", false);
imshow(car.capture());
title("Is the photo upright? Is the text a mirror image?");
drawnow;
upright = ask("Is the photo upright (not upside down)?");
mirrored = ask("Is the text a mirror image?");
vflip = ElegooRobotCar.PhotoVFlip;
mirror = ElegooRobotCar.PhotoMirror;
if ~upright && ~mirrored         % turned 180 degrees: flip and mirror
    vflip = 1 - vflip;
    mirror = 1 - mirror;
elseif ~upright                  % upside down and mirrored: flip only
    vflip = 1 - vflip;
elseif mirrored                  % upright and mirrored: mirror only
    mirror = 1 - mirror;
end
car.setCamera("vflip", vflip);
car.setCamera("hmirror", mirror);
imshow(car.capture());
title("Photo with the new setting: upright and readable?");
drawnow;
if ~ask("Is the photo now upright and readable?")
    warning("The photo direction is still not correct. Do this step again.");
end

%% 4. Wheel sides (car lifted, motor switch ON)
fprintf("\n4. WHEELS\n");
ask("LIFT the car so that the wheels are off the floor. Set the motor switch ON. Push Enter.", false);
car.drive(120, 0);
pause(0.6);
car.stop();
swap = ElegooRobotCar.SwapSides;
if ~ask("drive(120, 0) was sent. Did the LEFT wheels turn?")
    swap = ~swap;
end
drive = @(l, r) car.drive(l, r);
if swap ~= ElegooRobotCar.SwapSides
    drive = @(l, r) car.drive(r, l);   % the file is not correct yet: correct it for this test
end

car.drive(100, 100);
fprintf("   Both wheels turn now. MATLAB sends nothing for 2 s ...\n");
pause(2);
report.watchdogOk = ask("Did the wheels stop after approximately 1 s?");
car.stop();
if ~report.watchdogOk
    warning("The motor watchdog does not work. Do not let students drive this car.");
end

%% 5. Turn on the floor
fprintf("\n5. TURN LEFT ON THE FLOOR\n");
ask("Put the car on the floor, 1 m from furniture, with clear floor around it. Push Enter.", false);
before = im2gray(car.capture());
gyroZ = [];
timer = tic;
drive(-130, 130);                 % the same turn step as the template
while toc(timer) < 0.8
    imu = car.readIMU();
    gyroZ(end+1) = imu.gyro(3); %#ok<AGROW>
    drive(-130, 130);             % send again: the watchdog stays fed
end
car.stop();
pause(0.3);
after = im2gray(car.capture());
report.turnedLeft = ask("Did the car turn LEFT (counter-clockwise, seen from above)?");
report.gyroSignLeft = sign(median(gyroZ));
report.cameraTurnDeg = cameraTurn(before, after, 60);
fprintf("   Gyro z during the left turn: %+d (median raw). Camera: %+.0f deg (+ = left).\n", ...
    round(median(gyroZ)), report.cameraTurnDeg);
if report.turnedLeft && report.cameraTurnDeg < 0
    warning("The camera sees a RIGHT turn. The photo is mirrored: do step 3 again.");
end

%% Result
report.SwapSides = swap;
report.PhotoVFlip = vflip;
report.PhotoMirror = mirror;
fprintf("\nRESULT: write these values at the top of ElegooRobotCar.m:\n");
fprintf("    SwapSides = %s\n", mat2str(swap));
fprintf("    PhotoVFlip = %d\n", vflip);
fprintf("    PhotoMirror = %d\n", mirror);
if swap ~= ElegooRobotCar.SwapSides || vflip ~= ElegooRobotCar.PhotoVFlip || ...
        mirror ~= ElegooRobotCar.PhotoMirror
    fprintf("    (These are different from the file now. Change the file, then run checkCar again.)\n");
else
    fprintf("    (The file is already correct.)\n");
end
end

function yes = ask(question, isYesNo)
% Ask a question in the Command Window. isYesNo = false: wait for Enter only.
if nargin < 2, isYesNo = true; end
if ~isYesNo
    input("   " + question + " ", "s");
    yes = true;
    return
end
answer = "";
while ~ismember(answer, ["y", "n"])
    answer = lower(strtrim(string(input("   " + question + " (y/n) ", "s"))));
end
yes = answer == "y";
end

function sec = medianTime(fcn, n)
t = zeros(1, n);
for i = 1:n
    timer = tic;
    fcn();
    t(i) = toc(timer);
end
sec = median(t);
end

function deg = cameraTurn(g1, g2, fovDeg)
% Turn between two photos from matched corners (+ = left), as in the template.
[f1, c1] = extractFeatures(g1, detectORBFeatures(g1));
[f2, c2] = extractFeatures(g2, detectORBFeatures(g2));
pairs = matchFeatures(f1, f2, Unique = true, MatchThreshold = 40, MaxRatio = 0.8);
deg = NaN;
if size(pairs, 1) < 10, return; end
width = size(g1, 2);
focal = (width / 2) / tand(fovDeg / 2);
x1 = c1.Location(pairs(:, 1), 1) - width / 2;
x2 = c2.Location(pairs(:, 2), 1) - width / 2;
deg = median(atan2d(x2, focal) - atan2d(x1, focal));
end
