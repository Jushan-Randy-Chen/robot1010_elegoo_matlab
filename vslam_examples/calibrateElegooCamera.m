function intrinsics = calibrateElegooCamera(squareSizeMm, host, flipImage)
% CALIBRATEELEGOOCAMERA  Measure the focal length and the lens distortion of
% the ESP32 camera. Task 5 of elegooMappingTemplate.m uses these values to
% change pixels into 3-D directions. (The TA demo visualSlamDemo.m also uses them.) Do this procedure one time for each car.
%
% PROCEDURE
%   1. Make a checkerboard. Type: calibrateElegooCamera("print")
%      This makes the file checkerboard_print.png (7 x 10 squares) in the
%      folder of this file.
%   2. Print the file on one sheet of paper. Select "Shrink to fit"
%      (Windows) or "Scale to Fit" (Mac). Do not select "Fill". Attach the
%      paper to a flat, stiff board.
%   3. Measure one square with a ruler (mm).
%   4. Connect the computer to the Wi-Fi network of the car.
%   5. Type: calibrateElegooCamera(squareSizeMm)
%      Example: calibrateElegooCamera(25)
%   6. If the live photo is upside down, push F before you keep photos.
%      Then also set S.flipImage = true in elegooMappingTemplate.m.
%   7. Hold the board 20 cm to 60 cm in front of the car. Move and tilt the
%      board. Put it in all areas of the photo, also in the corners.
%   8. Click the figure window. The keys work only when this window is
%      selected. Push SPACE to keep a photo. Keep a photo only when the
%      corners show in green. Keep 15 to 20 photos.
%   9. Push C to calibrate. The function saves the result in
%      elegooCameraIntrinsics.mat in the folder of this file. It saves the
%      photos in the calibrationImages folder in the same folder.
%  10. Do not move elegooCameraIntrinsics.mat. elegooMappingTemplate.m finds
%      it in the same folder.
%
% NOTE: Calibrate at the photo size that task 5 uses. The template shows the
%       photo size when it connects ("The camera gives photos of ...").
% NOTE: Push Q to stop without a calibration.

if nargin >= 1 && (isstring(squareSizeMm) || ischar(squareSizeMm)) && squareSizeMm == "print"
    writePrintableBoard();
    return
end
if nargin < 1 || isempty(squareSizeMm)
    error("calibrateElegooCamera:SquareSize", ...
        "Give the size of one printed square in mm. Example: calibrateElegooCamera(25).");
end
if nargin < 2 || isempty(host), host = "192.168.4.1"; end
if nargin < 3 || isempty(flipImage), flipImage = false; end

% Find ElegooRobotCar.m. In the student kit, it is in this folder. In the
% course repository, it is in the basic_wireless_control folder.
if exist("ElegooRobotCar", "file") ~= 2
    addpath(fullfile(fileparts(mfilename("fullpath")), "..", "basic_wireless_control"));
end
robot = ElegooRobotCar(host);
robot.stop();
robot.setWatchdog(1000);
robot.pan(90);
cleanupObj = onCleanup(@() safeClose(robot));

% Save the results in the folder of this file (not the MATLAB current folder).
here = fileparts(mfilename("fullpath"));
outDir = fullfile(here, "calibrationImages");
if ~isfolder(outDir), mkdir(outDir); end

fig = figure("Name", "Elegoo camera calibration", "Color", "w");
setappdata(fig, "pendingKey", "");
fig.WindowKeyPressFcn = @(f, evt) setappdata(f, "pendingKey", evt.Key);
ax = axes(fig);

images = {};
imagePoints = [];
boardSize = [];
intrinsics = [];

while isvalid(fig)
    frame = robot.capture();
    if flipImage, frame = rot90(frame, 2); end
    [points, thisBoard] = detectCheckerboardPoints(frame, PartialDetections = false);
    found = ~isempty(points) && all(thisBoard > 0) && ...
        (isempty(boardSize) || isequal(thisBoard, boardSize));

    imshow(frame, "Parent", ax);
    hold(ax, "on");
    if found
        plot(ax, points(:, 1), points(:, 2), "g+", "MarkerSize", 6, "LineWidth", 1);
    end
    hold(ax, "off");
    title(ax, sprintf("%dx%d  |  %d photos kept  |  Click this window, then: SPACE = keep  F = flip  C = calibrate  Q = quit", ...
        size(frame, 2), size(frame, 1), numel(images)));
    drawnow;

    key = string(getappdata(fig, "pendingKey"));
    setappdata(fig, "pendingKey", "");
    switch key
        case "space"
            if found
                boardSize = thisBoard;
                images{end+1} = frame; %#ok<AGROW>
                imagePoints = cat(3, imagePoints, points);
                imwrite(frame, fullfile(outDir, sprintf("calib_%02d.png", numel(images))));
                fprintf("Kept photo %d\n", numel(images));
            else
                fprintf("The computer did not find the checkerboard in this photo. The photo is not kept.\n");
            end
        case "f"
            if ~isempty(images)
                fprintf("Push F before you keep a photo. To change it now, start the function again.\n");
            else
                flipImage = ~flipImage;
                fprintf("flipImage = %d\n", flipImage);
            end
        case "c"
            if numel(images) < 8
                fprintf("Keep 8 photos or more first (15 to 20 photos is better).\n");
                continue
            end
            intrinsics = runCalibration(imagePoints, boardSize, squareSizeMm, size(images{1}, 1:2));
            calibFile = fullfile(here, "elegooCameraIntrinsics.mat");
            save(calibFile, "intrinsics", "flipImage");
            fprintf("Saved %s\n", calibFile);
            break
        case "q"
            break
    end
end
if isvalid(fig), close(fig); end
end

function intrinsics = runCalibration(imagePoints, boardSize, squareSizeMm, imageSize)
worldPoints = generateCheckerboardPoints(boardSize, squareSizeMm);
params = estimateCameraParameters(imagePoints, worldPoints, ImageSize = imageSize, ...
    NumRadialDistortionCoefficients = 3, WorldUnits = "mm");
intrinsics = params.Intrinsics;

f = intrinsics.FocalLength;
hfov = 2 * atand(imageSize(2) / 2 / f(1));
fprintf("Focal length: %.1f x %.1f px  (horizontal field of view %.1f deg)\n", f(1), f(2), hfov);
fprintf("Principal point: (%.1f, %.1f) px\n", intrinsics.PrincipalPoint);
fprintf("Radial distortion: %s\n", mat2str(intrinsics.RadialDistortion, 3));
fprintf("Mean reprojection error: %.2f px (below ~0.5 px is good)\n", params.MeanReprojectionError);
if params.MeanReprojectionError > 1
    warning("Reprojection error is high: keep sharper photos, a flatter board, and more image-corner coverage.");
end
if hfov > 100
    warning("This is a very wide lens (%.0f deg). The standard lens model may be poor at the edges; " + ...
        "consider estimateFisheyeParameters.", hfov);
end

figure("Name", "Calibration check", "Color", "w");
tiledlayout(1, 2);
nexttile; showReprojectionErrors(params);
nexttile; showExtrinsics(params, "CameraCentric");
end

function writePrintableBoard()
% 7 x 10 squares (one odd, one even side, so the board orientation is unique).
square = 200;   % pixels per square in the image file
rows = 7;
cols = 10;
[c, r] = meshgrid(0:cols*square-1, 0:rows*square-1);
board = mod(floor(r / square) + floor(c / square), 2) == 1;
board = padarray(board, [square square], true);   % white margin
file = fullfile(fileparts(mfilename("fullpath")), "checkerboard_print.png");
imwrite(board, file);
fprintf("Made %s. Print it on one sheet of paper. Measure one square in mm.\n", file);
end

function safeClose(robot)
try
    robot.stop();
    robot.close();
catch
end
end
