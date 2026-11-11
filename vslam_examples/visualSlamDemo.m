function results = visualSlamDemo(source, options)
% VISUALSLAMDEMO  Visual SLAM with the ELEGOO Smart Robot Car V4 (camera + ultrasonic).
%
%   visualSlamDemo()                  real car at 192.168.4.1
%   visualSlamDemo("sim")             simulated living room, no car needed
%   visualSlamDemo("vslamRun_...")    replay a saved run folder
%   r = visualSlamDemo("sim", AutoRun=true, ShowUI=false)   run without clicking or windows
%   visualSlamDemo("robot", Config=struct("circleInner", 120))   change a setting
%
% WHAT IS VISUAL SLAM?
%   SLAM = Simultaneous Localization And Mapping: the car builds a map of the
%   room and, at the same time, works out where it is in that map. "Visual":
%   the camera is the sensor. MATLAB's monovslam (Computer Vision Toolbox):
%     - finds ORB corners in every photo and matches them with earlier photos
%     - triangulates matched corners into 3-D map points
%     - estimates the camera pose of each key frame (bundle adjustment)
%     - recognizes a place it has seen before and closes the loop
%     - relocalizes: finds itself in the map again after it got lost
%
% WHAT THE CAR DOES ('Start Auto Run')
%   1. Start the map: 4 steps forward, 4 back. One camera must see the scene
%      from two places (parallax) to measure depth. If there is no map after
%      4 photos, SLAM restarts from the newest photo and the car tries again.
%   2. Circle: a big gentle circle (~1.25 turns). The camera looks across the
%      room, and returning to the start lets SLAM close the loop. The car has
%      no wheel encoders, so SLAM measures how far it has turned.
%   3. Scale: one camera cannot tell 1 m from 1 cm. The car backs away from the
%      nearest obstacle and drives straight back at it; the ultrasonic range
%      changes by exactly the distance driven, which converts the map to meters.
%   4. Map: top view of the path and map points, and an occupancy grid
%      (free / occupied / unknown) built from map points and sonar beams.
%   Lost tracking? The car retraces its last steps until SLAM finds itself
%   (relocalization). Lost again at the same place on the circle? It drives on
%   past it, and SLAM recognizes the already-mapped part of the room.
%
% FOR THE REAL CAR
%   - Run calibrateElegooCamera once (focal length and lens distortion).
%   - Start in the middle of an open floor area at least 2.5 m across, in a room
%     with furniture, posters or boxes. Blank walls and glossy floors give the
%     camera nothing to track. If the circle is too big, lower 'Circle inner PWM'.
%   - The car moves in short stop-and-go steps (motor pulse, stop, photo): the
%     ESP32 camera is slow and blurs while moving, and there are no encoders.
%   - Camera addresses (connect your computer to the car Wi-Fi first):
%       live video:   http://192.168.4.1/stream    (open it in a web browser)
%       one photo:    http://192.168.4.1/capture
%     Close the /stream page before you run this demo. While the stream is
%     open, the car cannot send photos to MATLAB.
%   - Each run saves a vslamRun_<date>_<time> folder: the photos (frames),
%     debug.txt (settings, car info, messages, errors) and steps.csv (one line
%     per step: photo check, battery, corners, SLAM status). If something goes
%     wrong, look there first.

arguments
    source (1,1) string = "robot"
    options.Host (1,1) string = "192.168.4.1"
    options.AutoRun (1,1) logical = false
    options.ShowUI (1,1) logical = true
    options.Config (1,1) struct = struct()
end
results = [];
cfg = defaultConfig();
for name = string(fieldnames(options.Config))'
    assert(isfield(cfg, name), "Unknown setting '%s' (see defaultConfig).", name);
    cfg.(name) = options.Config.(name);
end
isReplay = ~any(source == ["robot", "sim"]);

% --- 1. Run folder with the debug log, then connect and check the sensors ---
runDir = "";
runTimer = tic;
if ~isReplay
    runDir = "vslamRun_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    mkdir(fullfile(runDir, "frames"));
    debugNote(runDir, "Run started: " + source + ", host " + options.Host + ", MATLAB " + version);
    debugNote(runDir, "Settings: " + jsonencode(cfg));
    writelines("step,time_s,phase,action,range_cm,battery_V,photo_tries,photo_diff,photo_s," + ...
        "corners,slam_s,status,key_frames,map_started,lost_count,turned_deg", ...
        fullfile(runDir, "steps.csv"));
end
robot = [];
startMessages = strings(0, 1);
try
    if isReplay
        replay = loadRun(source);
        camK = replay.intrinsics;
    else
        if source == "sim"
            robot = ElegooSimCar();
        else
            % Find ElegooRobotCar.m. In the student kit, it is in this folder. In the
            % course repository, it is in the basic_wireless_control folder.
            if exist("ElegooRobotCar", "file") ~= 2
                addpath(fullfile(fileparts(mfilename("fullpath")), "..", "basic_wireless_control"));
            end
            robot = ElegooRobotCar(options.Host);
        end
        robot.stop();
        robot.setWatchdog(1000);           % motors stop if MATLAB stops talking
        robot.pan(90);                     % camera and sonar look straight ahead
        pause(0.5);
        debugNote(runDir, "Car: " + carInfo(robot));
        [camK, startMessages] = startupCheck(robot, cfg);
    end
catch ME
    debugNote(runDir, "Could not start: " + getReport(ME, "extended", "hyperlinks", "off"));
    safeClose(robot);
    if options.AutoRun, rethrow(ME); end
    uialert(uifigure, "Could not start (" + source + "): " + ME.message, "Connection Error");
    return
end
cleanupObj = onCleanup(@() safeClose(robot));

% --- 2. Visual SLAM object (photos are undistorted and resized to 640x480) ---
slamK = scaleIntrinsics(cameraIntrinsics(camK.FocalLength, camK.PrincipalPoint, camK.ImageSize), ...
    cfg.workSize);
vslam = monovslam(slamK, MaxNumPoints = cfg.maxPoints, TrackFeatureRange = cfg.trackFeatureRange, ...
    LoopClosureThreshold = cfg.loopClosureThreshold);

% --- 3. Run state (shared with the nested functions below) ---
maxSteps = cfg.maxSteps;
if isReplay, maxSteps = height(replay.runLog); end
runLog = table('Size', [maxSteps 3], 'VariableTypes', ["string", "double", "string"], ...
    'VariableNames', ["action", "rangeCm", "status"]);
runLog.keyFrames = zeros(maxSteps, 1);      % size of the SLAM map after each photo
runLog.trueCamPose = nan(maxSteps, 3);      % simulator only: ground truth
k = 0;                                      % photos taken
rangeCm = NaN;                              % latest ultrasonic reading
mapStarted = false;
photosBeforeMap = 0;                        % photos added before the last SLAM restart
lostCount = 0;                              % photos in a row with tracking lost
heading = NaN;                              % camera heading from SLAM (deg)
turned = 0;                                 % how far the circle has turned (deg)
trail = strings(0, 1);                      % steps since the map started (to retrace)
retraced = 0;
relocalizations = 0;
skipped = 0;                                % circle steps driven while lost
scaleSteps = 0;
backedUp = false;
phase = "start";                            % start -> circle -> scale
isRunning = false;
isBusy = false;
if ~isReplay
    rangeCm = readRange(robot);
end

% --- 4. Dashboard (same layout style as the other demos) ---
fig = uifigure('Name', "Elegoo Robot Car - Visual SLAM (" + source + ")", ...
    'Position', [40, 40, 1300, 820], 'Visible', options.ShowUI);
mainGrid = uigridlayout(fig, [1, 2]);
mainGrid.ColumnWidth = {'3x', '2x'};
leftGrid = uigridlayout(mainGrid, [2, 1]);
leftGrid.RowHeight = {'2.4x', '1x'};
rightGrid = uigridlayout(mainGrid, [2, 1]);
rightGrid.RowHeight = {'2x', '1.4x'};

viewGrid = uigridlayout(uipanel(leftGrid, 'Title', 'Camera Features & Live Map'), [1, 2]);
axCam = uiaxes(viewGrid);
hImg = imshow(zeros([cfg.workSize 3], "uint8"), 'Parent', axCam);
hold(axCam, 'on');
hOrb = plot(axCam, NaN, NaN, 'g+', 'MarkerSize', 4);
title(axCam, 'Camera + ORB corners');
axTop = uiaxes(viewGrid);
hold(axTop, 'on'); grid(axTop, 'on'); axis(axTop, 'equal');
hTopPts = plot(axTop, NaN, NaN, '.', 'Color', [0.5 0.5 0.5], 'MarkerSize', 3);
hTopPath = plot(axTop, NaN, NaN, 'b-o', 'MarkerSize', 3, 'LineWidth', 1.5);
xlabel(axTop, 'forward (SLAM units)'); ylabel(axTop, 'left (SLAM units)');
title(axTop, 'Top view: key frames and map points');

controlGrid = uigridlayout(uipanel(leftGrid, 'Title', 'Drive'), [3, 4]);
uilabel(controlGrid, 'Text', 'Forward PWM:');
speedInput = uieditfield(controlGrid, 'numeric', 'Value', cfg.forwardSpeed, 'Limits', [60, 255]);
uilabel(controlGrid, 'Text', 'Circle inner PWM (lower = tighter):');
circleInput = uieditfield(controlGrid, 'numeric', 'Value', cfg.circleInner, ...
    'Limits', [60, cfg.circleOuter - 5]);
startBtn = uibutton(controlGrid, 'Text', 'Start Auto Run', 'BackgroundColor', [0.4 0.8 0.4], ...
    'ButtonPushedFcn', @(~,~) runAuto());
uibutton(controlGrid, 'Text', '▲ Step', 'ButtonPushedFcn', @(~,~) manualStep("forward"));
uibutton(controlGrid, 'Text', '◄ Arc', 'ButtonPushedFcn', @(~,~) manualStep("arcLeft"));
uibutton(controlGrid, 'Text', 'Arc ►', 'ButtonPushedFcn', @(~,~) manualStep("arcRight"));
uibutton(controlGrid, 'Text', 'STOP', 'BackgroundColor', [1 0.6 0.6], 'ButtonPushedFcn', @(~,~) stopRun());
uibutton(controlGrid, 'Text', '▼ Step', 'ButtonPushedFcn', @(~,~) manualStep("backward"));
finishBtn = uibutton(controlGrid, 'Text', 'Finish & Build Map', 'BackgroundColor', [0.6 0.8 1], ...
    'ButtonPushedFcn', @(~,~) finishRun());
finishBtn.Layout.Column = [3, 4];

statusGrid = uigridlayout(uipanel(rightGrid, 'Title', 'Ultrasonic Range & Status'), [2, 1]);
axRange = uiaxes(statusGrid);
hRange = plot(axRange, NaN, NaN, 'b.-');
grid(axRange, 'on'); xlabel(axRange, 'photo'); ylabel(axRange, 'range (cm)');
logArea = uitextarea(statusGrid, 'Editable', 'off', 'Value', "Ready.");

infoGrid = uigridlayout(uipanel(rightGrid, 'Title', 'How it works'), [2, 1]);
infoGrid.RowHeight = {'1x', 40};
uitextarea(infoGrid, 'Editable', 'off', 'Value', [ ...
    "Each step: motor pulse -> stop -> photos -> sonar -> SLAM."; ...
    "1. Start: 4 steps forward / 4 back until SLAM has a map (parallax)."; ...
    "2. Circle ~1.25 turns: SLAM measures the turn; back at the start it"; ...
    "   recognizes the place and closes the loop."; ...
    "3. Scale: back away from the nearest obstacle, drive straight at it;"; ...
    "   sonar cm vs SLAM units."; ...
    "Lost: retrace the last steps until SLAM finds itself again; lost again on"; ...
    "   the circle: drive on until it recognizes a mapped place."]);
uibutton(infoGrid, 'Text', 'Emergency Stop Robot', 'BackgroundColor', [1 0.3 0.3], ...
    'ButtonPushedFcn', @(~,~) stopRun());

fig.CloseRequestFcn = @(~,~) closeUI();
for startMsg = startMessages'
    logMsg(startMsg);
end
if isReplay
    startBtn.Text = 'Replay Run';
    logMsg("Replay of " + source + ": " + maxSteps + " photos.");
end
drawnow;

if options.AutoRun
    runAuto();
    results = finishRun();
    if ~options.ShowUI
        closeUI();
    end
end

    % =================================================================
    % Nested functions: they share the run state above
    % =================================================================
    function runAuto()
        if isRunning || isBusy, return; end
        isRunning = true;
        logMsg("Auto run started.");
        while isRunning && k < maxSteps && isvalid(fig)
            if isReplay
                action = replay.runLog.action(k + 1);
            else
                action = nextAction();
            end
            if action == "done" || ~takeStep(action)
                break
            end
        end
        safeStop(robot);
        isRunning = false;
        logMsg("Auto run ended after " + k + " photos. Click 'Finish & Build Map'.");
    end

    function manualStep(action)
        if isRunning || isBusy || isReplay || k >= maxSteps, return; end
        if action ~= "backward" && tooClose()
            logMsg("Blocked: obstacle at " + rangeCm + " cm.");
            return
        end
        takeStep(action);
        if lostCount == cfg.lostPhotos
            logMsg("Tracking lost: step back toward where it was tracking.");
        end
    end

    function action = nextAction()
        % The auto run as a short list of rules, checked in order.

        % Lost? First retrace the last steps (undo them) until SLAM finds itself.
        % Lost again on the circle, or retracing did not help: that spot has too
        % little texture, so drive on past it; SLAM recognizes the mapped part
        % of the room later.
        if mapStarted && lostCount >= cfg.lostPhotos
            driveOn = phase == "circle" && (relocalizations >= 1 || retraced >= cfg.maxRetrace);
            if driveOn && skipped < cfg.maxSkipSteps
                if skipped == 0
                    logMsg("Lost again here: driving on around the circle until SLAM recognizes a mapped place.");
                end
                skipped = skipped + 1;
                action = "circle";
            elseif ~driveOn && ~isempty(trail) && retraced < cfg.maxRetrace
                action = "undo:" + trail(end);
                trail(end) = [];
                retraced = retraced + 1;
            else
                logMsg("Could not find the map again. Ending the run.");
                action = "done";
            end
            return
        end
        if retraced > 0 || skipped > 0
            logMsg(sprintf("Found the map again (relocalized) after %d steps.", retraced + skipped));
            retraced = 0;
            skipped = 0;
            relocalizations = relocalizations + 1;
            if relocalizations > cfg.maxRelocalizations
                % It keeps losing track at the same place: give up there.
                logMsg("Tracking keeps failing here (too little texture). Ending the run.");
                action = "done";
                return
            end
        end

        if phase == "start" && mapStarted
            phase = "circle";
            logMsg("Map started. Driving a circle.");
        end
        if phase == "circle" && turned >= 360 * cfg.circleTurns
            phase = "scale";
            logMsg("Circle done. Measuring the scale: back away from the nearest obstacle, then drive at it.");
        end

        switch phase
            case "start"
                % 4 steps forward, 4 back, ... until SLAM has a map. After 24
                % photos without a map, turn a little to face a new scene.
                cycle = mod(k, 24);
                if cycle >= 24 - cfg.newViewSteps
                    action = "arcLeft";
                elseif mod(cycle, 2 * cfg.startSweep) < cfg.startSweep && ~tooClose()
                    action = "forward";
                else
                    action = "backward";
                end
            case "circle"
                if tooClose(cfg.circleClearance)
                    action = "backward";      % something in the way: back off, keep circling
                else
                    action = "circle";
                end
            case "scale"
                % Back away to get a run-up (the path behind is known to be free),
                % then drive straight in to 50 cm. Both straight runs give the scale.
                if ~backedUp && rangeCm < cfg.scaleStartCm && scaleSteps < cfg.maxScaleSteps
                    scaleSteps = scaleSteps + 1;
                    action = "backward";
                elseif rangeCm > cfg.scaleStopCm && scaleSteps < 2 * cfg.maxScaleSteps
                    backedUp = true;
                    scaleSteps = scaleSteps + 1;
                    action = "forward";
                else
                    action = "done";
                end
        end
    end

    function tf = tooClose(limitCm)
        % A missing reading counts as "too close", to be safe.
        if nargin < 1, limitCm = cfg.stopDistanceCm; end
        tf = isnan(rangeCm) || rangeCm < limitCm;
    end

    function ok = takeStep(action)
        % One stop-and-go step: move, stop, photo, sonar, SLAM. False on error.
        ok = false;
        isBusy = true;
        try
            if isReplay
                frame = imread(replay.frameFiles(k + 1));
                newRange = replay.runLog.rangeCm(k + 1);
                photo = struct("tries", 1, "diff", NaN, "sec", 0);
                battery = NaN;
            else
                move(robot, action, speedInput.Value, circleInput.Value, cfg);
                [frame, photo] = grabFrame(robot, cfg);
                newRange = readRange(robot);
                battery = readBatteryVolts(robot);
            end
        catch ME
            safeStop(robot);
            isRunning = false;
            isBusy = false;
            debugNote(runDir, "Step failed: " + getReport(ME, "extended", "hyperlinks", "off"));
            logMsg("Step failed (" + ME.message + "). Car stopped; check Wi-Fi and battery.");
            return
        end
        if photo.diff >= cfg.photoAgree
            logMsg("Photos keep coming out striped; using the last one. Charge the battery or restart the car.");
        end
        k = k + 1;
        rangeCm = newRange;
        runLog.action(k) = action;
        runLog.rangeCm(k) = rangeCm;
        if isReplay
            runLog.trueCamPose(k, :) = replay.runLog.trueCamPose(k, :);
        else
            imwrite(frame, fullfile(runDir, "frames", sprintf("frame_%04d.jpg", k)), "Quality", 95);
            if isa(robot, "ElegooSimCar"), runLog.trueCamPose(k, :) = robot.cameraPose(); end
        end
        if mapStarted && ~startsWith(action, "undo:")
            trail(end+1) = action;                  % remember the step for retracing
        end

        % ---- Visual SLAM ----
        img = imresize(undistort(frame, camK), cfg.workSize);
        % The ESP32 photos are pale and soft. Stronger local contrast (CLAHE)
        % gives about 3x more corners; monovslam needs cfg.maxPoints corners in
        % a photo before it can start a map.
        slamImg = adapthisteq(im2gray(img));
        corners = detectORBFeatures(slamImg);
        waitTimer = tic;                        % wait until SLAM finishes this photo
        addFrame(vslam, slamImg);
        while ~isDone(vslam) && toc(waitTimer) < 20
            pause(0.02);
        end
        slamSec = toc(waitTimer);

        % No map after one sweep (4 forward, 4 back)? SLAM keeps comparing new
        % photos with its first photo, and they now share too few corners: start
        % over from here.
        if ~mapStarted
            if numel(poses(vslam)) > 1
                mapStarted = true;
            elseif k - photosBeforeMap >= 2 * cfg.startSweep
                % Start over with a new SLAM object. (Do not use reset(vslam):
                % after it, isDone never turns true and each photo waits 20 s.)
                delete(vslam);
                vslam = monovslam(slamK, MaxNumPoints = cfg.maxPoints, ...
                    TrackFeatureRange = cfg.trackFeatureRange, ...
                    LoopClosureThreshold = cfg.loopClosureThreshold);
                photosBeforeMap = k;
            end
        end
        status = "Starting map";
        if mapStarted
            status = string(checkStatus(vslam));
            runLog.keyFrames(k) = numel(poses(vslam));
        end
        runLog.status(k) = status;
        lostCount = (lostCount + 1) * (status == "TrackingLost");

        % Heading from the newest key frame; add up the turn of the circle.
        camPoses = [];
        if mapStarted && hasNewKeyFrame(vslam)
            camPoses = poses(vslam);
            newHeading = cameraHeading(camPoses(end));
            if phase == "circle" && ~isnan(heading)
                turned = turned + mod(newHeading - heading + 180, 360) - 180;
            end
            heading = newHeading;
        end

        % Debug log: one line per step in steps.csv.
        if runDir ~= ""
            writelines(sprintf("%d,%.1f,%s,%s,%g,%.2f,%d,%.3f,%.2f,%d,%.2f,%s,%d,%d,%d,%.0f", ...
                k, toc(runTimer), phase, action, rangeCm, battery, photo.tries, photo.diff, ...
                photo.sec, corners.Count, slamSec, status, runLog.keyFrames(k), mapStarted, ...
                lostCount, turned), fullfile(runDir, "steps.csv"), WriteMode = "append");
            if mod(k, 10) == 0 && isa(robot, "ElegooRobotCar")
                debugNote(runDir, "Step " + k + ": " + carInfo(robot));
            end
        end

        updateView(img, action, status, camPoses, corners);
        isBusy = false;
        ok = true;
    end

    function updateView(img, action, status, camPoses, corners)
        if ~isvalid(fig), return; end
        hImg.CData = img;
        pts = selectStrongest(corners, 300);
        set(hOrb, 'XData', pts.Location(:, 1), 'YData', pts.Location(:, 2));
        title(axCam, sprintf('Photo %d | %s | %s', k, action, status));
        set(hRange, 'XData', 1:k, 'YData', runLog.rangeCm(1:k));
        if ~isempty(camPoses)
            P = vertcat(camPoses.Translation);
            X = mapPoints(vslam);
            % Camera frame: x right, y down, z forward -> top view: forward = z, left = -x.
            set(hTopPts, 'XData', X(:, 3), 'YData', -X(:, 1));
            set(hTopPath, 'XData', P(:, 3), 'YData', -P(:, 1));
            title(axTop, sprintf('Top view: %d key frames, %d map points', size(P, 1), size(X, 1)));
        end
        drawnow limitrate;
    end

    function res = finishRun()
        res = [];
        if isBusy
            logMsg("Still taking a step; click again in a moment.");
            return
        end
        isRunning = false;
        safeStop(robot);
        logMsg("Building the map from " + k + " photos...");
        waitTimer = tic;
        while ~isDone(vslam) && toc(waitTimer) < 60
            pause(0.2);
        end
        log = runLog(1:k, :);
        if runDir ~= ""
            intrinsics = camK;                       % saved first, so a replay is always possible
            save(fullfile(runDir, "vslamRun.mat"), "log", "intrinsics", "cfg");
        end
        try
            [res, msgs, resultFig] = buildMap(vslam, photosBeforeMap, log, slamK, cfg, options.ShowUI);
        catch ME
            debugNote(runDir, "Building the map failed: " + getReport(ME, "extended", "hyperlinks", "off"));
            rethrow(ME);
        end
        for mapMsg = msgs'
            logMsg(mapMsg);
        end
        if runDir ~= "" && ~isempty(resultFig)
            save(fullfile(runDir, "vslamRun.mat"), "res", "-append");
            exportgraphics(resultFig, fullfile(runDir, "slamResult.png"));
            saveCameraVideo(runDir, k);
            logMsg("Saved the run, map picture and camera video in " + runDir);
        end
        if ~options.ShowUI && ~isempty(resultFig)
            close(resultFig);
        end
        assignin('base', 'vslamResults', res);
    end

    function stopRun()
        isRunning = false;
        safeStop(robot);
        logMsg("Stopped.");
    end

    function logMsg(msg)
        fprintf("%s\n", msg);
        debugNote(runDir, msg);
        if isvalid(fig)
            logArea.Value = ["[" + string(datetime("now", "Format", "HH:mm:ss")) + "] " + msg; ...
                string(logArea.Value)];
            drawnow limitrate;
        end
    end

    function closeUI()
        isRunning = false;
        safeStop(robot);
        delete(fig);
    end
end

% =====================================================================
% Settings (change them here, or with Config=struct(...))
% =====================================================================
function cfg = defaultConfig()
cfg.intrinsicsFile = fullfile(fileparts(mfilename("fullpath")), "elegooCameraIntrinsics.mat");   % from calibrateElegooCamera
cfg.guessFovDeg = 60;          % lens guess if there is no calibration file
cfg.workSize = [480 640];      % photos are resized to this for SLAM
cfg.cameraHeightM = 0.12;      % lens height above the floor (measure yours)

% Stop-and-go steps: [left PWM, right PWM, seconds]. Undo = same with -PWM.
cfg.forwardSpeed = 100;        % forward/backward PWM (dashboard field)
cfg.forwardSec = 0.30;         % ~5-8 cm per step
cfg.arc = [100 160 0.10];      % manual arcs: ~6 deg per step
cfg.circleInner = 135;         % circle: inner wheels (dashboard field)
cfg.circleOuter = 160;         %         outer wheels
cfg.circleSec = 0.15;          % ~4 deg per step. In tests 6 deg per step lost
                               % tracking; TT motors are unreliable below ~0.1 s.
cfg.settleSec = 0.15;          % let the car stop rocking before the photo
cfg.photoTries = 6;            % photos per step until two in a row agree
cfg.photoAgree = 0.12;         % two good photos differ by < 0.05 (simulator noise: < 0.11);
                               % a photo with stripes differs by > 0.15

% Auto run
cfg.startSweep = 4;            % map start: 4 steps forward, 4 back (one try = 8 photos)
cfg.newViewSteps = 4;          % small left turn every 24 photos without a map
cfg.circleTurns = 1.25;        % one circle plus a quarter to revisit the start
cfg.circleClearance = 45;      % back off if something is closer (cm)
cfg.stopDistanceCm = 20;       % never drive forward closer than this
cfg.scaleStartCm = 120;        % scale drive: back up until the obstacle is this far,
cfg.scaleStopCm = 50;          % then drive in to this distance
cfg.maxScaleSteps = 20;
cfg.lostPhotos = 2;            % lost this many photos in a row: retrace
cfg.maxRetrace = 8;            % steps to retrace before giving up
cfg.maxSkipSteps = 25;         % circle steps (~90 deg) to drive on while lost
cfg.maxRelocalizations = 6;    % lost again and again at one place: end the run
cfg.maxSteps = 500;

% monovslam and the map
cfg.maxPoints = 1000;          % ORB corners per photo (2000+ is too slow). monovslam
                               % starts a map only on a photo with this many corners.
cfg.trackFeatureRange = [30 120];   % lost below 30 tracked corners; new key frame
                                    % below 120 (more key frames while turning)
cfg.loopClosureThreshold = 40;
cfg.scaleRangeCm = [8 140];    % trust sonar readings in this range (cap 150)
cfg.obstacleHeightM = [0.04 0.80];  % map points this high above the floor are obstacles
cfg.cellsPerMeter = 20;        % occupancy grid: 5 cm cells
end

% =====================================================================
% Car I/O
% =====================================================================
function move(robot, action, speed, circleInner, cfg)
% Stop-and-go step. "undo:<step>" runs <step> with both wheels reversed.
undo = startsWith(action, "undo:");
name = erase(action, "undo:");
v = round(speed);
switch name
    case "forward",  w = [v v cfg.forwardSec];
    case "backward", w = [-v -v cfg.forwardSec];
    case "arcLeft",  w = cfg.arc;
    case "arcRight", w = cfg.arc([2 1 3]);
    case "circle",   w = [round(circleInner) cfg.circleOuter cfg.circleSec];   % gentle left curve
    otherwise,       return
end
if undo, w(1:2) = -w(1:2); end
robot.drive(w(1), w(2));
pause(w(3));
robot.stop();
pause(cfg.settleSec);
end

function [frame, photo] = grabFrame(robot, cfg)
% The ESP32 camera sometimes turns the lower part of a photo into colored
% stripes, and MATLAB does not notice. The car stands still here, so two good
% photos in a row look the same. Take photos until two in a row agree.
timer = tic;
frame = capturePhoto(robot);
difference = NaN;
tries = 1;
while tries < cfg.photoTries
    previous = frame;
    frame = capturePhoto(robot);
    tries = tries + 1;
    difference = photoDifference(previous, frame);
    if difference < cfg.photoAgree
        break
    end
end
photo = struct("tries", tries, "diff", difference, "sec", toc(timer));
end

function frame = capturePhoto(robot)
try
    frame = robot.capture();
catch
    pause(0.3);                     % one retry if Wi-Fi hiccuped
    frame = robot.capture();
end
end

function d = photoDifference(a, b)
% How different two photos are (0 = the same), in the worst band of 16 rows.
% Only color counts, so a change in brightness (auto exposure) does not.
a = im2single(a);
b = im2single(b);
ca = a - mean(a, 3);
cb = b - mean(b, 3);
dd = abs((ca - mean(ca, [1 2])) - (cb - mean(cb, [1 2])));
rowDiff = mean(sum(dd, 3), 2);                  % one value per photo row
bands = floor(numel(rowDiff) / 16);
d = max(mean(reshape(rowDiff(1:16 * bands), 16, bands), 1));
end

function v = readBatteryVolts(robot)
% Battery voltage, NaN if there is no reply (the simulator has no battery).
v = NaN;
try
    if ismethod(robot, "readBattery")
        reply = robot.readBattery();
        if isfield(reply, "battery"), v = double(reply.battery); end
    end
catch
end
end

function text = carInfo(robot)
% Car settings, battery and ESP32 state, for debug.txt.
if isa(robot, "ElegooSimCar")
    text = "simulator";
    return
end
text = sprintf("PhotoSize=%d PhotoVFlip=%d PhotoMirror=%d SwapSides=%d, battery %.2f V", ...
    robot.PhotoSize, robot.PhotoVFlip, robot.PhotoMirror, robot.SwapSides, readBatteryVolts(robot));
try
    text = text + ", ESP32 info " + jsonencode(robot.info());
catch ME
    text = text + ", ESP32 info failed: " + ME.message;
end
try
    text = text + ", ESP32 status " + jsonencode(robot.status());
catch ME
    text = text + ", ESP32 status failed: " + ME.message;
end
end

function debugNote(runDir, text)
% Add a line to <runDir>/debug.txt, to find problems after a run.
if runDir == "", return; end
writelines("[" + string(datetime("now", "Format", "HH:mm:ss.SSS")) + "] " + text, ...
    fullfile(runDir, "debug.txt"), WriteMode = "append");
end

function r = readRange(robot)
% Whole cm, NaN if no reply. The firmware sends 0 for no echo (nothing in
% range): use 150 cm, the same value as the simulator.
r = NaN;
try
    reply = robot.readDistance();
    if isfield(reply, "distance"), r = double(reply.distance); end
catch
end
if r == 0 || r > 150, r = 150; end
end

function [K, msgs] = startupCheck(robot, cfg)
% Camera model, image size and the ultrasonic sensor, explained in the log.
if isa(robot, "ElegooSimCar")
    K = robot.cameraIntrinsics();
    calibMsg = "Simulated camera.";
elseif isfile(cfg.intrinsicsFile)
    saved = load(cfg.intrinsicsFile);
    K = saved.intrinsics;
    calibMsg = "Using camera calibration " + cfg.intrinsicsFile + ".";
else
    K = [];
    calibMsg = "WARNING: no camera calibration; guessing a 60-degree lens. Run calibrateElegooCamera.";
end
[frame, photo] = grabFrame(robot, cfg);
sz = size(frame, 1:2);
if isempty(K)
    f = (sz(2) / 2) / tand(cfg.guessFovDeg / 2);
    K = cameraIntrinsics([f f], sz([2 1]) / 2 + 0.5, sz);
end
assert(abs(K.ImageSize(2) / K.ImageSize(1) - sz(2) / sz(1)) < 0.01, ...
    "The calibration was made at another image shape. Recalibrate.");
K = scaleIntrinsics(K, sz);
r = readRange(robot);
msgs = [calibMsg; sprintf("Camera %dx%d, ultrasonic %g cm.", sz(2), sz(1), r)];
if photo.diff >= cfg.photoAgree
    msgs(end+1) = "WARNING: the photos are striped. Charge the battery and restart the car.";
end
if sz(2) < 400
    msgs(end+1) = "WARNING: photos are smaller than 400x296; maps will start poorly.";
end
assert(~isnan(r), "The ultrasonic sensor does not answer.");
end

function safeStop(robot)
try
    if ~isempty(robot), robot.stop(); end
catch
end
end

function safeClose(robot)
try
    if ~isempty(robot), robot.stop(); robot.close(); end
catch
end
end

% =====================================================================
% Camera geometry
% =====================================================================
function K = scaleIntrinsics(K, imageSize)
% Same camera, different image size.
if isequal(K.ImageSize, imageSize), return; end
s = imageSize([2 1]) ./ K.ImageSize([2 1]);
K = cameraIntrinsics(K.FocalLength .* s, (K.PrincipalPoint - 0.5) .* s + 0.5, imageSize, ...
    RadialDistortion = K.RadialDistortion, TangentialDistortion = K.TangentialDistortion);
end

function img = undistort(img, K)
if any(K.RadialDistortion) || any(K.TangentialDistortion)
    img = undistortImage(img, K, OutputView = "same");
end
end

function deg = cameraHeading(pose)
% Heading in the top view (forward = camera z, left = -camera x).
fwd = pose.R(:, 3);
deg = atan2d(-fwd(1), fwd(3));
end

% =====================================================================
% Building the map
% =====================================================================
function [res, msgs, fh] = buildMap(vslam, photosBeforeMap, log, slamK, cfg, showUI)
fh = [];
[camPoses, kfPhotos] = poses(vslam);
kfPhotos = kfPhotos + photosBeforeMap;          % key frame -> photo number
xyz = mapPoints(vslam);
res = struct("keyFramePhotos", kfPhotos, "cameraPoses", camPoses, "mapPoints", xyz, ...
    "cmPerUnit", NaN, "pathM", [], "obstaclesM", [], "occupancy", [], "pathErrorCm", NaN, "log", log);
if numel(camPoses) < 3
    msgs = "No map: SLAM never started. Add texture, calibrate the camera, or drive more.";
    return
end

% 1. Scale: on straight drives, sonar cm vs SLAM distance (a line fit).
C = vertcat(camPoses.Translation);
[res.cmPerUnit, nRuns] = sonarScale(C, kfPhotos, log, cfg);
if isnan(res.cmPerUnit)
    msgs = "No straight drive inside sonar range: the map stays in SLAM units.";
    res.cmPerUnit = 100;
else
    msgs = sprintf("Scale: %.1f cm per SLAM unit (from %d straight drive(s)).", res.cmPerUnit, nRuns);
end
msgs = [sprintf("Map: %d key frames, %d map points.", numel(camPoses), size(xyz, 1)); msgs];

% 2. Floor frame: the camera path lies in a plane parallel to the floor.
[toFloor, basis] = floorFrame(C);
m = res.cmPerUnit / 100;
pathM = toFloor(C) * m;
ptsM = toFloor(xyz) * m;
height = ptsM(:, 3) + cfg.cameraHeightM;         % height above the floor
isObstacle = height > cfg.obstacleHeightM(1) & height < cfg.obstacleHeightM(2);
res.pathM = pathM(:, 1:2);
res.obstaclesM = ptsM(isObstacle, 1:2);

% 3. Occupancy grid from the camera points and the sonar beams.
headings = zeros(numel(camPoses), 1);
for i = 1:numel(camPoses)
    d = camPoses(i).R(:, 3)' * basis;            % viewing direction in the floor frame
    headings(i) = atan2d(d(2), d(1));
end
hfov = 2 * atand(slamK.ImageSize(2) / 2 / slamK.FocalLength(1));
res.occupancy = occupancyFromSensors(res.pathM, headings, log.rangeCm(kfPhotos), res.obstaclesM, hfov, cfg);

% 4. Simulator only: compare with the true path.
truth = truePath(log, kfPhotos);
if ~isempty(truth)
    [s, R, t] = fitSimilarity2D(res.pathM, truth);
    res.pathErrorCm = 100 * sqrt(mean(sum(((s * R * res.pathM' + t)' - truth).^2, 2)));
    msgs(end+1) = sprintf("Simulator check: path error %.1f cm, scale off by %+.0f%%.", ...
        res.pathErrorCm, 100 * (1 / s - 1));
end
fh = showMap(res, ptsM, height, truth, cfg, showUI);
end

function [cmPerUnit, nRuns] = sonarScale(C, kfPhotos, log, cfg)
% While the car drives straight toward (or away from) something, the range
% changes 1 cm per cm driven. On each straight drive, fit range vs SLAM
% distance: the slope is cm per SLAM unit.
slopes = [];
weights = [];
act = log.action;
straight = act == "forward" | act == "backward";
k = 1;
while k <= numel(act)
    if ~straight(k), k = k + 1; continue; end
    e = k;
    while e < numel(act) && act(e + 1) == act(k), e = e + 1; end
    kf = find(kfPhotos >= k - 1 & kfPhotos <= e);           % key frames on this drive
    r = log.rangeCm(kfPhotos(kf));
    use = r >= cfg.scaleRangeCm(1) & r <= cfg.scaleRangeCm(2);
    kf = kf(use);
    r = r(use);
    if numel(kf) >= 2 && max(r) - min(r) > 10
        d = vecnorm(C(kf, :) - C(kf(1), :), 2, 2);
        p = polyfit(d, r, 1);
        slopes(end+1) = abs(p(1)); %#ok<AGROW>
        weights(end+1) = max(r) - min(r); %#ok<AGROW>
    end
    k = e + 1;
end
nRuns = numel(slopes);
cmPerUnit = NaN;
if nRuns > 0, cmPerUnit = sum(slopes .* weights) / sum(weights); end
end

function [toFloor, basis] = floorFrame(C)
% Plane through the camera path: its normal is "up" (camera y points down).
[~, S, V] = svd(C - mean(C, 1), 0);
up = V(:, 3)';
if size(S, 1) < 2 || S(2, 2) < 0.1 * S(1, 1), up = [0 -1 0]; end   % path is a line
if dot(up, [0 -1 0]) < 0, up = -up; end
fwd = [0 0 1] - dot([0 0 1], up) * up;
fwd = fwd / norm(fwd);
basis = [fwd' cross(up, fwd)' up'];                         % x forward, y left, z up
toFloor = @(P) (P - C(1, :)) * basis;
end

function map = occupancyFromSensors(path, headings, rangesCm, obstacles, hfov, cfg)
% Every key frame adds: a 15-degree sonar cone (free up to the echo, occupied
% at the echo; a "far" reading only means free), and camera lines of sight
% (free along the line, occupied at each obstacle point within 2.5 m).
sonarEnds = path + min(rangesCm / 100, 1.45) .* [cosd(headings) sind(headings)];
[lo, hi] = mapBounds(path, [obstacles; sonarEnds(~isnan(rangesCm), :)]);
obstacles = obstacles(all(obstacles >= lo & obstacles <= hi, 2), :);
map = occupancyMap(hi(1) - lo(1), hi(2) - lo(2), cfg.cellsPerMeter);
map.GridLocationInWorld = lo;
beam = deg2rad(-7.5:3.75:7.5);
for i = 1:size(path, 1)
    if ~isnan(rangesCm(i))
        insertRay(map, [path(i, :) deg2rad(headings(i))], rangesCm(i) / 100 * ones(size(beam)), beam, 1.45);
    end
    d = obstacles - path(i, :);
    seen = abs(mod(atan2d(d(:, 2), d(:, 1)) - headings(i) + 180, 360) - 180) < hfov / 2 & ...
        vecnorm(d, 2, 2) < 2.5;
    pts = obstacles(seen, :);
    pts = pts(round(linspace(1, size(pts, 1), min(60, size(pts, 1)))), :);
    if ~isempty(pts), insertRay(map, path(i, :), pts); end
end
end

function [lo, hi] = mapBounds(path, pts)
% Limits that hold the path and the bulk of the points (ignore a few strays).
if size(pts, 1) >= 20
    pts = pts(all(pts >= prctile(pts, 2) & pts <= prctile(pts, 98), 2), :);
end
xy = [path; pts];
lo = floor(min(xy, [], 1) * 10) / 10 - 0.3;
hi = ceil(max(xy, [], 1) * 10) / 10 + 0.3;
end

function truth = truePath(log, kfPhotos)
% Simulator only: true camera positions in the first key frame's frame.
truth = [];
pose = log.trueCamPose(kfPhotos, :);
if any(isnan(pose(:))), return; end
c = cos(pose(1, 3)); s = sin(pose(1, 3));
truth = (pose(:, 1:2) - pose(1, 1:2)) * [c -s; s c];
end

function [s, R, t] = fitSimilarity2D(A, B)
% Best scale s, rotation R and shift t with B ~ s*R*A + t (rows are points).
muA = mean(A, 1);
muB = mean(B, 1);
[U, S, V] = svd((B - muB)' * (A - muA));
D = diag([1 sign(det(U * V'))]);
R = U * D * V';
s = trace(S * D) / sum((A - muA).^2, "all");
t = muB' - s * R * muA';
end

function fh = showMap(res, ptsM, height, truth, cfg, showUI)
fh = figure("Name", "Elegoo Visual SLAM map", "Color", "w", "Position", [80 80 1250 540], ...
    "Visible", showUI);
tl = tiledlayout(fh, 1, 2, "TileSpacing", "compact", "Padding", "compact");

ax = nexttile(tl);
hold(ax, "on"); grid(ax, "on"); axis(ax, "equal");
onFloor = abs(height) < cfg.obstacleHeightM(1);
h = plot(ax, [ptsM(onFloor, 1); NaN], [ptsM(onFloor, 2); NaN], ".", "Color", [0.75 0.75 0.75], ...
    "MarkerSize", 3, "DisplayName", "floor points");
h(2) = plot(ax, [res.obstaclesM(:, 1); NaN], [res.obstaclesM(:, 2); NaN], "r.", "MarkerSize", 4, ...
    "DisplayName", "obstacle points");
h(3) = plot(ax, res.pathM(:, 1), res.pathM(:, 2), "b-", "LineWidth", 2, "DisplayName", "SLAM path");
h(4) = plot(ax, res.pathM(1, 1), res.pathM(1, 2), "go", "MarkerFaceColor", "g", "DisplayName", "start");
if ~isempty(truth)
    [s, R, t] = fitSimilarity2D(truth, res.pathM);
    truthInMap = (s * R * truth' + t)';
    h(5) = plot(ax, truthInMap(:, 1), truthInMap(:, 2), "k--", "DisplayName", "true path (sim)");
end
legend(ax, h, "Location", "bestoutside");
[lo, hi] = mapBounds(res.pathM, res.obstaclesM);
axis(ax, [lo(1) hi(1) lo(2) hi(2)]);
xlabel(ax, "forward (m)"); ylabel(ax, "left (m)");
title(ax, sprintf("Map points and path (%.1f cm per SLAM unit)", res.cmPerUnit));

ax2 = nexttile(tl);
show(res.occupancy, "Parent", ax2);
hold(ax2, "on");
plot(ax2, res.pathM(:, 1), res.pathM(:, 2), "b-", "LineWidth", 2);
title(ax2, "Occupancy grid: white free, black occupied, gray unknown");
xlabel(ax2, "forward (m)"); ylabel(ax2, "left (m)");
end

function saveCameraVideo(runDir, n)
% The saved photos as an MP4 (like make_video.m).
v = VideoWriter(fullfile(runDir, "cameraFrames.mp4"), "MPEG-4");
v.FrameRate = 5;
open(v);
for i = 1:n
    writeVideo(v, imread(fullfile(runDir, "frames", sprintf("frame_%04d.jpg", i))));
end
close(v);
end

% =====================================================================
% Replay
% =====================================================================
function replay = loadRun(runDir)
data = load(fullfile(runDir, "vslamRun.mat"), "log", "intrinsics");
files = dir(fullfile(runDir, "frames", "frame_*.jpg"));
files = sort(string(fullfile({files.folder}, {files.name})));
n = min(numel(files), height(data.log));
replay.runLog = data.log(1:n, :);
replay.frameFiles = files(1:n);
replay.intrinsics = data.intrinsics;
end
