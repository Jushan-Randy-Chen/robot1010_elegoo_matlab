function report = irSensorTest(robot, options)
% Guided test of the three IR line-tracking sensors of the ELEGOO V4.
% The motors stay off for the whole test. Put the car on a light, flat floor
% and have a strip of matte black electrical tape ready. The test tells you
% when to move the car.
%
%   robot  = ElegooRobotCar("192.168.4.1");   % real car with the course firmware
%   robot  = ElegooSimIRCar();                % or the simulator (no prompts)
%   report = irSensorTest(robot);
%   report = irSensorTest(robot, LiveSeconds=20, SaveFile="irTest.mat");
%
% Steps
%   1 Response format  readIR gives ir (3 ADC counts, 0..1023) and onLine (3 of 0/1)
%   2 Read rate        readIR is fast enough for line tracking over Wi-Fi
%   3 onLine rule      the firmware's onLine equals ir > irThreshold
%   4 Light floor      readings are steady and not stuck at an ADC limit
%   5 Black line       readings are steady and not stuck at an ADC limit
%   6 Contrast         black reads higher than light by MinContrast on each sensor
%   7 Threshold        one threshold separates light and black on all sensors
%   8 Lifted car       a lifted car (as at a table edge) reads above the threshold
%   9 Firmware update  setIRThreshold stores the new threshold (lost at power off)
%
% report.results has one row per step (PASS, FAIL or SKIP). report.passed is
% true when no step failed. Sensor order is always [left middle right].

arguments
    robot
    options.Samples (1,1) double {mustBeInteger, mustBeGreaterThanOrEqual(options.Samples, 5)} = 30
    options.MinRateHz (1,1) double {mustBePositive} = 5
    options.MaxNoise (1,1) double {mustBePositive} = 25      % std of a still reading, ADC counts
    options.MinContrast (1,1) double {mustBePositive} = 150  % black minus light, ADC counts
    options.TestLifted (1,1) logical = true
    options.ApplyThreshold (1,1) logical = true
    options.LiveSeconds (1,1) double {mustBeNonnegative} = 0
    options.ShowFigure (1,1) logical = true
    options.SaveFile (1,1) string = ""
end

% The test never drives. Stop old commands and keep the 1 s motor watchdog on.
robot.stop();
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot));

n = options.Samples;
results = table('Size', [0 4], 'VariableTypes', ["double" "string" "string" "string"], ...
    'VariableNames', ["Step" "Name" "Result" "Detail"]);
report = struct("results", results, "passed", false, "rateHz", NaN, ...
    "firmwareThreshold", NaN, "recommendedThreshold", NaN, ...
    "perSensorThreshold", NaN(1, 3), "light", [], "black", [], "lifted", [], "live", []);

fprintf("\nIR line sensor test. Sensor order: [left middle right]. Motors stay off.\n");

% ---------------- 1. Response format ----------------
try
    [ir, onLine] = readSensors(robot);
    results = addResult(results, "Response format", true, ...
        sprintf("ir = [%d %d %d], onLine = [%d %d %d]", ir, onLine));
catch problem
    results = addResult(results, "Response format", false, problem.message);
    report = finish(report, results, options);
    return
end

% ---------------- 2. Read rate, 3. onLine rule ----------------
firmwareState = robot.state();
if isstruct(firmwareState) && isfield(firmwareState, "irThreshold")
    report.firmwareThreshold = double(firmwareState.irThreshold);
end
[irLog, onLineLog, elapsed] = readMany(robot, n);
report.rateHz = n / elapsed;
results = addResult(results, "Read rate", report.rateHz >= options.MinRateHz, ...
    sprintf("%.1f readings/s (%.0f ms each, need %g/s)", ...
    report.rateHz, 1000 * elapsed / n, options.MinRateHz));
results = checkOnLineRule(results, irLog, onLineLog, report.firmwareThreshold);

% ---------------- 4. Light floor, 5. Black line ----------------
prepare(robot, "light", "Put the car on the LIGHT floor. Keep tape away from the sensors.");
report.light = measure(robot, n);
results = checkSurface(results, "Light floor", report.light, options.MaxNoise);

prepare(robot, "black", "Put BLACK tape under ALL THREE sensors. Hold the car level on the floor.");
report.black = measure(robot, n);
results = checkSurface(results, "Black line", report.black, options.MaxNoise);

% ---------------- 6. Contrast, 7. Threshold ----------------
contrast = report.black.mean - report.light.mean;
detail = sprintf("black - light = [%.0f %.0f %.0f] counts", contrast);
reversed = find(contrast < 0);
weak = find(contrast >= 0 & contrast < options.MinContrast);
if ~isempty(reversed)
    detail = detail + sprintf(". %s read LOWER over black, but the firmware sets onLine = ir > threshold", ...
        sensorNames(reversed));
end
if ~isempty(weak)
    detail = detail + sprintf(". %s below %g: use matte black tape on a light floor, check the sensor cable", ...
        sensorNames(weak), options.MinContrast);
end
results = addResult(results, "Contrast", isempty(reversed) && isempty(weak), detail);

report.perSensorThreshold = round((report.light.mean + report.black.mean) / 2);
if ~isempty(reversed)
    results = addResult(results, "Threshold", missing, "Black must read higher than light first");
else
    % Keep 4 standard deviations of noise clear on both sides of the threshold.
    lightTop = max(report.light.mean + 4 * report.light.std);
    blackBottom = min(report.black.mean - 4 * report.black.std);
    if blackBottom > lightTop
        report.recommendedThreshold = round((lightTop + blackBottom) / 2);
        detail = sprintf("use %d (light reaches %.0f, black goes down to %.0f)", ...
            report.recommendedThreshold, lightTop, blackBottom);
        if ~isnan(report.firmwareThreshold)
            if report.firmwareThreshold > lightTop && report.firmwareThreshold < blackBottom
                detail = detail + sprintf(". The current firmware threshold %d also works", ...
                    report.firmwareThreshold);
            else
                detail = detail + sprintf(". The current firmware threshold %d does NOT work here", ...
                    report.firmwareThreshold);
            end
        end
        results = addResult(results, "Threshold", true, detail);
    else
        results = addResult(results, "Threshold", false, sprintf( ...
            "no single threshold works: light reaches %.0f, black goes down to %.0f. " + ...
            "Per-sensor midpoints [%d %d %d] can be used in MATLAB on the ir values", ...
            lightTop, blackBottom, report.perSensorThreshold));
    end
end

% ---------------- 8. Lifted car ----------------
threshold = report.recommendedThreshold;
if isnan(threshold)
    threshold = report.firmwareThreshold;
end
if ~options.TestLifted
    results = addResult(results, "Lifted car", missing, "TestLifted = false");
elseif isnan(threshold)
    results = addResult(results, "Lifted car", missing, "No threshold to compare with");
else
    prepare(robot, "lifted", "LIFT the car about 5 cm off the floor. Hold it level.");
    report.lifted = measure(robot, n);
    low = find(report.lifted.min <= threshold);
    detail = sprintf("lowest [%d %d %d] vs threshold %d", report.lifted.min, threshold);
    if isempty(low)
        detail = detail + ". A table edge reads onLine = 1, as the edge mappers expect";
    else
        detail = detail + sprintf(". %s can miss a table edge", sensorNames(low));
    end
    results = addResult(results, "Lifted car", isempty(low), detail);
end

% ---------------- 9. Firmware update ----------------
if ~options.ApplyThreshold
    results = addResult(results, "Firmware update", missing, "ApplyThreshold = false");
elseif isnan(report.recommendedThreshold)
    results = addResult(results, "Firmware update", missing, "No threshold to send");
else
    robot.setIRThreshold(report.recommendedThreshold);
    firmwareState = robot.state();
    stored = NaN;
    if isstruct(firmwareState) && isfield(firmwareState, "irThreshold")
        stored = double(firmwareState.irThreshold);
    end
    [irLog, onLineLog] = readMany(robot, 5);
    agrees = all(onLineLog == (irLog > stored), "all");
    results = addResult(results, "Firmware update", stored == report.recommendedThreshold && agrees, ...
        sprintf("irThreshold is %g (was %g). The car forgets it at power off", ...
        stored, report.firmwareThreshold));
end

% ---------------- Optional live check ----------------
if options.LiveSeconds > 0
    prepare(robot, "track", "LIVE CHECK: slide the black tape slowly under the sensors, left to right.");
    report.live = irLiveMonitor(robot, options.LiveSeconds, ShowFigure=options.ShowFigure);
end

report = finish(report, results, options);
end

% ======================================================================

function [ir, onLine] = readSensors(robot)
% One readIR call, checked against the course firmware format (N=11).
response = robot.readIR();
if ~isstruct(response) || ~all(isfield(response, ["ir" "onLine"]))
    error("irSensorTest:BadResponse", ...
        "readIR did not return the fields ir and onLine. Is the course firmware on the car?");
end
ir = double(response.ir(:).');
onLine = double(response.onLine(:).');
if numel(ir) ~= 3 || numel(onLine) ~= 3 || any(ir < 0 | ir > 1023 | ir ~= round(ir)) || ...
        any(onLine ~= 0 & onLine ~= 1)
    error("irSensorTest:BadResponse", ...
        "readIR must give 3 ADC counts (0..1023) and 3 onLine flags (0/1).");
end
end

function [irLog, onLineLog, elapsed] = readMany(robot, n)
irLog = zeros(n, 3);
onLineLog = zeros(n, 3);
clock = tic;
for k = 1:n
    [irLog(k, :), onLineLog(k, :)] = readSensors(robot);
end
elapsed = toc(clock);
end

function stats = measure(robot, n)
irLog = readMany(robot, n);
stats = struct("mean", mean(irLog, 1), "std", std(irLog, 0, 1), ...
    "min", min(irLog, [], 1), "max", max(irLog, [], 1), "samples", irLog);
end

function prepare(robot, surface, message)
fprintf("\n%s\n", message);
if isa(robot, "ElegooSimIRCar")
    robot.setSurface(surface);    % the simulator moves the car for you
else
    input('Press Enter when ready: ', 's');
end
end

function results = checkOnLineRule(results, irLog, onLineLog, threshold)
if isnan(threshold)
    results = addResult(results, "onLine rule", missing, "state() gave no irThreshold");
    return
end
agree = all(onLineLog == (irLog > threshold), 2);
results = addResult(results, "onLine rule", all(agree), ...
    sprintf("%d of %d readings agree with ir > %d", sum(agree), numel(agree), threshold));
end

function results = checkSurface(results, name, stats, maxNoise)
detail = sprintf("mean [%.0f %.0f %.0f], noise [%.1f %.1f %.1f]", stats.mean, stats.std);
noisy = find(stats.std > maxNoise);
stuck = find(all(stats.samples == 0 | stats.samples == 1023, 1));
if ~isempty(noisy)
    detail = detail + sprintf(". %s noise above %g: hold the car still, keep away from sunlight", ...
        sensorNames(noisy), maxNoise);
end
if ~isempty(stuck)
    detail = detail + sprintf(". %s stuck at 0 or 1023: check the sensor cable", sensorNames(stuck));
end
results = addResult(results, name, isempty(noisy) && isempty(stuck), detail);
end

function results = addResult(results, name, passed, detail)
% passed is true, false or missing (SKIP).
if ismissing(passed)
    result = "SKIP";
elseif passed
    result = "PASS";
else
    result = "FAIL";
end
step = height(results) + 1;
results(step, :) = {step, name, result, string(detail)};
fprintf("[%s] %d %s: %s\n", result, step, name, detail);
end

function names = sensorNames(index)
labels = ["left" "middle" "right"];
names = "Sensor " + join(labels(index), ", ");
end

function report = finish(report, results, options)
report.results = results;
report.passed = ~any(results.Result == "FAIL");
counts = [sum(results.Result == "PASS"), sum(results.Result == "FAIL"), sum(results.Result == "SKIP")];
verdict = ["FAILED" "PASSED"];
fprintf("\nIR SENSOR TEST %s: %d pass, %d fail, %d skip\n", ...
    verdict(report.passed + 1), counts);
if options.ShowFigure && ~isempty(report.light) && ~isempty(report.black)
    plotReadings(report);
end
if options.SaveFile ~= ""
    save(options.SaveFile, "report");
    fprintf("Saved the report to %s\n", options.SaveFile);
end
end

function plotReadings(report)
surfaces = [report.light; report.black; report.lifted];
names = ["Light floor" "Black tape" "Lifted car"];
names = names(1:numel(surfaces));
colors = [0.165 0.471 0.839; 0.922 0.408 0.204; 0.106 0.686 0.478];  % fixed series order

fig = figure("Name", "IR line sensor test", "Color", "w");
ax = axes(fig);
hold(ax, "on");
bars = bar(ax, 1:3, vertcat(surfaces.mean).', "grouped", "EdgeColor", "none");
for k = 1:numel(bars)
    bars(k).FaceColor = colors(k, :);
    bars(k).DisplayName = names(k);
    errorbar(ax, bars(k).XEndPoints, surfaces(k).mean, surfaces(k).std, "k", ...
        "LineStyle", "none", "HandleVisibility", "off");
end
if ~isnan(report.firmwareThreshold)
    yline(ax, report.firmwareThreshold, "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.5, ...
        "DisplayName", sprintf("Firmware threshold (%d)", report.firmwareThreshold));
end
if ~isnan(report.recommendedThreshold)
    yline(ax, report.recommendedThreshold, "-", "Color", [0.15 0.15 0.15], "LineWidth", 1.5, ...
        "DisplayName", sprintf("Recommended threshold (%d)", report.recommendedThreshold));
end
hold(ax, "off");
ax.XTick = 1:3;
ax.XTickLabel = ["Left (A2)" "Middle (A1)" "Right (A0)"];
ylim(ax, [0 1100]);
ylabel(ax, "Reading, ADC counts (0 to 1023)");
title(ax, "IR line sensors: mean reading per surface (bars show ±1 std)");
legend(ax, "Location", "northoutside", "Orientation", "horizontal", "NumColumns", 3);
grid(ax, "on");
ax.XGrid = "off";
end

function stopRobotSafely(robot)
try
    robot.stop();
catch
    % Best effort if Wi-Fi has already disconnected.
end
end
