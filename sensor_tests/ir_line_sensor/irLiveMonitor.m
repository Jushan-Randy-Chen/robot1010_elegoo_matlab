function liveLog = irLiveMonitor(robot, durationSec, options)
% Live ui of the three IR line sensors with the motors off. It shows the
% raw readings, the firmware's onLine flags and the steering decision that
% the line tracker would make (see lineTrackerDecision). Slide black tape
% under the sensors and watch the decision change. Close the figure to stop.
%
%   robot = ElegooRobotCar("192.168.4.1");
%   liveLog = irLiveMonitor(robot, 30);
%
%   robot = ElegooSimIRCar();  robot.setSurface("track");   % moving tape strip
%   liveLog = irLiveMonitor(robot, 10);
%
% liveLog is a table: Time (s), IR (ADC counts), OnLine (0/1), Decision.
% Sensor order is [left middle right].

arguments
    robot
    durationSec (1,1) double {mustBePositive} = 30
    options.SamplePause (1,1) double {mustBeNonnegative} = 0.02
    options.WindowSec (1,1) double {mustBePositive} = 10
    options.ShowFigure (1,1) logical = true
end

robot.stop();
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot));

threshold = NaN;
firmwareState = robot.state();
if isstruct(firmwareState) && isfield(firmwareState, "irThreshold")
    threshold = double(firmwareState.irThreshold);
end

if options.ShowFigure
    ui = createView(threshold, options.WindowSec);
    drawnow    % show the figure before the clock starts
end

capacity = 1000;
times = zeros(capacity, 1);
irLog = zeros(capacity, 3);
onLineLog = zeros(capacity, 3);
decisions = strings(capacity, 1);
count = 0;
previous = "";

fprintf("Live IR check for %g s. Motors are off. Pattern [left middle right]:\n", durationSec);
clock = tic;
while toc(clock) < durationSec
    if options.ShowFigure && ~isvalid(ui.fig)
        break
    end
    response = robot.readIR();
    ir = double(response.ir(:).');
    onLine = double(response.onLine(:).');
    decision = lineTrackerDecision(onLine);

    count = count + 1;
    if count > capacity
        capacity = 2 * capacity;
        times(capacity) = 0;
        irLog(capacity, :) = 0;
        onLineLog(capacity, :) = 0;
        decisions(capacity) = "";
    end
    times(count) = toc(clock);
    irLog(count, :) = ir;
    onLineLog(count, :) = onLine;
    decisions(count) = decision;

    pattern = sprintf("[%d %d %d] %s", onLine, decision);
    if pattern ~= previous
        fprintf("%6.2f s  %s   ir = [%4d %4d %4d]\n", times(count), pattern, ir);
        previous = pattern;
    end
    if options.ShowFigure
        updateView(ui, times(1:count), irLog(1:count, :), onLine, decision, options.WindowSec);
    end
    pause(options.SamplePause);
end

liveLog = table(times(1:count), irLog(1:count, :), onLineLog(1:count, :), decisions(1:count), ...
    'VariableNames', ["Time" "IR" "OnLine" "Decision"]);
fprintf("Live IR check done: %d readings in %.1f s.\n", count, toc(clock));
end

% ======================================================================

function ui = createView(threshold, windowSec)
colors = [0.165 0.471 0.839; 0.922 0.408 0.204; 0.106 0.686 0.478];  % left, middle, right
names = ["Left (A2)" "Middle (A1)" "Right (A0)"];

ui.fig = figure("Name", "IR live monitor", "Color", "w");
layout = tiledlayout(ui.fig, 3, 1, "TileSpacing", "compact");

ui.ax = nexttile(layout, [2 1]);
hold(ui.ax, "on");
for k = 1:3
    ui.lines(k) = plot(ui.ax, NaN, NaN, "-", "LineWidth", 2, ...
        "Color", colors(k, :), "DisplayName", names(k));
end
if ~isnan(threshold)
    yline(ui.ax, threshold, "--", "Color", [0.3 0.3 0.3], "LineWidth", 1.5, ...
        "DisplayName", sprintf("Firmware threshold (%d)", threshold));
end
hold(ui.ax, "off");
ylim(ui.ax, [0 1100]);
xlim(ui.ax, [0 windowSec]);
xlabel(ui.ax, "Time (s)");
ylabel(ui.ax, "Reading, ADC counts");
title(ui.ax, "IR line sensors (motors off)");
legend(ui.ax, "Location", "northoutside", "Orientation", "horizontal");
grid(ui.ax, "on");

% One box per sensor: black when the firmware says onLine = 1.
flagsAx = nexttile(layout);
axis(flagsAx, [0 3 0 1]);
axis(flagsAx, "off");
for k = 1:3
    ui.boxes(k) = rectangle(flagsAx, "Position", [k - 0.95, 0.15, 0.9, 0.55], ...
        "FaceColor", "w", "EdgeColor", [0.3 0.3 0.3], "LineWidth", 1.5);
    text(flagsAx, k - 0.5, 0.05, names(k), "HorizontalAlignment", "center");
end
ui.decision = text(flagsAx, 1.5, 0.92, "", "HorizontalAlignment", "center", ...
    "FontSize", 14, "FontWeight", "bold");
end

function updateView(ui, times, irLog, onLine, decision, windowSec)
for k = 1:3
    set(ui.lines(k), "XData", times, "YData", irLog(:, k));
    if onLine(k)
        ui.boxes(k).FaceColor = "k";
    else
        ui.boxes(k).FaceColor = "w";
    end
end
ui.decision.String = sprintf("onLine [%d %d %d]  ->  %s", onLine, upper(decision));
xlim(ui.ax, [max(0, times(end) - windowSec), max(windowSec, times(end))]);
drawnow limitrate
end

function stopRobotSafely(robot)
try
    robot.stop();
catch
    % Best effort if Wi-Fi has already disconnected.
end
end
