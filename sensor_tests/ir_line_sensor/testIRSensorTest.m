function tests = testIRSensorTest
% Unit tests for the IR line sensor test module. They use ElegooSimIRCar,
% so no car is needed. Run: runtests("testIRSensorTest")
tests = functiontests(localfunctions);
end

function setup(testCase)
rng(1010, "twister");
testCase.TestData.options = {"ShowFigure", false, "Samples", 20};
end

% ---------------- Simulator matches the firmware format ----------------

function testSimulatorResponseFormat(testCase)
robot = ElegooSimIRCar("ReadDelay", 0);
response = robot.readIR();
verifyEqual(testCase, sort(fieldnames(response)), sort({'ir'; 'onLine'}));
verifySize(testCase, response.ir, [3 1]);          % jsondecode of [L,M,R]
verifySize(testCase, response.onLine, [3 1]);
robot.setSurface(["light" "black" "lifted"]);
verifyEqual(testCase, robot.readIR().onLine, [0; 1; 1]);
verifyEqual(testCase, robot.setIRThreshold(2000).ir_threshold, 1023);
end

function testSimulatorRejectsBadSurface(testCase)
robot = ElegooSimIRCar("ReadDelay", 0);
verifyError(testCase, @() robot.setSurface("grey"), "ElegooSimIRCar:BadSurface");
verifyError(testCase, @() robot.setSurface(["track" "light" "light"]), "ElegooSimIRCar:BadSurface");
end

% ---------------- Line tracker rule ----------------

function testLineTrackerDecision(testCase)
patterns = [1 1 1; 0 1 0; 1 0 0; 1 1 0; 0 0 1; 0 1 1; 0 0 0; 1 0 1];
expected = ["forward" "forward" "left" "left" "right" "right" "stop" "stop"];
for k = 1:size(patterns, 1)
    verifyEqual(testCase, lineTrackerDecision(patterns(k, :)), expected(k), ...
        sprintf("pattern [%d %d %d]", patterns(k, :)));
end
end

% ---------------- Full test on a healthy and on faulty sensors ----------------

function testHealthySensorsPass(testCase)
robot = ElegooSimIRCar("ReadDelay", 0);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyTrue(testCase, report.passed);
verifyEqual(testCase, report.results.Result.', ...
    ["PASS" "PASS" "PASS" "PASS" "PASS" "PASS" "PASS" "PASS" "PASS"]);
threshold = report.recommendedThreshold;
verifyGreaterThan(testCase, threshold, max(robot.LightLevel));
verifyLessThan(testCase, threshold, min(robot.BlackLevel));
verifyEqual(testCase, robot.IRThreshold, threshold);     % sent to the firmware
verifyEqual(testCase, robot.Motors, [0 0]);              % never drove
end

function testNoApplyKeepsFirmwareThreshold(testCase)
robot = ElegooSimIRCar("ReadDelay", 0);
report = irSensorTest(robot, testCase.TestData.options{:}, "ApplyThreshold", false);
verifyTrue(testCase, report.passed);
verifyEqual(testCase, resultOf(report, "Firmware update"), "SKIP");
verifyEqual(testCase, robot.IRThreshold, 600);
end

function testStuckSensorFails(testCase)
robot = ElegooSimIRCar("ReadDelay", 0, "StuckSensor", 2);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyFalse(testCase, report.passed);
verifyEqual(testCase, resultOf(report, "Contrast"), "FAIL");
verifySubstring(testCase, detailOf(report, "Contrast"), "middle");
end

function testReversedSensorFails(testCase)
robot = ElegooSimIRCar("ReadDelay", 0, "BlackLevel", [690 60 705]);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyEqual(testCase, resultOf(report, "Contrast"), "FAIL");
verifySubstring(testCase, detailOf(report, "Contrast"), "LOWER");
verifyEqual(testCase, resultOf(report, "Threshold"), "SKIP");
verifyEqual(testCase, robot.IRThreshold, 600);           % nothing sent
end

function testNoisySensorsFail(testCase)
robot = ElegooSimIRCar("ReadDelay", 0, "Noise", 60);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyEqual(testCase, resultOf(report, "Light floor"), "FAIL");
verifyEqual(testCase, resultOf(report, "Black line"), "FAIL");
end

function testNoCommonThresholdFails(testCase)
% Each sensor has contrast, but the middle one's light level is above the
% left one's black level, so one firmware threshold cannot serve all three.
robot = ElegooSimIRCar("ReadDelay", 0, "LightLevel", [100 450 100], "BlackLevel", [400 900 700]);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyEqual(testCase, resultOf(report, "Contrast"), "PASS");
verifyEqual(testCase, resultOf(report, "Threshold"), "FAIL");
verifyTrue(testCase, isnan(report.recommendedThreshold));
verifyEqual(testCase, report.perSensorThreshold, [250 675 400], "AbsTol", 10);
end

function testLowLiftedReadingFails(testCase)
robot = ElegooSimIRCar("ReadDelay", 0, "LiftedLevel", [990 200 990]);
report = irSensorTest(robot, testCase.TestData.options{:});
verifyEqual(testCase, resultOf(report, "Lifted car"), "FAIL");
end

function testBadResponseStopsEarly(testCase)
robot = ElegooSimIRCar("ReadDelay", 0);
fake = struct("readIR", @() struct("IR", [1 2 3]), ...
    "stop", @() robot.stop(), "setWatchdog", @(ms) robot.setWatchdog(ms));
report = irSensorTest(fake, testCase.TestData.options{:});
verifyFalse(testCase, report.passed);
verifyEqual(testCase, height(report.results), 1);
end

% ---------------- Live monitor ----------------

function testLiveMonitorLogsDecisions(testCase)
robot = ElegooSimIRCar("ReadDelay", 0, "SweepPeriod", 1);
robot.setSurface("track");
liveLog = irLiveMonitor(robot, 1.2, "ShowFigure", false, "SamplePause", 0.005);
verifyGreaterThan(testCase, height(liveLog), 20);
verifyEqual(testCase, liveLog.Properties.VariableNames, {'Time', 'IR', 'OnLine', 'Decision'});
% The tape sweeps across all three sensors, so every decision shows up.
verifyEqual(testCase, sort(unique(liveLog.Decision)).', ["forward" "left" "right" "stop"]);
end

% ---------------- Helpers ----------------

function result = resultOf(report, name)
result = report.results.Result(report.results.Name == name);
end

function detail = detailOf(report, name)
detail = report.results.Detail(report.results.Name == name);
end
