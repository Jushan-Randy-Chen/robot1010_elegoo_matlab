%% IR line sensor test for the ELEGOO Smart Robot Car V4
% Run this script to test the three IR sensors that the line tracker uses.

%% Settings
USE_SIMULATOR = true;          % false = real car over Wi-Fi
HOST = "192.168.4.1";          % car address on its own Wi-Fi network
LIVE_SECONDS = 15;             % live check at the end; 0 = no live check
SAVE_FILE = "";                % for example "irTest.mat"; "" = do not save

%% Connect
if USE_SIMULATOR
    robot = ElegooSimIRCar();
else
    % Find ElegooRobotCar.m.
    if exist("ElegooRobotCar", "file") ~= 2
        addpath(fullfile(fileparts(mfilename("fullpath")), "..", "..", "basic_wireless_control"));
    end
    robot = ElegooRobotCar(HOST);
end

%% Test
report = irSensorTest(robot, LiveSeconds=LIVE_SECONDS, SaveFile=SAVE_FILE);
disp(report.results)
