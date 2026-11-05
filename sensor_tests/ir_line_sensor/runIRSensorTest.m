%% IR line sensor test for the ELEGOO Smart Robot Car V4
% Run this script to test the three IR sensors that the line tracker uses.
% The motors stay off. You need a light, flat floor and matte black tape.
%
% FIRST USE
% 1. Set USE_SIMULATOR = true and run the script once. No car is needed.
% 2. Join the car's Wi-Fi network (ELEGOO-...). Set USE_SIMULATOR = false.
% 3. Run the script and follow the prompts in the Command Window.
%
% The test sends the recommended threshold to the car (setIRThreshold).
% The car forgets it at power off. Run this test again after a restart,
% or on a new floor.

%% Settings
USE_SIMULATOR = true;          % false = real car over Wi-Fi
HOST = "192.168.4.1";          % car address on its own Wi-Fi network
LIVE_SECONDS = 15;             % live check at the end; 0 = no live check
SAVE_FILE = "";                % for example "irTest.mat"; "" = do not save

%% Connect
if USE_SIMULATOR
    robot = ElegooSimIRCar();
else
    % Find ElegooRobotCar.m. In the course repository, it is in the
    % basic_wireless_control folder.
    if exist("ElegooRobotCar", "file") ~= 2
        addpath(fullfile(fileparts(mfilename("fullpath")), "..", "..", "basic_wireless_control"));
    end
    robot = ElegooRobotCar(HOST);
end

%% Test
report = irSensorTest(robot, LiveSeconds=LIVE_SECONDS, SaveFile=SAVE_FILE);
disp(report.results)
