function tableMapperVideoUI()
    % ELEGOO Smart Robot Car - Continuous Table Edge Mapper & Video Recorder UI
    % Connects using the provided ElegooRobotCar class framework.

    % --- 1. Connection Setup ---
    % Find ElegooRobotCar.m. In the course repository, it is in the
    % basic_wireless_control folder.
    if exist("ElegooRobotCar", "file") ~= 2
        addpath(fullfile(fileparts(mfilename("fullpath")), "..", "basic_wireless_control"));
    end
    hostIP = '192.168.4.1'; 
    try
        robot = ElegooRobotCar(hostIP);
        robot.stop();
        robot.setWatchdog(1000);
        cleanupObj = onCleanup(@() safeClose(robot));
    catch ME
        uialert(uifigure, ['Failed to connect to robot at ', hostIP, '. Check connection. Error: ', ME.message], 'Connection Error');
        return;
    end

    isRoutineRunning = false;

    % --- 2. Create Main UI Figure & Layout ---
    fig = uifigure('Name', 'Elegoo Robot Car - Continuous Table Edge Mapper', 'Position', [50, 50, 1100, 750]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'3x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.3x', '1.7x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== SENSOR & STATUS PANEL ====================
    sensorPanel = uipanel(leftGrid, 'Title', 'Live Sensors & Routine Status');
    sensorGrid = uigridlayout(sensorPanel, [1, 2]);

    axSensor = uiaxes(sensorGrid); title(axSensor, 'IR onLine State & Range Readings');
    xlabel(axSensor, 'Sample'); ylabel(axSensor, 'State / Distance');

    statusTextArea = uitextarea(sensorGrid, 'Editable', 'off', ...
        'Value', ["System Ready.", "Click 'Start Continuous Mapping' to begin perpetual loop."]);

    % ==================== AUTOMATED ROUTINE CONTROL PANEL ====================
    controlPanel = uipanel(leftGrid, 'Title', 'Mapping Controls');
    controlGrid = uigridlayout(controlPanel, [5, 2]);

    uilabel(controlGrid, 'Text', 'Linear Speed v:');
    speedInput = uieditfield(controlGrid, 'numeric', 'Value', 70, 'Limits', [10, 255]);

    uilabel(controlGrid, 'Text', 'Turn speed (PWM):');
    turnSpeedInput = uieditfield(controlGrid, 'numeric', 'Value', 100, 'Limits', [10, 255]);

    uilabel(controlGrid, 'Text', 'Turn Duration (s) for 90°:');
    turnTimeInput = uieditfield(controlGrid, 'numeric', 'Value', 0.5, 'Limits', [0.1, 5.0]);

    startMapBtn = uibutton(controlGrid, 'Text', 'Start Continuous Mapping & Video', ...
        'BackgroundColor', [0.4 0.8 0.4], ...
        'ButtonPushedFcn', @(~,~) runPerpetualTableMapping(speedInput.Value, turnSpeedInput.Value, turnTimeInput.Value));
    startMapBtn.Layout.Column = [1, 2];

    stopRoutineBtn = uibutton(controlGrid, 'Text', 'STOP ROUTINE', ...
        'BackgroundColor', [1 0.6 0.6], ...
        'ButtonPushedFcn', @(~,~) stopRoutine());
    stopRoutineBtn.Layout.Column = [1, 2];

    % ==================== SIDE INFO & MANUAL OVERRIDE ====================
    infoPanel = uipanel(rightGrid, 'Title', 'Routine Configuration & Info');
    infoGrid = uigridlayout(infoPanel, [3, 1]);

    infoText = uitextarea(infoGrid, 'Editable', 'off', 'Value', [ ...
        "Routine Protocol:"; ...
        "1. Drives forward until edge [1 1 1] is reached."; ...
        "2. Saves snapshot to video file at each edge."; ...
        "3. Backs up and turns 90 degrees."; ...
        "4. Loops perpetually across all table sides until STOP is pressed." ...
    ]);

    uibutton(infoGrid, 'Text', 'Emergency Stop Robot', ...
        'BackgroundColor', [1 0.3 0.3], ...
        'ButtonPushedFcn', @(~,~) robot.stop());

    uibutton(infoGrid, 'Text', 'Save Mapping Log to Workspace', ...
        'ButtonPushedFcn', @(~,~) assignin('base', 'mappingLog', statusTextArea.Value));

    maxPoints = 40;
    timeBuffer = zeros(1, maxPoints);
    irSumBuffer = zeros(1, maxPoints);
    rangeBuffer = zeros(1, maxPoints);

    sensorTimer = timer('Period', 0.4, 'ExecutionMode', 'fixedRate', ...
        'TimerFcn', @(~,~) updateSensors());
    start(sensorTimer);

    fig.CloseRequestFcn = @(~,~) closeUI(fig, sensorTimer, robot);

    function updateSensors()
        try
            irData = robot.readIR();
            distData = robot.readDistance();

            irSum = 0;
            if isstruct(irData) && isfield(irData, 'onLine')
                irSum = sum(double(irData.onLine));
            end

            dVal = 0;
            if isstruct(distData) && isfield(distData, 'distance')
                dVal = double(distData.distance);
            end

            timeBuffer = [timeBuffer(2:end), toc];
            irSumBuffer = [irSumBuffer(2:end), irSum];
            rangeBuffer = [rangeBuffer(2:end), dVal];

            plot(axSensor, timeBuffer, irSumBuffer, 'r-', timeBuffer, rangeBuffer/10, 'b--');
            legend(axSensor, {'IR sum (onLine)', 'Range / 10'}, 'Location', 'northwest');
        catch
        end
    end

    function logMsg(msg)
        currentText = string(statusTextArea.Value);
        timestampedMsg = "[" + datestr(now, 'HH:MM:SS') + "] " + msg;
        statusTextArea.Value = [timestampedMsg; currentText];
    end

    function runPerpetualTableMapping(speed, turnSpd, turnDuration)
        if isRoutineRunning
            return;
        end
        isRoutineRunning = true;
        logMsg('Starting continuous table edge mapping routine...');

        videoFileName = ['TableEdgeMapping_', datestr(now, 'yyyymmdd_HHMMSS'), '.mp4'];
        vWriter = VideoWriter(videoFileName, 'MPEG-4');
        vWriter.FrameRate = 2; 
        open(vWriter);

        edgeSnapshots = {};
        totalEdgeCount = 0;

        try
            % Perpetual loop until isRoutineRunning is set to false
            while isRoutineRunning
                totalEdgeCount = totalEdgeCount + 1;
                logMsg(['Approaching edge #' + string(totalEdgeCount) + '...']);
                
                consecutiveEdgeCount = 0;
                requiredConsecutive = 2;
                
                % 1. Drive forward until edge reached
                while isRoutineRunning
                    robot.forward(speed);
                    
                    irData = robot.readIR();
                    edgeDetected = false;
                    
                    if isstruct(irData) && isfield(irData, 'onLine')
                        if all(irData.onLine == [1 1 1])
                            consecutiveEdgeCount = consecutiveEdgeCount + 1;
                            if consecutiveEdgeCount >= requiredConsecutive
                                edgeDetected = true;
                            end
                        else
                            consecutiveEdgeCount = 0;
                        end
                    end
                    
                    if edgeDetected
                        logMsg(['Edge [1 1 1] detected for edge #' + string(totalEdgeCount) + '!']);
                        break;
                    end
                    pause(0.02);
                end

                if ~isRoutineRunning, break; end

                % 2. Stop & Capture Snapshot
                robot.stop();
                pause(0.2);
                try
                    frame = robot.capture();
                    edgeSnapshots{end+1} = frame;
                    
                    writeVideo(vWriter, imresize(frame, [480, 640]));
                    logMsg(['Snapshot recorded for edge #' + string(totalEdgeCount)]);
                catch
                    logMsg('Warning: Failed to capture camera frame at edge.');
                end

                if ~isRoutineRunning, break; end

                % 3. Back up safely
                robot.backward(speed);
                pause(0.9);
                robot.stop();
                pause(0.3);

                if ~isRoutineRunning, break; end

                % 4. Turn 90 degrees
                logMsg('Turning 90 degrees...');
                robot.turnRight(turnSpd);
                pause(turnDuration);
                robot.stop();
                pause(0.5);
            end

            close(vWriter);
            logMsg(['Video file saved as: ', videoFileName]);

        catch ME
            try
                close(vWriter);
            catch
            end
            logMsg(['Routine error: ', ME.message]);
            robot.stop();
            isRoutineRunning = false;
        end
    end

    function stopRoutine()
        isRoutineRunning = false;
        try
            robot.stop();
            logMsg('Routine stopped by user.');
        catch
        end
    end
end

function safeClose(robot)
    try
        robot.stop();
        robot.close();
    catch
    end
end

function closeUI(fig, timerObj, robot)
    try
        stop(timerObj);
        delete(timerObj);
        safeClose(robot);
    catch
    end
    delete(fig);
end
