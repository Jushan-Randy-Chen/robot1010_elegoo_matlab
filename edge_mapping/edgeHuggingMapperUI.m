function edgeHuggingMapperUI()
    % ELEGOO Smart Robot Car - Perpetual Edge Crawler UI
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
    fig = uifigure('Name', 'Elegoo Robot Car - Perpetual Edge Crawler', 'Position', [50, 50, 1100, 750]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'3x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.3x', '1.7x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== SENSOR & STATUS PANEL ====================
    sensorPanel = uipanel(leftGrid, 'Title', 'Live Sensors & Crawler Status');
    sensorGrid = uigridlayout(sensorPanel, [1, 2]);

    axSensor = uiaxes(sensorGrid); title(axSensor, 'IR onLine State & Range Readings');
    xlabel(axSensor, 'Sample'); ylabel(axSensor, 'State / Distance');

    statusTextArea = uitextarea(sensorGrid, 'Editable', 'off', ...
        'Value', ["System Ready.", "Click 'Start Edge Hugging' to begin perpetual edge crawl."]);

    % ==================== AUTOMATED ROUTINE CONTROL PANEL ====================
    controlPanel = uipanel(leftGrid, 'Title', 'Perimeter Crawler Controls');
    controlGrid = uigridlayout(controlPanel, [5, 2]);

    uilabel(controlGrid, 'Text', 'Linear Speed v:');
    speedInput = uieditfield(controlGrid, 'numeric', 'Value', 70, 'Limits', [10, 255]);

    uilabel(controlGrid, 'Text', 'Turn Speed omega:');
    omegaInput = uieditfield(controlGrid, 'numeric', 'Value', 100, 'Limits', [10, 255]);

    uilabel(controlGrid, 'Text', 'Slight Left Turn Duration (s):');
    turnTimeInput = uieditfield(controlGrid, 'numeric', 'Value', 0.25, 'Limits', [0.05, 2.0]);

    startMapBtn = uibutton(controlGrid, 'Text', 'Start Perpetual Edge Hugging', ...
        'BackgroundColor', [0.4 0.8 0.4], ...
        'ButtonPushedFcn', @(~,~) runEdgeHuggingRoutine(speedInput.Value, omegaInput.Value, turnTimeInput.Value));
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
        "1. Drives forward until ANY sensor shows 1 (edge detected)."; ...
        "2. Backs up slightly from the edge."; ...
        "3. Turns left SLIGHTLY (adjustable duration)."; ...
        "4. Drives forward to push back to edge."; ...
        "5. Repeats indefinitely until STOP is pressed." ...
    ]);

    uibutton(infoGrid, 'Text', 'Emergency Stop Robot', ...
        'BackgroundColor', [1 0.3 0.3], ...
        'ButtonPushedFcn', @(~,~) robot.stop());

    uibutton(infoGrid, 'Text', 'Save Crawler Log to Workspace', ...
        'ButtonPushedFcn', @(~,~) assignin('base', 'crawlerLog', statusTextArea.Value));

    maxPoints = 40;
    timeBuffer = zeros(1, maxPoints);
    irSumBuffer = zeros(1, maxPoints);
    
    fig.CloseRequestFcn = @(~,~) closeUI(fig, robot);

    function logMsg(msg)
        currentText = string(statusTextArea.Value);
        timestampedMsg = "[" + datestr(now, 'HH:MM:SS') + "] " + msg;
        statusTextArea.Value = [timestampedMsg; currentText];
        drawnow limitrate;
    end

    function runEdgeHuggingRoutine(speed, turnSpd, leftTurnDuration)
        if isRoutineRunning
            return;
        end
        isRoutineRunning = true;
        logMsg('Starting perpetual edge-hugging crawler...');

        stepCount = 0;

        try
            while isRoutineRunning
                stepCount = stepCount + 1;
                logMsg(['Driving to edge (Cycle #' + string(stepCount) + ')...']);
                
                consecutiveEdgeCount = 0;
                requiredConsecutive = 2;
                
                % ---------------- 1. DRIVE FORWARD TO EDGE ----------------
                while isRoutineRunning
                    robot.forward(speed);
                    
                    irData = [];
                    try
                        irData = robot.readIR();
                    catch
                    end
                    
                    edgeDetected = false;
                    
                    if ~isempty(irData) && isstruct(irData) && isfield(irData, 'onLine')
                        % Update plot
                        irSum = sum(double(irData.onLine));
                        timeBuffer = [timeBuffer(2:end), toc];
                        irSumBuffer = [irSumBuffer(2:end), irSum];
                        plot(axSensor, timeBuffer, irSumBuffer, 'r-');
                        legend(axSensor, {'IR sum (onLine)'}, 'Location', 'northwest');

                        % Check if ANY sensor detects 1 (edge)
                        if any(irData.onLine == 1)
                            consecutiveEdgeCount = consecutiveEdgeCount + 1;
                            if consecutiveEdgeCount >= requiredConsecutive
                                edgeDetected = true;
                            end
                        else
                            consecutiveEdgeCount = 0;
                        end
                    end
                    
                    if edgeDetected
                        logMsg(['Edge detected! Stopping...']);
                        break;
                    end
                    pause(0.03);
                end

                if ~isRoutineRunning, break; end

                % ---------------- 2. STOP AT EDGE ----------------
                robot.stop();
                pause(0.2);

                if ~isRoutineRunning, break; end

                % ---------------- 3. BACK UP ----------------
                logMsg('Backing up from edge...');
                tBack = tic;
                while isRoutineRunning && toc(tBack) < 0.5
                    robot.backward(speed);
                    pause(0.05);
                end
                robot.stop();
                pause(0.2);

                if ~isRoutineRunning, break; end

                % ---------------- 4. TURN LEFT SLIGHTLY ----------------
                logMsg('Turning left slightly...');
                tTurn = tic;
                while isRoutineRunning && toc(tTurn) < leftTurnDuration
                    robot.turnLeft(turnSpd);
                    pause(0.05);
                end
                robot.stop();
                pause(0.2);
            end

            logMsg('Edge hugging routine stopped.');

        catch ME
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

function closeUI(fig, robot)
    try
        safeClose(robot);
    catch
    end
    delete(fig);
end
