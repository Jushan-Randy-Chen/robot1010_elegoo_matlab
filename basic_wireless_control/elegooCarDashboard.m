function elegooCarDashboard()
    % ELEGOO Smart Robot Car Dashboard & Control UI
    % Connects using the provided ElegooRobotCar class framework.

    % --- 1. Connection Setup ---
    hostIP = '192.168.4.1'; % Default AP mode IP for Elegoo ESP32
    try
        robot = ElegooRobotCar(hostIP);
        robot.stop();
        robot.setWatchdog(1000);
        cleanupObj = onCleanup(@() safeClose(robot));
    catch ME
        uialert(uifigure, ['Failed to connect to robot at ', hostIP, '. Check connection. Error: ', ME.message], 'Connection Error');
        return;
    end

    % State flags for continuous sweeping and continuous teleoperation
    sweepActive = false;
    isDriving = false;

    % --- 2. Create Main UI Figure & Layout ---
    fig = uifigure('Name', 'Elegoo Robot Car Control & Sensor Dashboard', 'Position', [30, 30, 1350, 900]);
    
    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {'4.5x', '2x'};

    leftGrid = uigridlayout(mainGrid, [2, 1]);
    leftGrid.RowHeight = {'2.8x', '3.5x'};

    rightGrid = uigridlayout(mainGrid, [2, 1]);
    rightGrid.RowHeight = {'2.5x', '2.5x'};

    % ==================== SENSOR VISUALIZATION PANEL (Enlarged Plots) ====================
    sensorPanel = uipanel(leftGrid, 'Title', 'Real-Time Sensor Readings & Camera');
    sensorGrid = uigridlayout(sensorPanel, [1, 4]);

    axIMU = uiaxes(sensorGrid); title(axIMU, 'Gyro (Z) & Accel (Y)');
    xlabel(axIMU, 'Sample'); ylabel(axIMU, 'Val');
    
    axRange = uiaxes(sensorGrid); title(axRange, 'Ultrasonic Range');
    xlabel(axRange, 'Sample'); ylabel(axRange, 'Distance (cm)');
    
    axLine = uiaxes(sensorGrid); title(axLine, 'Line Track (IR)');
    xlabel(axLine, 'Sample'); ylabel(axLine, 'Digital');
    
    axCam = uiaxes(sensorGrid); title(axCam, 'Camera Feed');
    axis(axCam, 'off');

    % ==================== DRIVE CONTROL PANEL (Enlarged Arrow Buttons) ====================
    drivePanel = uipanel(leftGrid, 'Title', 'Drive Control & Teleoperation');
    driveGrid = uigridlayout(drivePanel, [1, 2]);
    driveGrid.ColumnWidth = {'1x', '1.2x'};

    % Left sub-grid: Specified v, omega, T parameters & Timed Drive Button
    paramGrid = uigridlayout(driveGrid, [4, 2]);
    paramGrid.RowHeight = {'1x', '1x', '1x', '1.5x'};
    paramGrid.ColumnWidth = {'2.2x', '1x'};
    
    uilabel(paramGrid, 'Text', 'Linear Speed v (-255 to 255):');
    vInput = uieditfield(paramGrid, 'numeric', 'Value', 100, 'Limits', [-255, 255]);

    uilabel(paramGrid, 'Text', 'Angular Speed omega (-255 to 255):');
    omegaInput = uieditfield(paramGrid, 'numeric', 'Value', 30, 'Limits', [-255, 255]);

    uilabel(paramGrid, 'Text', 'Duration T (seconds):');
    tInput = uieditfield(paramGrid, 'numeric', 'Value', 3, 'Limits', [0.1, 60]);

    runTimedBtn = uibutton(paramGrid, 'Text', 'Run Timed Drive (v, \omega, T)', ...
        'BackgroundColor', [0.6 0.8 1], ...
        'ButtonPushedFcn', @(~,~) executeTimedDrive());
    runTimedBtn.Layout.Column = [1, 2];
    
    % Right sub-grid: Teleoperation Direction Arrow Buttons (Enlarged & Prominent)
    teleopGrid = uigridlayout(driveGrid, [4, 3]);
    teleopGrid.RowHeight = {'1.2x', '1.2x', '1.2x', '0.8x'};

    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    btnForward = uibutton(teleopGrid, 'Text', '▲ Forward', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.forward(vInput.Value)));
    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);

    btnLeft = uibutton(teleopGrid, 'Text', '◄ Turn Left', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.turnLeft(omegaInput.Value)));
    btnStop = uibutton(teleopGrid, 'Text', '■ STOP', 'BackgroundColor', [1 0.6 0.6], ...
        'ButtonPushedFcn', @(~,~) stopDrive());
    btnRight = uibutton(teleopGrid, 'Text', 'Turn Right ►', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.turnRight(omegaInput.Value)));

    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    btnBackward = uibutton(teleopGrid, 'Text', '▼ Backward', ...
        'ButtonPushedFcn', @(~,~) startContinuousDrive(@() robot.backward(vInput.Value)));
    uipanel(teleopGrid, 'BackgroundColor', [0.94 0.94 0.94]);

    % Pulse length input row spanning teleop grid
    uilabel(teleopGrid, 'Text', 'Pulse Time (s):');
    pulseInput = uieditfield(teleopGrid, 'numeric', 'Value', 0.2, 'Limits', [0.01, 5.0]);
    pulseInput.Layout.Column = [2, 3];

    % ==================== SCANNING CONFIGURATION PANEL ====================
    scanPanel = uipanel(rightGrid, 'Title', 'Scanning Configuration');
    scanGrid = uigridlayout(scanPanel, [3, 2]);

    uilabel(scanGrid, 'Text', 'Scan Mode:');
    scanModeDropdown = uidropdown(scanGrid, 'Items', {'Fixed Angle', 'Continuous Sweep'}, ...
        'ValueChangedFcn', @(src, ~) toggleScanMode(src.Value));

    uilabel(scanGrid, 'Text', 'Pan Angle (0 to 180 deg):');
    angleSpinner = uispinner(scanGrid, 'Value', 90, 'Limits', [0, 180], ...
        'ValueChangedFcn', @(src, ~) applyFixedAngle(angleSpinner.Value));

    applyScanBtn = uibutton(scanGrid, 'Text', 'Apply Fixed Angle', ...
        'ButtonPushedFcn', @(~,~) applyFixedAngle(angleSpinner.Value));
    
    uipanel(scanGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    uipanel(scanGrid, 'BackgroundColor', [0.94 0.94 0.94]);
    uipanel(scanGrid, 'BackgroundColor', [0.94 0.94 0.94]);

    % ==================== DATA & SNAPSHOT WORKSPACE PANEL ====================
    dataPanel = uipanel(rightGrid, 'Title', 'Data Logging & Snapshots');
    dataGrid = uigridlayout(dataPanel, [3, 1]);

    uibutton(dataGrid, 'Text', 'Capture Camera Snapshot', ...
        'ButtonPushedFcn', @(~,~) captureAndShowSnapshot());

    uibutton(dataGrid, 'Text', 'Save Current Drive Data to Workspace', ...
        'ButtonPushedFcn', @(~,~) saveWorkspaceData());

    uibutton(dataGrid, 'Text', 'Take UI Window Snapshot (PNG)', ...
        'ButtonPushedFcn', @(~,~) takeUISnapshot(fig));

    % --- Data Buffers for Real-Time Plotting ---
    maxPoints = 50;
    timeBuffer = zeros(1, maxPoints);
    gyroBuffer = zeros(1, maxPoints);
    accelYBuffer = zeros(1, maxPoints);
    rangeBuffer = zeros(1, maxPoints);
    lineBuffer  = zeros(1, maxPoints);
    
    % --- Background Polling Timer for Sensors ---
    sensorTimer = timer('Period', 0.5, 'ExecutionMode', 'fixedRate', ...
        'TimerFcn', @(~,~) updateSensors());
    start(sensorTimer);

    % Ensure cleanup when UI closes
    fig.CloseRequestFcn = @(~,~) closeUI(fig, sensorTimer, robot);

    % --- Nested Callback Functions ---
    function updateSensors()
        try
            imuData = robot.readIMU();%[span_0](start_span)[span_0](end_span)
            distData = robot.readDistance();%[span_1](start_span)[span_1](end_span)[span_2](start_span)[span_2](end_span)
            irData = robot.readIR();%[span_3](start_span)[span_3](end_span)

            gVal = 0; ayVal = 0;
            if isstruct(imuData)
                if isfield(imuData, 'gyro')
                    gVal = imuData.gyro(3);%[span_4](start_span)[span_4](end_span)
                end
                if isfield(imuData, 'accel')
                    ayVal = imuData.accel(2);%[span_5](start_span)[span_5](end_span)
                end
            end

            dVal = 0;
            if isstruct(distData) && isfield(distData, 'distance')
                dVal = double(distData.distance);%[span_6](start_span)[span_6](end_span)
            end

            lVal = 0;
            if isstruct(irData)
                if isfield(irData, 'State')
                    lVal = double(irData.State);
                elseif isfield(irData, 'IR')
                    lVal = double(sum(irData.IR));
                end
            end

            timeBuffer = [timeBuffer(2:end), toc];
            gyroBuffer = [gyroBuffer(2:end), gVal];
            accelYBuffer = [accelYBuffer(2:end), ayVal];
            rangeBuffer = [rangeBuffer(2:end), dVal];
            lineBuffer  = [lineBuffer(2:end), lVal];

            plot(axIMU, timeBuffer, gyroBuffer, 'r-', timeBuffer, accelYBuffer, 'b--');
            legend(axIMU, {'Gyro Z', 'Accel Y'}, 'Location', 'northwest');
            
            plot(axRange, timeBuffer, rangeBuffer, 'b-');
            plot(axLine, timeBuffer, lineBuffer, 'g-');
        catch
        end
    end

    function executeTimedDrive()
        v = vInput.Value;
        omega = omegaInput.Value;
        T = tInput.Value;
        
        timerObj = tic;
        while toc(timerObj) < T
            robot.forward(v);%[span_7](start_span)[span_7](end_span)
            if omega ~= 0
                robot.drive(max(min(v + omega, 255), -255), max(min(v - omega, 255), -255));%[span_8](start_span)[span_8](end_span)
            end
            pause(0.02);
        end
        robot.stop();%[span_9](start_span)[span_9](end_span)
    end

    function startContinuousDrive(driveFunc)
        isDriving = true;
        pTime = pulseInput.Value;
        driveFunc();
        
        startTime = tic;
        while isDriving && toc(startTime) < (pTime + 0.1)
            pause(0.02);
        end
        if isDriving
        else
            robot.stop();%[span_10](start_span)[span_10](end_span)
        end
    end

    function stopDrive()
        isDriving = false;
        try
            robot.stop();%[span_11](start_span)[span_11](end_span)
        catch
        end
    end

    function toggleScanMode(modeStr)
        if strcmp(modeStr, 'Continuous Sweep')
            sweepActive = true;
            scanModeDropdown.Value = 'Continuous Sweep';
            
            angles = [0, 30, 60, 90, 120, 150, 180, 150, 120, 90, 60, 30];
            idx = 1;
            
            while sweepActive && isvalid(fig)
                robot.pan(angles(idx));%[span_12](start_span)[span_12](end_span)
                idx = mod(idx, length(angles)) + 1;
                pause(0.25);
            end
        else
            sweepActive = false;
        end
    end

    function applyFixedAngle(angleVal)
        sweepActive = false;
        scanModeDropdown.Value = 'Fixed Angle';
        try
            robot.pan(angleVal);%[span_13](start_span)[span_13](end_span)
        catch
        end
    end

    function captureAndShowSnapshot()
        try
            frame = robot.capture();%[span_14](start_span)[span_14](end_span)
            imshow(frame, 'Parent', axCam);
            title(axCam, 'Camera Feed');
        catch ME
            uialert(fig, ['Failed to grab camera frame: ', ME.message], 'Camera Error');
        end
    end

    function saveWorkspaceData()
        carData = struct(...
            'time', timeBuffer, ...
            'gyro', gyroBuffer, ...
            'accelY', accelYBuffer, ...
            'range', rangeBuffer, ...
            'line', lineBuffer, ...
            'timestamp', datetime('now') ...
        );
        assignin('base', 'carDriveData', carData);
        uialert(fig, 'Sensor and drive arrays successfully saved to workspace variable "carDriveData".', 'Workspace Saved');
    end

    function takeUISnapshot(targetFig)
        img = getframe(targetFig);
        filename = ['ElegooUI_Snapshot_', datestr(now, 'yyyymmdd_HHMMSS'), '.png'];
        imwrite(img.cdata, filename);
        uialert(fig, ['UI snapshot successfully exported to ', filename], 'Snapshot Exported');
    end
end

function safeClose(robot)
    try
        robot.stop();%[span_15](start_span)[span_15](end_span)
        robot.close();%[span_16](start_span)[span_16](end_span)[span_17](start_span)[span_17](end_span)
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
