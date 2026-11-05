classdef ElegooSimIRCar < handle
    % Simulated IR line-tracking sensors of the ELEGOO Smart Robot Car V4.
    % It answers the IR commands of ElegooRobotCar (readIR, setIRThreshold,
    % state, allSensors) with the same JSON fields as the course firmware,
    % so irSensorTest and irLiveMonitor run without a car. The car stands
    % still: drive commands are accepted and ignored.
    %
    % Reading model (10-bit ADC, 0..1023): a light floor reflects IR and reads
    % low, black tape reads higher, and a lifted car (or a table edge) gets no
    % reflection and reads near the top. The firmware sets onLine = ir > threshold.
    %
    % Example:
    %   robot = ElegooSimIRCar();
    %   robot.setSurface("black");                   % all three sensors
    %   robot.readIR()
    %   robot.setSurface(["light" "black" "light"]); % one surface per sensor
    %   robot.setSurface("track");                   % a tape strip slides under the sensors
    %   robot = ElegooSimIRCar("StuckSensor", 2);    % fault: middle sensor reads a fixed value

    properties
        LightLevel = [95 110 100]     % mean ADC counts over a light floor [left middle right]
        BlackLevel = [690 720 705]    % over black electrical tape
        LiftedLevel = [985 1000 990]  % car lifted about 5 cm, or past a table edge
        Noise = 6                     % std of one reading, ADC counts
        StuckSensor = []              % index (1..3) of a sensor that always reads StuckValue
        StuckValue = 512
        ReadDelay = 0.01              % s per command, like the Wi-Fi round trip
        SensorSpacing = 0.016         % m between neighbor sensors (estimate)
        SpotWidth = 0.006             % m of floor that one sensor sees (estimate)
        TapeWidth = 0.019             % m, standard electrical tape
        SweepAmplitude = 0.035        % m, "track" mode: how far the tape moves to each side
        SweepPeriod = 4               % s, "track" mode: one full left-right-left sweep
    end

    properties (SetAccess = private)
        Surface = ["light" "light" "light"]
        IRThreshold = 600             % firmware default
        Watchdog = 0
        Motors = [0 0]
    end

    properties (Access = private)
        Clock
    end

    methods
        function obj = ElegooSimIRCar(varargin)
            % Optional name-value pairs set properties, e.g. ElegooSimIRCar("Noise", 20).
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k+1};
            end
            obj.Clock = tic;
        end

        % ---------------- Sim-only ----------------
        function setSurface(obj, surface)
            % "light", "black", "lifted" or "track", for all sensors, or a
            % 1x3 string array with one of "light", "black", "lifted" per sensor.
            surface = string(surface);
            if isscalar(surface)
                surface = repmat(surface, 1, 3);
            end
            valid = ["light" "black" "lifted" "track"];
            if numel(surface) ~= 3 || ~all(ismember(surface, valid)) || ...
                    (any(surface == "track") && ~all(surface == "track"))
                error("ElegooSimIRCar:BadSurface", ...
                    "Surface must be one of %s, or three of light/black/lifted.", ...
                    join(valid, ", "));
            end
            obj.Surface = surface;
        end

        % ---------------- Same API as ElegooRobotCar ----------------
        function response = readIR(obj)
            obj.wait();
            [ir, onLine] = obj.sample();
            response = struct("ir", ir, "onLine", onLine);
        end

        function response = setIRThreshold(obj, threshold)
            obj.wait();
            obj.IRThreshold = min(max(round(threshold), 0), 1023);
            response = struct("ir_threshold", obj.IRThreshold);
        end

        function response = state(obj)
            obj.wait();
            response = struct("ts", obj.millis(), "motors", obj.Motors(:), ...
                "watchdog", obj.Watchdog, "irThreshold", obj.IRThreshold);
        end

        function response = allSensors(obj)
            obj.wait();
            [ir, onLine] = obj.sample();
            response = struct("ts", obj.millis(), "distance", 150, ...
                "ir", ir, "onLine", onLine, "battery", 7.6);
        end

        function response = drive(obj, left, right)
            obj.wait();
            obj.Motors = [left right];
            response = struct("tank", [left; right]);
        end

        function response = stop(obj)
            obj.wait();
            obj.Motors = [0 0];
            response = struct("stopped", 1);
        end

        function response = setWatchdog(obj, timeoutMs)
            obj.wait();
            obj.Watchdog = timeoutMs;
            response = struct("watchdog", timeoutMs);
        end

        function response = ping(obj)
            obj.wait();
            response = struct("pong", 1);
        end

        function close(~)
        end
    end

    methods (Access = private)
        function [ir, onLine] = sample(obj)
            % Column vectors, like jsondecode of the firmware's [L,M,R] arrays.
            if all(obj.Surface == "track")
                blackFraction = obj.tapeUnderSensors();
                level = obj.LightLevel + blackFraction .* (obj.BlackLevel - obj.LightLevel);
            else
                levels = [obj.LightLevel; obj.BlackLevel; obj.LiftedLevel];
                [~, row] = ismember(obj.Surface, ["light" "black" "lifted"]);
                level = levels(sub2ind(size(levels), row, 1:3));
            end
            ir = level + obj.Noise * randn(1, 3);
            if ~isempty(obj.StuckSensor)
                ir(obj.StuckSensor) = obj.StuckValue;
            end
            ir = min(max(round(ir), 0), 1023).';
            onLine = double(ir > obj.IRThreshold);
        end

        function fraction = tapeUnderSensors(obj)
            % Fraction of each sensor's spot that the moving tape strip covers.
            t = toc(obj.Clock);
            tapeCenter = obj.SweepAmplitude * sin(2 * pi * t / obj.SweepPeriod);
            sensorX = [-1 0 1] * obj.SensorSpacing;    % left middle right
            overlap = min(sensorX + obj.SpotWidth / 2, tapeCenter + obj.TapeWidth / 2) - ...
                max(sensorX - obj.SpotWidth / 2, tapeCenter - obj.TapeWidth / 2);
            fraction = min(max(overlap / obj.SpotWidth, 0), 1);
        end

        function wait(obj)
            if obj.ReadDelay > 0
                pause(obj.ReadDelay);
            end
        end

        function ms = millis(obj)
            ms = round(1000 * toc(obj.Clock));
        end
    end
end
