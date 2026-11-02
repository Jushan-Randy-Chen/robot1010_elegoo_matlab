classdef ElegooRobotCar < handle
    % MATLAB client for the custom ELEGOO Smart Robot Car V4 firmware.
    % Arduino commands use TCP port 100; ESP32 HTTP endpoints use port 80.

    properties (SetAccess = private)
        Host string
        Port double
        HttpBase string
    end

    properties (Access = private)
        Tcp
    end

    methods
        function obj = ElegooRobotCar(host, port)
            if nargin < 2 || isempty(port)
                port = 100;
            end
            obj.Host = string(host);
            obj.Port = double(port);
            obj.HttpBase = "http://" + obj.Host;
            obj.Tcp = tcpclient(char(obj.Host), obj.Port, "Timeout", 3);
            configureTerminator(obj.Tcp, "LF");
        end

        function response = command(obj, number, d1, d2, d3)
            % Send one JSON command to the Arduino over the ESP32 TCP bridge.
            if nargin < 3, d1 = []; end
            if nargin < 4, d2 = []; end
            if nargin < 5, d3 = []; end

            payload = struct("N", double(number));
            if ~isempty(d1), payload.D1 = double(d1); end
            if ~isempty(d2), payload.D2 = double(d2); end
            if ~isempty(d3), payload.D3 = double(d3); end

            writeline(obj.Tcp, string(jsonencode(payload)));
            while true
                line = readline(obj.Tcp);
                if isempty(line)
                    error("ElegooRobotCar:Timeout", ...
                        "Timed out waiting for the Arduino response.");
                end
                line = strtrim(string(line));
                if line == "{Heartbeat}"
                    continue
                end
                try
                    response = jsondecode(line);
                    return
                catch
                    error("ElegooRobotCar:BadResponse", ...
                        "Unexpected response from Arduino: %s", line);
                end
            end
        end

        function response = ping(obj)
            response = obj.command(1);
        end

        function response = drive(obj, left, right)
            % Set left and right motor speeds in the range -255 to 255.
            validateattributes(left, {'numeric'}, {'scalar', 'integer', '>=', -255, '<=', 255});
            validateattributes(right, {'numeric'}, {'scalar', 'integer', '>=', -255, '<=', 255});
            response = obj.command(7, left, right);
        end

        function response = forward(obj, speed)
            if nargin < 2, speed = []; end
            response = obj.command(2, speed);
        end

        function response = backward(obj, speed)
            if nargin < 2, speed = []; end
            response = obj.command(3, speed);
        end

        function response = turnRight(obj, speed)
            if nargin < 2, speed = []; end
            response = obj.command(4, speed);
        end

        function response = turnLeft(obj, speed)
            if nargin < 2, speed = []; end
            response = obj.command(5, speed);
        end

        function response = stop(obj)
            response = obj.command(6);
        end

        function response = setDefaultSpeed(obj, speed)
            response = obj.command(8, speed);
        end

        function response = readDistance(obj)
            response = obj.command(10);
        end

        function response = readIR(obj)
            response = obj.command(11);
        end

        function response = readIMU(obj)
            response = obj.command(12);
        end

        function response = readBattery(obj)
            response = obj.command(13);
        end

        function response = calibrateBattery(obj, actualVoltage)
            response = obj.command(14, round(actualVoltage * 100));
        end

        function response = setMotorBias(obj, leftPercent, rightPercent)
            response = obj.command(15, leftPercent, rightPercent);
        end

        function response = setIRThreshold(obj, threshold)
            response = obj.command(16, threshold);
        end

        function response = setLedColor(obj, red, green, blue)
            response = obj.command(20, red, green, blue);
        end

        function response = setLedBrightness(obj, brightness)
            response = obj.command(21, brightness);
        end

        function response = pan(obj, angle)
            response = obj.command(30, angle);
        end

        function response = tilt(obj, angle)
            response = obj.command(31, angle);
        end

        function response = allSensors(obj)
            response = obj.command(100);
        end

        function response = state(obj)
            response = obj.command(101);
        end

        function response = setWatchdog(obj, timeoutMs)
            response = obj.command(102, timeoutMs);
        end

        function data = status(obj)
            data = webread(char(obj.HttpBase + "/status"));
        end

        function data = info(obj)
            data = webread(char(obj.HttpBase + "/info"));
        end

        function data = logs(obj)
            data = webread(char(obj.HttpBase + "/logs"));
        end

        function data = restartESP32(obj)
            data = webread(char(obj.HttpBase + "/restart"));
        end

        function frame = capture(obj)
            % Return one JPEG frame as an MATLAB image array.
            frame = webread(char(obj.HttpBase + "/capture"));
        end

        function url = streamURL(obj)
            url = obj.HttpBase + "/stream";
        end

        function result = setCamera(obj, variable, value)
            result = webread(char(obj.HttpBase + "/control"), ...
                "var", char(string(variable)), "val", double(value));
        end

        function close(obj)
            if ~isempty(obj.Tcp)
                delete(obj.Tcp);
                obj.Tcp = [];
            end
        end

        function delete(obj)
            obj.close();
        end
    end
end
