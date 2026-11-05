classdef ElegooSimCar < handle
    % Simulated ELEGOO Smart Robot Car V4 with the same methods as ElegooRobotCar.
    % The car drives in a furnished 5 m x 4 m living room / study (wood floor,
    % rug, bookshelf, desk with thin legs, sofa, TV console, wardrobe, door,
    % window with curtains, cardboard boxes). The camera view is rendered with
    % MATLAB graphics; the ultrasonic sensor is ray-cast against the furniture
    % footprints. Textures: simAssets/README.txt.
    %
    % World frame: x east, y north, z up (meters). Pose = [x y heading(rad)].
    %
    % Example:
    %   robot = ElegooSimCar();
    %   robot.drive(100, 100); pause(0.3); robot.stop();
    %   imshow(robot.capture());
    %   imshow(robot.snapshot([0.3 0.3 2.0], [3 2.5 0]))   % free camera view

    properties (SetAccess = private)
        Pose = [2.6 1.15 0]           % [x y theta] of the wheel-axle center (open middle of the room)
        TruePath = zeros(0, 3)        % pose history, one row per capture()
        RoomSize = [5.0 4.0]          % room width (x) and depth (y), m
    end

    properties
        ImageSize = [480 640]         % camera image [rows cols] (VGA)
        HorizontalFovDeg = 62         % camera horizontal field of view
        MaxWheelSpeed = 0.7           % wheel speed at PWM 255 on a full 7.4 V battery, m/s (estimate)
        MotorDeadband = 50            % PWM below this does not move the TT gear motors
        TrackWidth = 0.20             % EFFECTIVE track width, m. The V4 is a 4WD skid-steer car:
                                      % its wheels slip sideways when turning, so it turns
                                      % slower than a 0.14 m two-wheel robot would.
        CameraHeight = 0.12           % camera height above the floor, m
        CameraForward = 0.09          % camera position ahead of the axle, m
        SpeedNoise = 0.05             % random wheel-speed error (fraction)
        DefaultSpeed = 100
        Esp32Camera = true            % add JPEG artifacts, sensor noise and auto-exposure flicker
    end

    properties (Access = private)
        Fig
        Ax
        Segments                      % 2-D obstacle edges at sonar height [x1 y1 x2 y2]
        AssetDir
        LeftPwm = 0
        RightPwm = 0
        LastUpdate
        LastCommand
        WatchdogSec = Inf
        PanDeg = 90
        WarnedMissingAsset = false
    end

    methods
        function obj = ElegooSimCar(varargin)
            % Optional name-value pairs set properties, e.g. ElegooSimCar("Pose", [1 1 0]).
            % Demos that create the car themselves pick up
            % setappdata(groot, "ElegooSimCarOptions", {...}).
            if isappdata(groot, "ElegooSimCarOptions")
                varargin = [getappdata(groot, "ElegooSimCarOptions"), varargin];
            end
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k+1};
            end
            obj.AssetDir = fullfile(fileparts(mfilename("fullpath")), "simAssets");
            rng(1010, "twister");     % same room every run
            obj.buildRoom();
            obj.LastUpdate = tic;
            obj.LastCommand = tic;
        end

        % ---------------- Same API as ElegooRobotCar ----------------
        function response = drive(obj, left, right)
            obj.advance();
            obj.LeftPwm = max(min(round(left), 255), -255);
            obj.RightPwm = max(min(round(right), 255), -255);
            obj.LastCommand = tic;
            response = struct("ok", true);
        end

        function response = forward(obj, speed)
            if nargin < 2 || isempty(speed), speed = obj.DefaultSpeed; end
            response = obj.drive(speed, speed);
        end

        function response = backward(obj, speed)
            if nargin < 2 || isempty(speed), speed = obj.DefaultSpeed; end
            response = obj.drive(-speed, -speed);
        end

        function response = turnRight(obj, speed)
            if nargin < 2 || isempty(speed), speed = obj.DefaultSpeed; end
            response = obj.drive(speed, -speed);
        end

        function response = turnLeft(obj, speed)
            if nargin < 2 || isempty(speed), speed = obj.DefaultSpeed; end
            response = obj.drive(-speed, speed);
        end

        function response = stop(obj)
            response = obj.drive(0, 0);
        end

        function response = setDefaultSpeed(obj, speed)
            obj.DefaultSpeed = speed;
            response = struct("ok", true);
        end

        function response = setWatchdog(obj, timeoutMs)
            obj.WatchdogSec = timeoutMs / 1000;
            response = struct("ok", true);
        end

        function response = ping(~)
            response = struct("ok", true);
        end

        function response = pan(obj, angle)
            % Camera and sonar share the pan mount. 90 = straight ahead; in this
            % sim, larger angles look left (the real servo may be mirrored).
            obj.advance();
            obj.PanDeg = max(10, min(170, angle));
            response = struct("ok", true);
        end

        function response = readDistance(obj)
            % HC-SR04 range in whole cm, capped at 150 cm like the stock firmware.
            % The beam is roughly 15 degrees wide, so cast several rays. Sound
            % bounces away from a surface hit at more than ~45 degrees, so such a
            % surface gives no echo and the sensor reads "far" (150 cm). Thin
            % desk and chair legs can slip between the rays, as they do in reality.
            obj.advance();
            [origin, heading] = obj.sensorOrigin();
            ranges = 10 * ones(1, 7);
            angles = deg2rad(-7.5:2.5:7.5);
            for k = 1:numel(angles)
                [r, incidence] = obj.castRay(origin, heading + angles(k));
                if incidence < deg2rad(45)
                    ranges(k) = r;
                end
            end
            distanceCm = min(150, round(100 * min(ranges) + randn));
            response = struct("distance", max(distanceCm, 2));
        end

        function response = readIR(~)
            response = struct("onLine", [0 0 0]);   % the floor never ends
        end

        function response = readIMU(obj)
            obj.advance();
            [v, w] = obj.bodyVelocity();
            response = struct("gyro", [0 0 rad2deg(w)], "accel", [0 0 1], "speed", v);
        end

        function frame = capture(obj)
            % Render the car camera's view as an RGB uint8 image.
            obj.advance();
            [origin, heading] = obj.sensorOrigin();
            camPos = [origin obj.CameraHeight];
            frame = obj.render(camPos, camPos + [cos(heading) sin(heading) 0], obj.viewAngleDeg());
            if obj.Esp32Camera
                frame = ElegooSimCar.esp32Look(frame);
            end
            obj.TruePath(end+1, :) = obj.Pose;
        end

        function img = snapshot(obj, position, target, viewAngleDeg)
            % Clean render from any viewpoint (for figures and documentation).
            if nargin < 4, viewAngleDeg = 60; end
            img = obj.render(position, target, viewAngleDeg);
        end

        function pose = cameraPose(obj)
            % True camera [x y heading] (the camera sits ahead of the axle).
            [origin, heading] = obj.sensorOrigin();
            pose = [origin heading];
        end

        function intrinsics = cameraIntrinsics(obj)
            % Exact pinhole intrinsics of the simulated camera.
            rows = obj.ImageSize(1);
            cols = obj.ImageSize(2);
            f = (cols / 2) / tand(obj.HorizontalFovDeg / 2);
            intrinsics = cameraIntrinsics([f f], [cols rows] / 2 + 0.5, [rows cols]);
        end

        function showFloorPlan(obj, ax)
            % Top view of the obstacle outlines the sonar can hit.
            if nargin < 2, ax = gca; end
            s = obj.Segments;
            plot(ax, [s(:, 1) s(:, 3)]', [s(:, 2) s(:, 4)]', "k-", "LineWidth", 1.5);
            axis(ax, "equal");
            xlim(ax, [-0.1 obj.RoomSize(1) + 0.1]);
            ylim(ax, [-0.1 obj.RoomSize(2) + 0.1]);
        end

        function close(obj)
            if ~isempty(obj.Fig) && isvalid(obj.Fig)
                delete(obj.Fig);
            end
        end

        function delete(obj)
            obj.close();
        end
    end

    methods (Access = private)
        % ---------------- Motion and sensors ----------------
        function advance(obj)
            % Move the car with the current wheel PWM since the last update.
            dt = toc(obj.LastUpdate);
            obj.LastUpdate = tic;
            if toc(obj.LastCommand) > obj.WatchdogSec
                obj.LeftPwm = 0;      % firmware watchdog stops the motors
                obj.RightPwm = 0;
            end
            nSteps = max(1, ceil(dt / 0.01));
            for k = 1:nSteps
                [v, w] = obj.bodyVelocity();
                v = v * (1 + obj.SpeedNoise * randn);
                w = w * (1 + obj.SpeedNoise * randn);
                th = obj.Pose(3);
                next = obj.Pose + (dt / nSteps) * [v*cos(th) v*sin(th) w];
                if obj.isFree(next(1:2))
                    obj.Pose = next;
                else
                    obj.Pose(3) = next(3);    % bumped into something: only rotate
                end
            end
        end

        function [v, w] = bodyVelocity(obj)
            wheel = @(pwm) sign(pwm) * max(abs(pwm) - obj.MotorDeadband, 0) ...
                / (255 - obj.MotorDeadband) * obj.MaxWheelSpeed;
            vl = wheel(obj.LeftPwm);
            vr = wheel(obj.RightPwm);
            v = (vl + vr) / 2;
            w = (vr - vl) / obj.TrackWidth;
        end

        function [origin, heading] = sensorOrigin(obj)
            % The pan servo sits CameraForward ahead of the axle; the camera
            % and sonar turn with it.
            body = obj.Pose(3);
            origin = obj.Pose(1:2) + obj.CameraForward * [cos(body) sin(body)];
            heading = body + deg2rad(obj.PanDeg - 90);
        end

        function tf = isFree(obj, xy)
            % Keep the car body (radius ~10 cm) away from every edge.
            tf = all(obj.pointSegmentDistance(xy) > 0.10);
        end

        function d = pointSegmentDistance(obj, p)
            a = obj.Segments(:, 1:2);
            ab = obj.Segments(:, 3:4) - a;
            t = max(0, min(1, sum((p - a) .* ab, 2) ./ sum(ab.^2, 2)));
            d = vecnorm(a + t .* ab - p, 2, 2);
        end

        function [r, incidence] = castRay(obj, origin, angle)
            % Distance (m) from origin along angle to the closest segment, and the
            % angle between the ray and that segment's normal (rad).
            dir = [cos(angle) sin(angle)];
            a = obj.Segments(:, 1:2);
            e = obj.Segments(:, 3:4) - a;
            denom = dir(1) * e(:, 2) - dir(2) * e(:, 1);
            ao = a - origin;
            t = (ao(:, 1) .* e(:, 2) - ao(:, 2) .* e(:, 1)) ./ denom;  % along ray
            u = (ao(:, 1) * dir(2) - ao(:, 2) * dir(1)) ./ denom;      % along segment
            hit = find(abs(denom) > 1e-9 & t > 0 & u >= 0 & u <= 1);
            r = 10;
            incidence = 0;
            if ~isempty(hit)
                [r, i] = min(t(hit));
                edge = e(hit(i), :) / norm(e(hit(i), :));
                incidence = acos(min(1, abs(dir(1) * edge(2) - dir(2) * edge(1))));
            end
        end

        % ---------------- Rendering ----------------
        function frame = render(obj, camPos, camTarget, viewAngle)
            obj.Ax.CameraPosition = camPos;
            obj.Ax.CameraTarget = camTarget;
            obj.Ax.CameraUpVector = [0 0 1];
            obj.Ax.CameraViewAngle = viewAngle;
            % Render at 2x and shrink: this smooths texture aliasing the way a
            % real lens blurs fine detail, so features stay stable between frames.
            % Grab the figure, not the axes: on a Mac Retina screen getframe(axes)
            % gives only one zoomed corner of the view.
            frame = imresize(getframe(obj.Fig).cdata, obj.ImageSize, "Antialiasing", true);
        end

        function a = viewAngleDeg(obj)
            % MATLAB's CameraViewAngle spans the shorter image side (the height).
            rows = obj.ImageSize(1);
            cols = obj.ImageSize(2);
            f = (cols / 2) / tand(obj.HorizontalFovDeg / 2);
            a = 2 * atand((min(rows, cols) / 2) / f);
        end

        % ---------------- The room ----------------
        function buildRoom(obj)
            W = obj.RoomSize(1);
            D = obj.RoomSize(2);
            H = 2.4;                          % ceiling height
            obj.Segments = [0 0 W 0; W 0 W D; W D 0 D; 0 D 0 0];

            obj.Fig = figure("Visible", "off", "Color", [0.85 0.85 0.85], "Units", "pixels", ...
                "Position", [50 50 2*obj.ImageSize(2) 2*obj.ImageSize(1)], ...
                "MenuBar", "none", "ToolBar", "none", "HandleVisibility", "off");
            obj.Ax = axes(obj.Fig, "Units", "normalized", "Position", [0 0 1 1], ...
                "Color", [0.85 0.85 0.85]);
            hold(obj.Ax, "on");
            axis(obj.Ax, "off");

            % Textures (simAssets: AWS RoboMaker small house, MIT-0)
            woodFloor = obj.asset("floor_wood_planks");
            carpet = obj.asset("rug_carpet");
            brick = obj.asset("wall_white_brick");
            paint = obj.asset("wall_paint");
            woodDark = obj.asset("wood_dark");
            woodMedium = obj.asset("wood_medium");
            woodLight = obj.asset("wood_light");
            curtain = obj.asset("curtain");
            fabric = uint8(double(carpet) .* reshape([0.55 0.62 0.78], 1, 1, 3));
            metal = ElegooSimCar.solid([70 70 75], 6);
            black = ElegooSimCar.solid([25 25 28], 4);

            % ---- Floor, rug, walls, baseboards ----
            obj.quad([0 0 0; W 0 0; W D 0; 0 D 0], obj.tile(woodFloor, [D W], 0.62, true));
            obj.quad([1.9 1.5 0.004; 3.7 1.5 0.004; 3.7 2.5 0.004; 1.9 2.5 0.004], ...
                ElegooSimCar.withBorder(obj.tile(carpet, [1.0 1.8], 0.5), [110 70 60], 0.06, 400));
            obj.quad([W 0 0; 0 0 0; 0 0 H; W 0 H], obj.tile(paint, [H W], 1.0));        % south
            obj.quad([W D 0; W 0 0; W 0 H; W D H], obj.tile(paint, [H D], 1.0));        % east
            obj.quad([0 D 0; W D 0; W D H; 0 D H], obj.tile(brick, [H W], 1.1));        % north
            obj.quad([0 0 0; 0 D 0; 0 D H; 0 0 H], obj.tile(paint, [H D], 1.0));        % west
            board = obj.tile(woodDark, [0.08 1], 0.8);
            obj.quad([W 0.005 0; 3.25 0.005 0; 3.25 0.005 0.08; W 0.005 0.08], board);
            obj.quad([2.25 0.005 0; 0 0.005 0; 0 0.005 0.08; 2.25 0.005 0.08], board);
            obj.quad([W-0.005 D 0; W-0.005 0 0; W-0.005 0 0.08; W-0.005 D 0.08], board);
            obj.quad([0 D-0.005 0; W D-0.005 0; W D-0.005 0.08; 0 D-0.005 0.08], board);
            obj.quad([0.005 0 0; 0.005 D 0; 0.005 D 0.08; 0.005 0 0.08], board);
            outlet = ElegooSimCar.outletTexture();
            obj.quad([2.05 0.006 0.25; 1.98 0.006 0.25; 1.98 0.006 0.36; 2.05 0.006 0.36], outlet);
            obj.quad([W-0.006 3.35 0.25; W-0.006 3.28 0.25; W-0.006 3.28 0.36; W-0.006 3.35 0.36], outlet);
            obj.quad([2.6 D-0.006 0.25; 2.67 D-0.006 0.25; 2.67 D-0.006 0.36; 2.6 D-0.006 0.36], outlet);
            obj.quad([0.006 1.25 0.25; 0.006 1.32 0.25; 0.006 1.32 0.36; 0.006 1.25 0.36], outlet);

            % ---- South wall: wardrobe (west), door, low shoe cabinet (east) ----
            obj.box([0.7 0 1.9 0.6 0 2.0], struct("default", obj.tile(woodLight, [0.6 0.6], 0.6), ...
                "yp", ElegooSimCar.cabinetFront(obj.tile(woodLight, [2.0 1.2], 0.6), 2, 1, true, 400)), true);
            obj.quad([3.2 0.004 0; 2.3 0.004 0; 2.3 0.004 2.05; 3.2 0.004 2.05], ...
                ElegooSimCar.doorTexture(obj.tile(woodDark, [2.05 0.9], 0.45)));
            obj.box([3.5 0 4.4 0.4 0 0.55], struct("default", obj.tile(woodMedium, [0.4 0.4], 0.5), ...
                "yp", ElegooSimCar.cabinetFront(obj.tile(woodMedium, [0.55 0.9], 0.5), 1, 3, false, 400)), true);

            % ---- East wall: TV console + TV, window with curtains ----
            obj.box([4.65 1.0 W 2.6 0 0.5], struct("default", obj.tile(woodMedium, [0.5 0.5], 0.5), ...
                "xm", ElegooSimCar.cabinetFront(obj.tile(woodMedium, [0.5 1.6], 0.5), 2, 3, false, 400), ...
                "top", obj.tile(woodDark, [0.35 1.6], 0.5)), true);
            obj.box([4.86 1.75 4.92 1.85 0.5 0.6], struct("default", black), false);   % TV stand
            obj.box([4.82 1.2 4.88 2.4 0.6 1.28], struct("default", black, ...
                "xm", ElegooSimCar.screenTexture()), false);
            obj.quad([W-0.004 3.75 0.9; W-0.004 2.75 0.9; W-0.004 2.75 2.1; W-0.004 3.75 2.1], ...
                ElegooSimCar.windowTexture());
            obj.quad([W-0.012 2.75 0.03; W-0.012 2.45 0.03; W-0.012 2.45 2.3; W-0.012 2.75 2.3], ...
                obj.tile(curtain, [2.27 0.3], 0.3));
            obj.quad([W-0.012 4.0 0.03; W-0.012 3.7 0.03; W-0.012 3.7 2.3; W-0.012 4.0 2.3], ...
                obj.tile(curtain, [2.27 0.3], 0.3));

            % ---- North wall: desk with thin legs, PC, chair (east), bookshelf (west) ----
            obj.box([3.3 3.4 4.6 D 0.72 0.75], struct("default", obj.tile(woodMedium, [0.6 1.3], 0.6)), false);
            for p = [3.32 3.42; 4.54 3.42; 3.32 3.94; 4.54 3.94]'
                obj.box([p(1) p(2) p(1)+0.04 p(2)+0.04 0 0.72], struct("default", metal), true);
            end
            obj.box([3.95 3.45 4.15 3.92 0 0.42], struct("default", black, ...
                "ym", ElegooSimCar.pcFront()), true);
            for p = [3.5 3.47; 3.87 3.47; 3.5 3.82; 3.87 3.82]'
                obj.box([p(1) p(2) p(1)+0.03 p(2)+0.03 0 0.44], struct("default", metal), true);
            end
            obj.box([3.48 3.45 3.92 3.87 0.44 0.48], struct("default", fabric), false);
            obj.box([3.48 3.83 3.92 3.87 0.48 0.9], struct("default", fabric), false);
            obj.box([0.9 3.68 2.3 D 0 1.9], struct("default", obj.tile(woodDark, [0.32 0.32], 0.5), ...
                "ym", ElegooSimCar.bookshelfFront(obj.tile(woodDark, [1.9 1.4], 0.5), [1.9 1.4], 400)), true);
            obj.quad([3.55 D-0.006 1.25; 4.4 D-0.006 1.25; 4.4 D-0.006 1.62; 3.55 D-0.006 1.62], ...
                ElegooSimCar.framed(obj.asset("picture_stonehenge")));

            % ---- West wall: sofa, pictures ----
            obj.box([0.12 1.9 0.85 3.4 0.08 0.42], struct("default", fabric, ...
                "xp", ElegooSimCar.seams(obj.tile(fabric, [0.34 1.5], 0.4), 3)), true);
            obj.box([0 1.9 0.25 3.4 0.08 0.86], struct("default", fabric), false);
            obj.box([0 1.9 0.85 2.08 0.08 0.62], struct("default", fabric), false);
            obj.box([0 3.22 0.85 3.4 0.08 0.62], struct("default", fabric), false);
            for p = [0.78 1.93; 0.78 3.33]'
                obj.box([p(1) p(2) p(1)+0.05 p(2)+0.05 0 0.08], struct("default", black), false);
            end
            obj.Segments(end+1:end+4, :) = ElegooSimCar.rectSegments([0 1.9 0.85 3.4]);
            obj.quad([0.006 2.0 1.0; 0.006 2.6 1.0; 0.006 2.6 1.42; 0.006 2.0 1.42], ...
                ElegooSimCar.framed(obj.asset("picture_lake")));
            obj.quad([0.006 2.8 1.0; 0.006 3.25 1.0; 0.006 3.25 1.42; 0.006 2.8 1.42], ...
                ElegooSimCar.framed(obj.asset("picture_meerkat")));

            % ---- South-west corner: cardboard boxes ----
            obj.box([0.08 0.08 0.55 0.5 0 0.4], ElegooSimCar.cardboard("FRAGILE"), true);
            obj.box([0.13 0.12 0.47 0.44 0.4 0.62], ElegooSimCar.cardboard("THIS SIDE UP"), false);

            % ---- Everyday clutter near the floor (what a low camera mostly sees) ----
            pattern = ElegooSimCar.matlabImage("fabric.png");
            obj.box([0.25 2.2 0.4 2.6 0.42 0.78], struct("default", pattern), false);       % cushions
            obj.box([0.25 2.7 0.4 3.1 0.42 0.78], struct("default", fliplr(pattern)), false);
            obj.box([0.1 1.5 0.55 1.85 0 0.5], struct("default", obj.tile(woodDark, [0.5 0.45], 0.4), ...
                "xp", ElegooSimCar.cabinetFront(obj.tile(woodDark, [0.5 0.35], 0.4), 2, 1, false, 400)), true);
            obj.box([0.15 1.55 0.45 1.8 0.5 0.56], ElegooSimCar.bookStack(), false);            % magazines
            obj.box([4.55 3.0 4.85 3.3 0 0.3], struct("default", ElegooSimCar.solid([170 90 60], 8)), true);
            obj.box([4.5 2.95 4.9 3.35 0.3 0.95], struct("default", ElegooSimCar.foliage()), false);
            obj.box([2.4 3.5 2.72 3.72 0 0.42], ElegooSimCar.backpack(), true);
            obj.box([4.5 0.45 4.78 0.73 0 0.38], ElegooSimCar.bin(), true);
            obj.box([2.45 0.06 2.58 0.34 0 0.1], struct("default", ElegooSimCar.solid([235 235 235], 10), ...
                "top", ElegooSimCar.solid([40 40 45], 6)), true);                           % sneakers
            obj.box([2.62 0.06 2.75 0.34 0 0.1], struct("default", ElegooSimCar.solid([235 235 235], 10), ...
                "top", ElegooSimCar.solid([40 40 45], 6)), true);

            % ---- Lights (two ceiling lamps) ----
            light(obj.Ax, "Position", [1.6 2.0 2.3], "Style", "local", "Color", [0.55 0.55 0.52]);
            light(obj.Ax, "Position", [3.6 2.0 2.3], "Style", "local", "Color", [0.55 0.55 0.52]);

            obj.Ax.XLim = [-0.1 W+0.1];
            obj.Ax.YLim = [-0.1 D+0.1];
            obj.Ax.ZLim = [-0.1 H+0.1];
            obj.Ax.DataAspectRatio = [1 1 1];
            obj.Ax.Projection = "perspective";
            obj.Ax.CameraViewAngle = obj.viewAngleDeg();
        end

        function quad(obj, corners, tex)
            % Textured rectangle. corners: 4x3, bottom-left, bottom-right,
            % top-right, top-left as seen from the side that faces the room.
            X = [corners(1,1) corners(2,1); corners(4,1) corners(3,1)];
            Y = [corners(1,2) corners(2,2); corners(4,2) corners(3,2)];
            Z = [corners(1,3) corners(2,3); corners(4,3) corners(3,3)];
            surface(obj.Ax, X, Y, Z, flipud(tex), "FaceColor", "texturemap", ...
                "EdgeColor", "none", "CDataMapping", "direct", "FaceLighting", "gouraud", ...
                "AmbientStrength", 0.62, "DiffuseStrength", 0.45, "SpecularStrength", 0.05);
        end

        function box(obj, b, tex, blocksSonar)
            % Box b = [x0 y0 x1 y1 z0 z1]. tex has optional fields xm, xp, ym, yp,
            % top (one image per face) and default.
            x0 = b(1); y0 = b(2); x1 = b(3); y1 = b(4); z0 = b(5); z1 = b(6);
            pick = @(f) ElegooSimCar.field(tex, f);
            obj.quad([x0 y0 z0; x1 y0 z0; x1 y0 z1; x0 y0 z1], pick("ym"));   % faces -y
            obj.quad([x1 y1 z0; x0 y1 z0; x0 y1 z1; x1 y1 z1], pick("yp"));   % faces +y
            obj.quad([x0 y1 z0; x0 y0 z0; x0 y0 z1; x0 y1 z1], pick("xm"));   % faces -x
            obj.quad([x1 y0 z0; x1 y1 z0; x1 y1 z1; x1 y0 z1], pick("xp"));   % faces +x
            obj.quad([x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1], pick("top"));
            if blocksSonar
                obj.Segments(end+1:end+4, :) = ElegooSimCar.rectSegments(b(1:4));
            end
        end

        function img = asset(obj, name)
            file = fullfile(obj.AssetDir, name + ".jpg");
            if isfile(file)
                img = imread(file);
            else
                if ~obj.WarnedMissingAsset      % One warning, not one for each file
                    warning("ElegooSimCar:MissingAsset", ...
                        "Texture files are missing in %s. The room uses plain gray. Camera tasks can fail.", ...
                        obj.AssetDir);
                    obj.WarnedMissingAsset = true;
                end
                img = ElegooSimCar.solid([170 170 170], 10);
            end
        end

        function out = tile(~, src, sizeM, tileWidthM, shadeTiles)
            % Cover a sizeM = [h w] surface with copies of src, each tileWidthM
            % wide, at 400 pixels per meter (capped for very large surfaces).
            % Copies are randomly mirrored and shifted (and, for floor planks,
            % shaded): exact repeats would confuse feature matching far more
            % than a real floor or wall does.
            if nargin < 5, shadeTiles = false; end
            pxPerM = min(400, 2400 / max(sizeM));
            tilePx = max(8, round(tileWidthM * pxPerM * [size(src, 1) / size(src, 2), 1]));
            t = imresize(src, tilePx);
            outPx = max(8, round(sizeM * pxPerM));
            reps = ceil(outPx ./ tilePx) + [0 1];
            out = zeros([reps .* tilePx 3], "like", t);
            for i = 1:reps(1)
                rowTiles = zeros([tilePx(1) reps(2) * tilePx(2) 3], "like", t);
                for j = 1:reps(2)
                    c = t;
                    if rand < 0.5, c = fliplr(c); end
                    if rand < 0.5, c = flipud(c); end
                    if shadeTiles
                        c = uint8(double(c) * (0.93 + 0.14 * rand));
                    end
                    rowTiles(:, (j-1)*tilePx(2)+1:j*tilePx(2), :) = c;
                end
                out((i-1)*tilePx(1)+1:i*tilePx(1), :, :) = circshift(rowTiles, randi(tilePx(2)), 2);
            end
            out = out(1:outPx(1), 1:outPx(2), :);
        end
    end

    methods (Static, Access = private)
        function img = esp32Look(img)
            % OV2640 over the ESP32: brightness jumps from auto exposure, sensor
            % noise, and strong JPEG compression (quality ~12).
            img = double(img) * (1 + 0.08 * randn) + 3 * randn(size(img));
            file = [tempname '.jpg'];
            imwrite(uint8(img), file, "Quality", 12);
            img = imread(file);
            delete(file);
        end

        function tex = field(s, name)
            if isfield(s, name)
                tex = s.(name);
            else
                tex = s.default;
            end
        end

        function seg = rectSegments(r)
            % Four edges of the footprint r = [x0 y0 x1 y1].
            seg = [r(1) r(2) r(3) r(2); r(3) r(2) r(3) r(4); r(3) r(4) r(1) r(4); r(1) r(4) r(1) r(2)];
        end

        function img = solid(rgb, noise)
            img = uint8(double(reshape(rgb, 1, 1, 3)) + noise * randn(64, 64, 3));
        end

        function img = withBorder(img, rgb, borderM, pxPerM)
            b = max(2, round(borderM * pxPerM));
            c = reshape(uint8(rgb), 1, 1, 3);
            img([1:b end-b+1:end], :, :) = repmat(c, 2*b, size(img, 2));
            img(:, [1:b end-b+1:end], :) = repmat(c, size(img, 1), 2*b);
        end

        function img = seams(img, n)
            % Dark vertical seams between n cushions.
            for k = 1:n-1
                c = round(k * size(img, 2) / n);
                img(:, max(1, c-2):c+2, :) = img(:, max(1, c-2):c+2, :) * 0.55;
            end
        end

        function img = cabinetFront(img, rows, cols, tall, pxPerM)
            % Door/drawer seams, small handles and a dark plinth.
            [h, w, ~] = size(img);
            img = double(img);
            plinth = min(round(0.07 * pxPerM), round(0.15 * h));
            img(end-plinth+1:end, :, :) = img(end-plinth+1:end, :, :) * 0.35;
            body = h - plinth;
            for r = 1:rows-1
                y = round(r * body / rows);
                img(max(1, y-1):y+1, :, :) = img(max(1, y-1):y+1, :, :) * 0.45;
            end
            for c = 1:cols-1
                x = round(c * w / cols);
                img(1:body, max(1, x-1):x+1, :) = img(1:body, max(1, x-1):x+1, :) * 0.45;
            end
            for r = 1:rows
                for c = 1:cols
                    if tall
                        cy = round((r - 0.5) * body / rows);
                        cx = round(c * w / cols - 0.12 * w / cols);
                        hh = round(0.12 * body / rows); hw = 3;
                    else
                        cy = round((r - 0.35) * body / rows);
                        cx = round((c - 0.5) * w / cols);
                        hh = 3; hw = round(0.15 * w / cols);
                    end
                    img(max(1, cy-hh):min(h, cy+hh), max(1, cx-hw):min(w, cx+hw), :) = 200;
                end
            end
            img = uint8(img);
        end

        function img = doorTexture(img)
            % Four recessed panels and a metal handle.
            [h, w, ~] = size(img);
            img = double(img);
            panels = [0.08 0.45 0.12 0.44; 0.08 0.45 0.56 0.88; 0.55 0.92 0.12 0.44; 0.55 0.92 0.56 0.88];
            for p = panels'
                r = round(p(1)*h):round(p(2)*h);
                c = round(p(3)*w):round(p(4)*w);
                img(r, c, :) = img(r, c, :) * 0.88;
                img([r(1) r(end)], c, :) = img([r(1) r(end)], c, :) * 0.6;
                img(r, [c(1) c(end)], :) = img(r, [c(1) c(end)], :) * 0.6;
            end
            img(round(0.5*h):round(0.53*h), round(0.06*w):round(0.16*w), :) = 190;
            img = uint8(img);
        end

        function img = windowTexture()
            % Bright sky with a white frame and a cross bar.
            h = 480; w = 400;
            g = linspace(0, 1, h)';
            sky = cat(3, 170 + 60*g, 200 + 40*g, 235 + 15*g) .* ones(1, w);
            sky(:, :, :) = sky + 4 * randn(h, w, 3);
            f = 18;
            sky([1:f end-f+1:end round(h/2)-f/2:round(h/2)+f/2], :, :) = 245;
            sky(:, [1:f end-f+1:end round(w/2)-f/2:round(w/2)+f/2], :) = 245;
            img = uint8(sky);
        end

        function img = screenTexture()
            % Dark TV screen with a faint reflection and a thin bezel.
            h = 340; w = 600;
            [x, y] = meshgrid(linspace(0, 1, w), linspace(0, 1, h));
            v = 18 + 30 * exp(-((x - 0.3).^2 / 0.02 + (y - 0.3).^2 / 0.05));
            img = uint8(repmat(v, 1, 1, 3) + 2 * randn(h, w, 3));
            img([1:8 end-7:end], :, :) = 8;
            img(:, [1:8 end-7:end], :) = 8;
        end

        function img = pcFront()
            % PC tower front: vents, drive bay, power button and LED.
            h = 420; w = 470;
            img = double(ElegooSimCar.solid([28 28 30], 3));
            img = imresize(img, [h w]);
            for y = round(0.45*h):12:round(0.9*h)
                img(y:y+4, round(0.15*w):round(0.85*w), :) = 12;
            end
            img(round(0.12*h):round(0.22*h), round(0.1*w):round(0.9*w), :) = 55;
            img(round(0.3*h):round(0.36*h), round(0.45*w):round(0.55*w), :) = 150;
            img(round(0.3*h):round(0.33*h), round(0.65*w):round(0.69*w), 3) = 255;
            img = uint8(img);
        end

        function img = matlabImage(name)
            % A sample image that ships with MATLAB toolboxes (gray if missing).
            if isempty(which(name))
                img = ElegooSimCar.solid([150 150 150], 10);
                return
            end
            img = imread(name);
            if size(img, 3) == 1, img = repmat(img, 1, 1, 3); end
        end

        function img = foliage()
            % Leaves: many overlapping green ellipses.
            img = repmat(uint8(reshape([30 60 30], 1, 1, 3)), 400, 300);
            for k = 1:220
                g = uint8([20 + 60*rand, 90 + 110*rand, 20 + 50*rand]);
                img = insertShape(img, "filled-circle", [randi(300) randi(400) randi([10 30])], ...
                    ShapeColor = g, Opacity = 1);
            end
            img = imgaussfilt(img, 1);
        end

        function tex = backpack()
            % Dark red backpack: zipper, front pocket, small logo.
            front = ElegooSimCar.solid([120 25 30], 8);
            front = imresize(front, [420 320]);
            front(90:96, 30:290, :) = 25;                                   % zipper
            front(220:380, 50:270, :) = front(220:380, 50:270, :) * 0.75;   % pocket
            front(226:232, 60:260, :) = 25;
            front = insertText(front, [120 300], "TREK", FontSize = 28, BoxOpacity = 0, ...
                TextColor = [230 230 230]);
            tex = struct("default", imresize(ElegooSimCar.solid([100 20 25], 8), [420 220]), ...
                "ym", front, "xp", front, "xm", front);
        end

        function tex = bin()
            % Gray trash bin with a recycling label.
            side = imresize(ElegooSimCar.solid([140 145 150], 6), [380 280]);
            side(30:40, :, :) = 90;
            side = insertText(side, [70 170], "RECYCLE", FontSize = 30, BoxColor = [30 120 60], ...
                BoxOpacity = 1, TextColor = "white");
            tex = struct("default", side, "top", ElegooSimCar.solid([40 40 40], 4));
        end

        function tex = bookStack()
            % A few magazines lying on the side table.
            d = fullfile(toolboxdir("vision"), "visiondata", "bookCovers");
            files = dir(fullfile(d, "*.jpg"));
            if isempty(files)
                top = ElegooSimCar.solid([200 60 60], 10);
            else
                top = imresize(imread(fullfile(d, files(5).name)), [300 360]);
            end
            edge = repmat(uint8(reshape([230 225 210], 1, 1, 3)), 60, 300);
            edge(10:3:end, :, :) = 150;
            tex = struct("default", edge, "top", top);
        end

        function img = outletTexture()
            img = repmat(uint8(235), 110, 70, 3);
            img([1:3 end-2:end], :, :) = 190;
            img(:, [1:3 end-2:end], :) = 190;
            img(28:42, 24:28, :) = 40; img(28:42, 42:46, :) = 40;
            img(68:82, 24:28, :) = 40; img(68:82, 42:46, :) = 40;
        end

        function img = framed(picture)
            % Picture with a dark wood frame and a white mat.
            p = imresize(picture, [300 NaN]);
            p = padarray(p, [24 24], 240, "both");
            img = padarray(p, [18 18], 60, "both");
        end

        function tex = cardboard(label)
            % Brown cardboard with packing tape and a printed label on each side.
            base = @(h, w) uint8(double(reshape([172 132 88], 1, 1, 3)) + 7 * randn(h, w, 3));
            front = base(400, 470);
            front(:, 220:250, :) = 205;                                     % tape
            front = insertText(front, [40 260], label, FontSize = 40, ...
                BoxColor = "white", BoxOpacity = 1, TextColor = "black");
            front = insertShape(front, "filled-polygon", [380 120 420 60 460 120], ShapeColor = "black");
            side = base(400, 420);
            side = insertText(side, [30 120], "No. 12-B", FontSize = 34, BoxOpacity = 0, TextColor = [60 40 20]);
            top = base(420, 470);
            top(:, 220:250, :) = 205;
            tex = struct("default", side, "xp", front, "yp", front, "top", top);
        end

        function img = bookshelfFront(frame, sizeM, pxPerM)
            % Shelves full of books: spines cut from the book-cover photos that
            % ship with the Computer Vision Toolbox.
            h = round(sizeM(1) * pxPerM); w = round(sizeM(2) * pxPerM);
            img = imresize(frame, [h w]);
            covers = dir(fullfile(toolboxdir("vision"), "visiondata", "bookCovers", "*.jpg"));
            board = round(0.025 * pxPerM);
            side = round(0.03 * pxPerM);
            nShelves = 5;
            shelfH = floor((h - board) / nShelves);
            for s = 1:nShelves
                top = (s - 1) * shelfH + board + 1;
                bottom = s * shelfH;
                img(top:bottom, side+1:w-side, :) = 35;          % dark inside the shelf
                x = side + 1;
                while x < w - side - 10
                    bw = randi(round([0.018 0.05] * pxPerM));
                    bh = randi(round([0.6 0.92] * (bottom - top)));
                    bw = min(bw, w - side - x);
                    if rand < 0.08                                % small gap
                        x = x + bw;
                        continue
                    end
                    if isempty(covers)
                        spine = repmat(uint8(255 * rand(1, 1, 3)), bh, bw);
                    else
                        c = imread(fullfile(covers(randi(numel(covers))).folder, ...
                            covers(randi(numel(covers))).name));
                        cx = randi([round(0.2 * size(c, 2)) round(0.8 * size(c, 2))]);
                        strip = c(round(0.1*size(c,1)):round(0.9*size(c,1)), cx:min(end, cx+40), :);
                        spine = imresize(strip, [bh bw]);
                    end
                    shade = linspace(0.7, 1, bw);
                    spine = uint8(double(spine) .* min(shade, fliplr(shade)) / max(shade));
                    img(bottom-bh+1:bottom, x:x+bw-1, :) = spine;
                    x = x + bw;
                end
            end
        end
    end
end
