%  Configuration parameters
gifFilename = 'test.gif'; 
numFrames = 30;       % Number of frames to record
delayTime = 0.1;      % Time delay between frames (0.1 seconds = 10 FPS)

fprintf('Recording live stream to %s...\n', gifFilename);

for k = 1:numFrames
    % Capture a single frame from the stream
    img = robot.capture;

    % Convert the color image (RGB) to an Indexed image for the GIF format
    [imind, cm] = rgb2ind(img, 256); 

    % Write/Append to the GIF file
    if k == 1
        % Initialize the file and set it to loop infinitely
        imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
    else
        % Append the next frame to the existing file
        imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end

    fprintf('Saved frame %d/%d\n', k, numFrames);

    % Pause briefly to match the target frame rate
    pause(delayTime); 
end

disp('GIF recording successfully generated and saved!');
