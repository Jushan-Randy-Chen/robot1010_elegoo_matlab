% Set up the VideoWriter object for MP4 format
videoFilename = 'test.mp4';
% MPEG-4 format ensures standard MP4 compression playable on any Mac/PC
vidObj = VideoWriter(videoFilename, 'MPEG-4'); 

% Configure video properties
vidObj.FrameRate = 10; % Match your target capture frame rate (e.g., 10 FPS)
vidObj.Quality = 95;   % Video compression quality level (1 to 100)

% Open the video file stream for writing data
open(vidObj);
fprintf('Recording video to %s...\n', videoFilename);

% Loop to pull stream chunks and encode them into the file
for k = 1:length(img)

    % Write the uncompressed matrix directly into the MP4 file container
    writeVideo(vidObj, img{k});

end

% Safely finalize and close the files
close(vidObj);
