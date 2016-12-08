load('Data.mat')
%% Initialize video
video = VideoReader('TrialExperiment.ogv');
frame_rate = video.FrameRate;
offset_in_seconds = -2.2;
%% Allocate space for the new video
video_width = video.Width;
video_height = video.Height;
scaling_factor = video_width / length(imOrig);
new_M = M * scaling_factor;
new_M(3,3) = 1;
% The trajectory in pixels is obtained, drawn and displayed
new_video = struct('cdata',zeros(video_height ,video_width ,3,'uint8'), 'colormap',[]);
%% Read and modify each of the frames. Store the modified frame in a new video file
k = 1;
video_time = offset_in_seconds;
N = length(Data.xMPC);
while hasFrame(video)
    im = undistortImage(readFrame(video), cameraParams);
    trajectory_index = find(Data.t > video_time, 1, 'first'); % The n*log(n) can be reduced to n, but it should go fast enough
    if video_time > Data.t(length(Data.t))
        trajectory_index = N;
    end
    x = 1000 * Data.xMPC{trajectory_index};
    y = 1000 * Data.yMPC{trajectory_index};
    trajectory = [x.' y.' ones(length(x), 1)];
    world_trajectory = RobotToWorldTrajectory(trajectory, 323.4, 100.2);
    camera_trajectory = WorldToCameraTrajectory(world_trajectory, new_M, planarR, translationVector);

    new_video(k).cdata = DrawTrajectoryOnImage(camera_trajectory(:,1).', camera_trajectory(:,2).', im);
    k = k+1;
    video_time = video_time + 1/frame_rate;
end
video_writer = VideoWriter('TrialExperimentDrawn.ogv');
video_writer.FrameRate = video.FrameRate;
open(video_writer);
writeVideo(video_writer, new_video);
close(video_writer);