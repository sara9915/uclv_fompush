function [ camera_trajectory ] = WorldToCameraTrajectory(world_trajectory, M, R, T)
%WorldToCameraTrajectory Gets a world trajectory and the calibration
%matrix and returns the trajectory in the camera frame. It's not in the
%Matlab frame (which has (0,0) in the upper left corner and the y axis
%flipped. R should be passed as rotation matrix, not pitch, roll, yaw. T is
%the translation vector as column vector
W = zeros(3);
W(1:3, 1:2) = R; % If the calibrator gives the inverse, replace by R.', should be symmetric, though
W(1:3, 3) = T; % If the calibrator gives the inverse, replace by R.' - R.' * T
MW = M * W;
camera_trajectory = zeros(length(world_trajectory(:, 1)), 3); % Just to allocate space
for i = 1:length(world_trajectory(:, 1))
    projective_space_value = MW * world_trajectory(i, :).';
    if projective_space_value(3) ~= 0
        camera_trajectory(i,:) = [(projective_space_value(1:2) / projective_space_value(3)).' 1];
    else
        camera_trajectory(i,:) = [0 0 1];
    end
    % Still requires changing from camera frame to Matlab frame
    % Still requires undistortion
end
end

