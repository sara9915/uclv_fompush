function [world_trajectory] = RobotToWorldTrajectory(robot_trajectory, x_world_0, y_world_0)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
R = [0 -1; 1 0] * [-1 0; 0 1];
T = [x_world_0 ; y_world_0];
RT = [R T; 0 0 1];
IRT = inv(RT);
world_trajectory = zeros(length(robot_trajectory(:, 1)), 3); % Just to allocate space
for i = 1:length(robot_trajectory(:, 1))
    world_trajectory(i,:) = IRT * robot_trajectory(i, :).';
end
end

