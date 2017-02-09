function [drawn_image] = DrawTrajectoryOnImage(y_waypoints, x_waypoints, im, color)
%DrawTrajectoryOnImage Gets the x and y values of the trajectory waypoints
% and draws it on image. It returns the modified image. If not set, color
% will be automatically set to blue
if nargin < 4
    color = [0 0 255];
end
drawn_image = im;     % creates a merged image
N = length(x_waypoints);
for i = 1:N
    drawn_image(floor(x_waypoints(i)), floor(y_waypoints(i)),:) = color;
    drawn_image(floor(x_waypoints(i)), ceil(y_waypoints(i)),:) = color;
    drawn_image(ceil(x_waypoints(i)), floor(y_waypoints(i)),:) = color;
    drawn_image(ceil(x_waypoints(i)), ceil(y_waypoints(i)),:) = color;
end
end

