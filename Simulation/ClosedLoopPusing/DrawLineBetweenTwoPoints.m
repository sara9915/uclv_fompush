function [new_im] = DrawLineBetweenTwoPoints(first, last, im)
%DrawLineBetweenTwoPoints. Fast function to change the pixel value of a
%certain image to draw a line segment between first and last point.
displacement_vector = last - first;
distance = sqrt(displacement_vector * displacement_vector.');
number_of_waypoints = ceil(distance);
step_displacement = displacement_vector / number_of_waypoints;
new_im = im;
for i = 0:number_of_waypoints
    waypoint = first + step_displacement * i;
    new_im(waypoint(1), waypoint(2)) = [255 0 0];
end
end

