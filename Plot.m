close all
clc
clear

M = dlmread('Output4.txt');

t = (M(:,1)-M(1,1));
x1 = M(:,2);
y1 = M(:,3);
theta1 = M(:,4);
dx1 = M(:,5);
dy1 = M(:,6);
dtheta1 = M(:,7);

figure; plot(t, x1); hold on; plot(t, y1);  hold on; plot(t, theta1);

figure; plot(t, dx1); hold on; plot(t, dy1);  hold on; plot(t, dtheta1);
legend('dx','dy','dtheta')