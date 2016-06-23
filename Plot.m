close all
clc
clear

M = dlmread('Output4.txt');

t = (M(:,1)-M(1,1));
dx1 = M(:,5);
dy1 = M(:,6);
dtheta1 = M(:,7);


figure; plot(t, dx1); hold on; plot(t, dy1);  hold on; plot(t, dtheta1);
legend('dx','dy','dtheta')