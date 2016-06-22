close all
clc
clear

M = dlmread('Output4.txt');

t = (M(:,1)-M(1,1));
sent = M(:,2);
actual = M(:,3);

figure; plot(t, sent); hold on; plot(t, actual);