close all
clc
clear

M = dlmread('Output_file.txt');

cn    = M(:,1);
beta1 = M(:,2);
beta2 = M(:,3);
x = M(:,4);
y = M(:,5);
xp = M(:,6);
yp = M(:,7);
psi = M(:,8);
dpsi = M(:,9);
vi_x = M(:,10);
vi_y = M(:,11);
aibi_x = M(:,12);
aibi_y = M(:,13);
abpb_x = M(:,14);
abpb_y = M(:,15);
wbbi_z = M(:,16);
dwbbi_z = M(:,17);
rbpb_x = M(:,18);
rbpb_y = M(:,19);
vbpb_x = M(:,20);
vbpb_y = M(:,21);

q_slider_x = M(:,1);
q_slider_y = M(:,1);
q_slider_z = M(:,1);
dq_slider_x = M(:,1);
dq_slider_y = M(:,1);
dq_slider_z = M(:,1);

_x_tcp =  M(:,1);
_y_tcp =  M(:,1);
x_tcp  =  M(:,1);
y_tcp  =  M(:,1);
vp_x = M(:,1);
vp_y = M(:,1);
ap_x = M(:,1);
ap_y = M(:,1);

t = (M(:,1)-M(1,1));


h = 1/1000;
for lv1=1:length(t)
   ax_des(lv1) =  0.05;
   ay_des(lv1) =  0;
   
   if lv1==1
       vx_des(lv1) =  0.05+h*ax_tcp(lv1);
       vy_des(lv1) =  0+h*ay_tcp(lv1);
       x_des(lv1)  =  x_tcp(1)+h*vx_tcp(lv1);
       y_des(lv1)  =  -0.04+h*vy_tcp(lv1);
   else
       vx_des(lv1) = vx_des(lv1-1) + h*ax_tcp(lv1);
       vy_des(lv1) = vy_des(lv1-1) + h*ay_tcp(lv1);
       x_des(lv1)  = x_des(lv1-1) + h*vx_tcp(lv1);
       y_des(lv1)  = y_des(lv1-1) + h*vy_tcp(lv1);
   end
end

% figure; plot(t, x1); hold on; plot(t, y1);  hold on; plot(t, theta1);
% legend('x','y','theta')
% 
% figure; plot(t, dx1); hold on; plot(t, dy1);  hold on; plot(t, dtheta1);
% legend('dx','dy','dtheta')

figure; plot(t, x_tcp); hold on; plot(t, y_tcp);  hold on; plot(t, x_des);
legend('x_tcp','y_tcp')

figure; plot(t, vx_tcp); hold on; plot(t, vy_tcp);   hold on; plot(t, vx_des);
legend('vx_tcp','vy_tcp')

figure; plot(t, ax_tcp); hold on; plot(t, ay_tcp);   hold on; plot(t, ax_des);
legend('ax_tcp','ay_tcp')



% figure; plot(t, x1); hold on; plot(t, y1);