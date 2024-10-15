figure, plot(p.u_control_vec), grid on, legend('u normale', 'u tangenziale')

figure(50)
ax1 = subplot(2,2,1);
plot(p.x_state_vec_sim(:,1)), grid on, title('x'), hold on, plot(p.x_state_vec(4:end,1)), plot(p.x_star_vect(1,:)), hold off, legend('sim','true')
ax2 = subplot(2,2,2);
plot(p.x_state_vec_sim(:,2)), grid on, title('y'), hold on, plot(p.x_state_vec(4:end,2)), plot(p.x_star_vect(2,:)), hold off, legend('sim','true')
ax3 = subplot(2,2,3);
plot(p.x_state_vec_sim(:,3)), grid on, title('theta'), hold on, plot(p.x_state_vec(4:end,3)), plot(p.x_star_vect(3,:)), hold off, legend('sim','true')
ax4 = subplot(2,2,4);
plot(p.x_state_vec(:,5)), grid on, title('ry') %hold on, plot(p.x_state_vec(4:end,5)), hold off, legend('sim','true')
linkaxes([ax1,ax2,ax3,ax4],'x');


figure(100)
ax101 = subplot(2,1,1);
plot(p.solutionVec), grid on, legend('sticking', 'sliding up', 'sliding down')
ax102 = subplot(2,1,2);
plot(p.index,'*'), grid on
linkaxes([ax101,ax102],'x');
%%

load("FOM_UPPER_7delay_2.mat");
robot_ = p;
load("FOM_UPPER_SIM_7delay_2.mat")
sim_ = p;

figure(50)
ax1 = subplot(2,2,1);
plot(robot_.t_vec, robot_.x_state_vec(:,1)), grid on, title('x'), hold on, plot(robot_.t_vec(1:end-1), robot_.x_star_vect(1,:)), plot(sim_.t, sim_.x_state{2}(:,1)), hold off, legend('robot','reference','sim')
ax2 = subplot(2,2,2);
plot(robot_.t_vec, robot_.x_state_vec(:,2)), grid on, title('y'), hold on, plot(robot_.t_vec(1:end-1), robot_.x_star_vect(2,:)), plot(sim_.t, sim_.x_state{2}(:,2)), hold off, legend('robot','reference','sim')
ax3 = subplot(2,2,3);
plot(robot_.t_vec, robot_.x_state_vec(:,3)), grid on, title('theta'), hold on, plot(robot_.t_vec(1:end-1), robot_.x_star_vect(3,:)), plot(sim_.t, sim_.x_state{2}(:,3)), hold off, legend('robot','reference','sim')
ax4 = subplot(2,2,4);
plot(p.x_state_vec(:,5)), grid on, title('ry') %hold on, plot(p.x_state_vec(4:end,5)), hold off, legend('sim','true')
linkaxes([ax1,ax2,ax3,ax4],'x');

