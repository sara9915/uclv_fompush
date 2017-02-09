 %% Main Animation 
function obj = Animate(obj, file_name, xMPC, yMPC, thetaMPC, pyMPC)
set(gcf,'Renderer','OpenGL');
set(gca,'FontSize',20) %Set size of axis font
axis equal
%Create label
xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
%Animation parameters
tf = obj.t(end);
N = length(obj.t);
if strcmp(obj.ControllerType,'Trajectory')
    xlim([-0.05 0.6]);
    ylim([-0.1 0.1]);
else
    xlim([-0.05 0.6]);
    ylim([-0.2 0.2]);
end      

Thick_Array = [1,N-mod(N,acc_factor)+1];
x_state = obj.x_state{2};
%Either static (0) or dynamic animation (1)

%Create movie file
open(v);
%Go through iterations and plot each one
for i1=1:acc_factor:length(obj.t)
    lv1=2; % TODO: WTF?
        Data{lv1} = obj.Data(i1,lv1);
        if lv1==1
            if i1==1
                Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',0.1);
                hold on 
                Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
                MPC = plot(xMPC{i1}, yMPC{i1});
            else
                Slider{lv1}.XData = Data{lv1}.x1rot;
                Slider{lv1}.YData = Data{lv1}.y1rot;
                Pusher{lv1}.XData = Data{lv1}.X;
                Pusher{lv1}.YData = Data{lv1}.Y;
                MPC.XData = xMPC{i1};
                MPC.YData = yMPC{i1};
            end
        else
            if ~isempty(find(Thick_Array == i1))
                rx = -obj.a/2;
                Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3.0);
                hold on 
                Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
                hold on
                MPC = plot(xMPC{i1}, yMPC{i1},'b');
            else
                Slider_thin{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'FaceAlpha', .2,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.1);
                hold on 
                Pusher_thin{lv1}=patch(Data{lv1}.X,Data{lv1}.Y,'red', 'FaceAlpha', .2,'EdgeAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1); 
                MPC.XData = xMPC{i1};
                MPC.YData = yMPC{i1};
            end
        end
    frame = getframe(animation);
    writeVideo(v,frame);
end           
close(v);

end
        