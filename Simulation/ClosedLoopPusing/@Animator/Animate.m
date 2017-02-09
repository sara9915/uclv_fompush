function [] = Animate(obj, pusher_slider, file_name, flag)
% Function to animate the pusher_slider simulation motion
animation = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], ...
'PaperPosition', [0, 0, 11, (6/8)*11]);

acc_factor = 5;
set(gcf,'Renderer','OpenGL'); % TODO: Check if it actually does anything
set(gca,'FontSize',20) % TODO: Idem here %Set size of axis font
axis equal
%Create label
xlabel('x(m)', 'Interpreter', 'latex', 'FontSize', obj.font_size);
ylabel('y(m)', 'Interpreter', 'latex', 'FontSize', obj.font_size);
% zlabel('z(m)', 'Interpreter', 'latex', 'FontSize', 16);
% Thick_Array = [1;386;906;1366];
Thick_Array = [51;4451;10401;14051];
Thick_Array = [1];
x_state = pusher_slider.x_state{1};
%Either static (0) or dynamic animation (1)
if flag
    %Animation parameters
    tf = pusher_slider.t(end);
    N = length(pusher_slider.t);
    %Create movie file
    videoname = strcat(file_name,'.avi');
    v = VideoWriter(videoname);
    fps = double(int64(N/(acc_factor*tf)));
    v.FrameRate = fps; % TODO: Check why this double(int(
    open(v);
    xlim([-.1 x_state(end,1)+pusher_slider.a]);
    ylim([-0.1 0.2]); % TODO: Change the way of setting the limits
    %Go through mpc iterations
    for iteration = 1:acc_factor:length(pusher_slider.t)-35
        for lv1 = pusher_slider.starIndex:pusher_slider.NumSim+1
            Data{lv1} = pusher_slider.Data(iteration,lv1);
            if lv1==1
                if iteration==1
                    Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',0.1);
                    hold on 
                    Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
                else
                    Slider{lv1}.XData = Data{lv1}.x1rot;
                    Slider{lv1}.YData = Data{lv1}.y1rot;
                    Pusher{lv1}.XData = Data{lv1}.X;
                    Pusher{lv1}.YData = Data{lv1}.Y;
                end
            else
                if ~isempty(find(Thick_Array == iteration))
                    rx = -pusher_slider.a/2; 
                    Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3.0);
                    hold on 
                    Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
                else
                    Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', .2, 'FaceAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor', 'NONE', 'LineWidth', 0.1);
                    hold on 
                    Pusher{lv1} = patch(Data{lv1}.X, Data{lv1}.Y, 'red', 'EdgeAlpha', .2, 'FaceAlpha', .2, 'EdgeColor', [0,0,1] * 0.3, 'FaceColor', [1,0,0] * 0.5, 'LineWidth', 0.1); 
                end
            end
        end
        frame = getframe(animation);
        writeVideo(v,frame);
    end           
    close(v);
else
    xlim([-0.05 0.38]);
    ylim([-0.2 0.22]);
    for iteration=1:acc_factor:length(pusher_slider.t)%1366
         for lv1=2:pusher_slider.NumSim+1
            Data{lv1} = pusher_slider.Data(iteration,lv1);

            if ~isempty(find(Thick_Array == iteration))
                rx = -pusher_slider.a/2;
                Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot, 'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1] * 0.3,'FaceColor','NONE','LineWidth', 3.0);
                hold on 
                Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1] * 0.3, 'FaceColor', [1,0,0] * 0.5, 'LineWidth', 0.1);
            else
                Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot, 'red', 'EdgeAlpha', .2, 'FaceAlpha', .2,'EdgeColor', [0,0,1] * 0.3,'FaceColor', 'NONE', 'LineWidth', 0.1);
                hold on 
                Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', .2, 'FaceAlpha', .2, 'EdgeColor', [0,0,1] * 0.3, 'FaceColor', [1,0,0] * 0.5, 'LineWidth', 0.1); 
            end
         end
    end
    h = scatter([0.23 0.23 0.03], [-0.11 .11 0.08],100,'b','^','filled');
    legend(h,'Target');
%   title('Simulated Trajectory');
    saveas(animation, file_name, 'epsc');
end
end