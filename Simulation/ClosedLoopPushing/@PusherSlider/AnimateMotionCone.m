%% Plot Motion Cone 
function obj = AnimateMotionCone(obj)

font_size  = 15;
line_size  = 15;
line_width = 2;

Ani = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], ...
'PaperPosition', [0, 0, 11, (6/8)*11]);
set(gcf,'Renderer','OpenGL');
set(gca,'FontSize',20) %Set size of axis font
axis equal
%Create label
xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);

%Sliding Object State
rx = -obj.a/2;
N = 100;
lim = abs(rx*1);
x_state = [0;0;0;rx;0];
accFactor=1;

xlim([-.1 0.3]);
ylim([-.2 .2]);
%Create movie file
videoname = strcat('MotionConeNonlinear','.avi');
v = VideoWriter(videoname);
fps = 10;
fps = double(fps);
v.FrameRate = fps;
open(v);
%Create patch and pusher plots
Data = obj.DataMC(x_state);
Slider  = patch(Data.x1rot, Data.y1rot,'red', 'FaceAlpha', 1,'EdgeAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3);
hold on 
Pusher  =patch(Data.X,Data.Y,'red', 'FaceAlpha', 1,'EdgeAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',1); 
MC1 = plot([0,1]+rx, [0,0],'Color', 'r','LineWidth', line_width); 
MC2 = plot([0,1]+rx, [0,0],'Color', 'b','LineWidth', line_width);
% MC1_linear = plot([0,1]+rx, [0,0],'Color', 'r','LineWidth', line_width); 
% MC2_linear = plot([0,1]+rx, [0,0],'Color', 'r','LineWidth', line_width);
%Go through iterations and plot each one
for counter = 1:N
    %Define gamma
    ry(counter) = -lim + (counter-1)*(lim*2/N);
    gamma_top(counter)     = (obj.nu_p*obj.c^2 - rx*ry(counter) + obj.nu_p*rx^2)/(obj.c^2 + ry(counter)^2 - obj.nu_p*rx*ry(counter));
    gamma_bottom(counter) = (-obj.nu_p*obj.c^2 - rx*ry(counter) - obj.nu_p*rx^2)/(obj.c^2 + ry(counter)^2 + obj.nu_p*rx*ry(counter));
%     gamma_top_linear(counter)    = obj.gammaTop_star    + obj.C_top_linear(4)*ry(counter);%(obj.nu_p*obj.c^2 - rx*ry(counter) + obj.nu_p*rx^2)/(obj.c^2 + ry(counter)^2 - obj.nu_p*rx*ry(counter));
%     gamma_bottom_linear(counter) = obj.gammaBottom_star + obj.C_bottom_linear(4)*ry(counter);%(-obj.nu_p*obj.c^2 - rx*ry(counter) - obj.nu_p*rx^2)/(obj.c^2 + ry(counter)^2 + obj.nu_p*rx*ry(counter));
    vnTop = 1;
    vtTop = gamma_top(counter);
%     vtTop_linear = gamma_top_linear(counter);
    vnBottom = 1;
    vtBottom = gamma_bottom(counter);
%     vtBottom_linear = gamma_bottom_linear(counter);
    %Update graph values
    Data = obj.DataMC([0,0,0,rx,ry(counter)]);
    Pusher.YData = Data.Y;
    MC1.YData = [0,gamma_top(counter)]+ry(counter);
    MC2.YData = [0,gamma_bottom(counter)]+ry(counter);
%     MC1_linear.YData = [0,gamma_top_linear(counter)]+ry(counter);
%     MC2_linear.YData = [0,gamma_bottom_linear(counter)]+ry(counter);
    %Save frames
    frame = getframe(Ani);
    writeVideo(v,frame);
end            
close(v);

Name = 'gammaNonlinear';
Figures.(Name)=Figure;
Figures.(Name).filename = Name;  
Figures.(Name).Create(1,1); 
Figures.(Name).xData = {ry'};
% Figures.(Name).yData = {[gamma_top', gamma_bottom',gamma_top_linear', gamma_bottom_linear']};
Figures.(Name).yData = {[gamma_top', gamma_bottom']};
Figures.(Name).xLabel={'$r_y$'};
Figures.(Name).yLabel={'$\gamma$ '};  
Figures.(Name).Color = {['r','b']};
% Figures.(Name).Color = {['r','b','r','b']};
Figures.(Name).Title = {Name};               
Figures.(Name).Plot2d;
Figures.(Name).LegendStr = {'$\gamma_t$','$\gamma_b$','$\gamma_{t,linear}$','$\gamma_{b,linear}$'};
Figures.(Name).addLegend ;
Figures.(Name).Save;

end