function obj = PlotMPC(obj, delta_u, delta_x)
    %Convert integer to strin
    t = linspace(0,obj.steps*obj.h_opt,obj.steps);
    %
    Number = 'LinearMPC';
    Name = strcat(obj.SimName, Number, 'SliderReponse');
    Figures.(Name)=Figure;
    Figures.(Name).filename = Name;  
    Figures.(Name).Create(4,1); 
    Figures.(Name).xData = {t};
    Figures.(Name).yData = {delta_x(:,1);delta_x(:,2);delta_x(:,3);delta_x(:,4)};
    Figures.(Name).xLabel={'t(s)';'t(s)';'t(s)';'t(s)'};
    Figures.(Name).yLabel={'$\bar{x}$ (m)';'$\bar{y}$ (m)';'$\bar{\theta}$ (rad)';'$\bar{p_y}$ (m)'};  
    Figures.(Name).Color = {'b','b','b'};
    Figures.(Name).Title = {Name};               
    Figures.(Name).Plot2d;
    Figures.(Name).Save;
% 
%     Name = strcat(obj.SimName, Number, 'PusherReponse');
%     Figures.(Name)=Figure;
%     Figures.(Name).filename = Name;  
%     Figures.(Name).Create(2,1); 
%     Figures.(Name).xData = {obj.t};
%     Figures.(Name).yData = {x_state(:,4);x_state(:,5)};
%     Figures.(Name).xLabel={'t(s)';'t(s)'};
%     Figures.(Name).yLabel={'$x$ (m)';'$y$ (m)'};  
%     Figures.(Name).Color = {'r','r'};
%     Figures.(Name).Title = {Name};               
%     Figures.(Name).Plot2d;
%     Figures.(Name).Save;
%     
    Name = strcat(obj.SimName, Number, 'ControlReponse');
    Figures.(Name)=Figure;
    Figures.(Name).filename = Name;  
    Figures.(Name).Create(2,1); 
    Figures.(Name).xData = {t};
    Figures.(Name).yData = {delta_u(:,1);delta_u(:,2)};
    Figures.(Name).xLabel={'t(s)';'t(s)'};
    Figures.(Name).yLabel={'$\bar{v}_{n}$ (m/s)';'$\bar{v}_{t}$ (m/s)'};  
    Figures.(Name).Color = {'r','r'};
    Figures.(Name).Title = {Name};               
    Figures.(Name).Plot2d;
    Figures.(Name).Save;
% 
%     %Set object figure property 
    obj.FiguresMPC = Figures;
end