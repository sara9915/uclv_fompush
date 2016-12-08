function obj = Plot(obj, Dataset)
    x_state = obj.x_state{Dataset+1};
    u_state = obj.u_state{Dataset+1};
    %Convert integer to strin
    Number = int2str(Dataset);
    %
    Name = strcat(obj.SimName, Number, 'SliderReponse');
    Figures.(Name)=Figure;
    Figures.(Name).filename = Name;  
    Figures.(Name).Create(3,1); 
    Figures.(Name).xData = {obj.t};
    Figures.(Name).yData = {x_state(:,1);x_state(:,2);x_state(:,3)};
    Figures.(Name).xLabel={'t(s)';'t(s)';'t(s)'};
    Figures.(Name).yLabel={'$x$ (m)';'$y$ (m)';'$\theta$ (rad)'};  
    Figures.(Name).Color = {'b','b','b'};
    Figures.(Name).Title = {Name};               
    Figures.(Name).Plot2d;
    Figures.(Name).Save(obj.FilePath);

    Name = strcat(obj.SimName, Number, 'PusherReponse');
    Figures.(Name)=Figure;
    Figures.(Name).filename = Name;  
    Figures.(Name).Create(2,1); 
    Figures.(Name).xData = {obj.t};
    Figures.(Name).yData = {x_state(:,4);x_state(:,5)};
    Figures.(Name).xLabel={'t(s)';'t(s)'};
    Figures.(Name).yLabel={'$x$ (m)';'$y$ (m)'};  
    Figures.(Name).Color = {'r','r'};
    Figures.(Name).Title = {Name};               
    Figures.(Name).Plot2d;
    Figures.(Name).Save(obj.FilePath);
    
    Name = strcat(obj.SimName, Number, 'ControlReponse');
    Figures.(Name)=Figure;
    Figures.(Name).filename = Name;  
    Figures.(Name).Create(2,1); 
    Figures.(Name).xData = {obj.t};
    Figures.(Name).yData = {u_state(:,1);[u_state(:,2)]};
    Figures.(Name).xLabel={'t(s)';'t(s)'};
    Figures.(Name).yLabel={'$v_{n}$ (m/s)';'$v_{t}$ (m/s)'};  
    Figures.(Name).Color = {'r',['r']};
    Figures.(Name).Title = {Name};               
    Figures.(Name).Plot2d;
    Figures.(Name).Save(obj.FilePath);

    %Set object figure property 
    obj.Figures = Figures;
end