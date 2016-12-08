        %% Main Animation 
        function obj = Animate(obj,flag)

        font_size  = 25;
        line_size  = 15;
        line_width = 2;
        
        obj.Ani = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], ...
        'PaperPosition', [0, 0, 11, (6/8)*11]);
        set(gcf,'Renderer','OpenGL');
        set(gca,'FontSize',20) %Set size of axis font
        axis equal
        %Create label
        xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
        ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
%         zlabel('z (m)','FontSize', 16);
        %Animation parameters
        tf = obj.t(end);
        N = length(obj.t);
%         accFactor = 50;
        accFactor = 5;
        
%         Thick_Array = [1;386;906;1366];
        Thick_Array = [51;4451;10401;14051];
        Thick_Array = [1];
        
        x_state = obj.x_state{1};
        %Either static (0) or dynamic animation (1)
        if flag
                      
            x_state(1,end)
            xlim([-.1 x_state(end,1)+obj.a]);
            ylim([-0.1 0.1]);
            %Create movie file
            videoname = strcat(obj.FilePath,'/',(obj.SimName),'.avi');
            v = VideoWriter(videoname);
            fps = int64(N/(accFactor*tf));
            fps = double(fps);
            v.FrameRate = fps;
            open(v);

            %Go through iterations and plot each one
            for i1=1:accFactor:length(obj.t)-35
                for lv1=obj.starIndex:obj.NumSim+1
                    Data{lv1} = obj.Data(i1,lv1);
                    if lv1==1
                        if i1==1
                            Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',0.1);
                            hold on 
                            Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
%                             Line{lv1} = plot([0,x_state(end,1)],[0,0],'k');
%                             hold on 
%                             h(1) = scatter(0.23,-0.11,100,'b','^','filled');
%                             hold on;
%                             h(2) = scatter(0.23, 0.11,100,'b','^','filled');
%                             hold on;
%                             h(3) = scatter(0.03, 0.08,100,'b','^','filled');
%                             hold on;
%                             disp('*******')
                        else
                            Slider{lv1}.XData = Data{lv1}.x1rot;
                            Slider{lv1}.YData = Data{lv1}.y1rot;
                            Pusher{lv1}.XData = Data{lv1}.X;
                            Pusher{lv1}.YData = Data{lv1}.Y;
%                             Slider_thin{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'FaceAlpha', .2,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.1);
%                             hold on 
%                             Pusher_thin{lv1}=patch(Data{lv1}.X,Data{lv1}.Y,'red', 'FaceAlpha', .2,'EdgeAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1); 
                        end
                    else
                        if ~isempty(find(Thick_Array == i1))
                            rx = -obj.a/2;
%                             VelocityVector{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vipi(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vipi(2)],'Color', 'b','LineWidth', line_width); 
%                             MC1{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_b(2)],'Color', 'k','LineWidth', line_width); 
%                             MC2{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_b(2)],'Color', 'k','LineWidth', line_width); 
%                             MC1_linear{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_linear_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_linear_b(2)],'Color', 'r','LineWidth', line_width); 
%                             MC2_linear{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_linear_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_linear_b(2)],'Color', 'r','LineWidth', line_width); 
                            Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3.0);
                            hold on 
                            Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
%                             Line{lv1} = plot([0,x_state(end,1)],[0,0],'k','LineWidth', line_width);
%                             hold on 
%                             hold on 
%                             h(1) = scatter(0.23,-0.11,100,'b','^','filled');
%                             hold on;
%                             h(2) = scatter(0.23, 0.11,100,'b','^','filled');
%                             hold on;
%                             h(3) = scatter(0.03, 0.08,100,'b','^','filled');
%                             hold on;
                        else
                            Slider{lv1}.XData = Data{lv1}.x1rot;
                            Slider{lv1}.YData = Data{lv1}.y1rot;
                            Pusher{lv1}.XData = Data{lv1}.X;
                            Pusher{lv1}.YData = Data{lv1}.Y;
                            Slider_thin{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'FaceAlpha', .2,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.1);
                            hold on 
                            Pusher_thin{lv1}=patch(Data{lv1}.X,Data{lv1}.Y,'red', 'FaceAlpha', .2,'EdgeAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1); 
                            %Motion Cones
%                             VelocityVector{lv1}.XData = [Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vipi(1)];
%                             VelocityVector{lv1}.YData = [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vipi(2)];
%                             MC1{lv1}.XData = [Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_b(1)];
%                             MC1{lv1}.YData = [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_b(2)];
%                             MC2{lv1}.XData = [Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_b(1)];
%                             MC2{lv1}.YData =  [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_b(2)];%+Data{lv1}.d;
%                             MC1_linear{lv1}.XData = [Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_linear_b(1)];
%                             MC1_linear{lv1}.YData = [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_linear_b(2)];
%                             MC2_linear{lv1}.XData = [Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_linear_b(1)];
%                             MC2_linear{lv1}.YData = [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_linear_b(2)];
                        end
                    end
                end
                frame = getframe(obj.Ani);
                writeVideo(v,frame);
            end           
            close(v);
        else
            xlim([-0.05 0.38]);
            ylim([-0.2 0.22]);
            for i1=1:accFactor:length(obj.t)%1366
                lv1=1;
                x_state = obj.x_state{1};
                
                 for lv1=2:obj.NumSim+1
                    Data{lv1} = obj.Data(i1,lv1);
                    
                    if ~isempty(find(Thick_Array == i1))
                        rx = -obj.a/2;
        %                             VelocityVector{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vipi(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vipi(2)],'Color', 'b','LineWidth', line_width); 
        %                             MC1{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_b(2)],'Color', 'k','LineWidth', line_width); 
        %                             MC2{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_b(2)],'Color', 'k','LineWidth', line_width); 
        %                             MC1_linear{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vTop_linear_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vTop_linear_b(2)],'Color', 'r','LineWidth', line_width); 
        %                             MC2_linear{lv1} = plot([Data{lv1}.pos(1) Data{lv1}.pos(1)+Data{lv1}.vBottom_linear_b(1)], [Data{lv1}.pos(2) Data{lv1}.pos(2)+Data{lv1}.vBottom_linear_b(2)],'Color', 'r','LineWidth', line_width); 
                        Slider{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3.0);
                        hold on 
                        Pusher{lv1} = patch(Data{lv1}.X,Data{lv1}.Y,'red', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
%                         Line{lv1} = plot([0,0.5],[0,0],'k','LineWidth', 1);
%                         hold on 
                    else
    %                     Slider{lv1}.XData = Data{lv1}.x1rot;
    %                     Slider{lv1}.YData = Data{lv1}.y1rot;
    %                     Pusher{lv1}.XData = Data{lv1}.X;
    %                     Pusher{lv1}.YData = Data{lv1}.Y;
                        Slider_thin{lv1} = patch(Data{lv1}.x1rot, Data{lv1}.y1rot,'red', 'FaceAlpha', .2,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.1);
                        hold on 
                        Pusher_thin{lv1}=patch(Data{lv1}.X,Data{lv1}.Y,'red', 'FaceAlpha', .2,'EdgeAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1); 
                    end
                 end
            end
            
            scatter(0.23,-0.11,100,'b','^','filled')
            scatter(0.23, 0.11,100,'b','^','filled')
            h = scatter(0.03, 0.08,100,'b','^','filled')
            
            legend(h,'Target');
            
%             title('Simulated Trajectory');
            FileName = strcat(obj.FilePath,'/',(obj.SimName));
            saveas(obj.Ani,FileName,'epsc');
        end
        end
        