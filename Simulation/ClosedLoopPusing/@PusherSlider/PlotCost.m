%%Controller Design (In world frame)
function obj = PlotCost(obj)

    Cost = obj.Cost;
    State = obj.StateCost;
    counter = 1;
    N1=20;
    for lv1=1:N1
        for lv2=1:N1
            for lv3=1:N1
                X(counter) = State{lv1,lv2,lv3}(1);
                Y(counter) = State{lv1,lv2,lv3}(2);
                Z(counter) = State{lv1,lv2,lv3}(3);
                Value1(counter)=Cost{lv1,lv2,lv3}(1);
                Value2(counter)=Cost{lv1,lv2,lv3}(2);
                Value3(counter)=Cost{lv1,lv2,lv3}(3);
                fvalVec =  [Value1(counter) Value2(counter) Value3(counter)];
                [minVal index] = min(fvalVec);
                if index==1
                    Value(counter) = 0;
                elseif index==2
                    Value(counter) = 2.5;
                else
                    Value(counter) = 5;
                end
                counter = counter+1;
            end 
        end 
    end
    close all;
%     figure;scatter3(X,Y,Z,15,Value1);colorbar;
%     figure;scatter3(X,Y,Z,15,Value2);colorbar;
%     figure;scatter3(X,Y,Z,15,Value3);colorbar;
font_size  = 15;
line_size  = 15;
line_width = 2;

figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], ...
'PaperPosition', [0, 0, 11, (6/8)*11]);
set(gcf,'Renderer','OpenGL');
set(gca,'FontSize',20) %Set size of axis font
axis equal
%
scatter3(X,Y,Z,15,Value);colorbar;
%Create label
xlabel('$\bar{x}$ (m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
ylabel('$\bar{y}$ (m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
zlabel('$\bar{\theta}$ (rad)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);

end