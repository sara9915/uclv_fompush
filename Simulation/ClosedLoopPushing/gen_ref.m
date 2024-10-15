function x_finals = gen_ref(points)
    % points = [x1 y1; x2 y2; ... ; xN yN];
    % x_finals = [x1 y1 th1; x2 y2 th2; ... ; xN yN thN];
    x_finals = zeros(2*size(points,1)-1,3);
    
%     OLD VERSION
%     x_finals = zeros(length(points),3);
%     x_finals(:,1:2) = points;
%     x_finals(1,3) = atan2(points(1,2), points(1,1));
%     x_finals(2:end,3) = atan2(points(2:end,2) ,points(2:end,1) - points(1:end-1,1));
    j = 1;
%      NEW VERSION
    for i = 1:size(x_finals,1)   
       x_finals(i,1:2) = points(j,:);
       if mod(i,2) == 0 && j < size(points,1)
           x_finals(i,3) = atan2(points(j+1,2), points(j+1,1));
           j = j+1;
       else
           if i>1
              x_finals(i,3) = x_finals(i-1,3); 
           end 
       end 
    end

end