function [x1, y1, x2, y2] = inputVerif(obs, sensor)
    x1 = obs(:,1)';
    x2 = sensor(:,1)';
    y1 = obs(:,2)';
    y2 = sensor(:,2)';

    if min(x1)>max(x2) || min(x2)>max(x1) || ...
            min(y2)>max(y1) || min(y1)>max(y2)
        x1=[]; 
        y1=[]; 
        x2=[]; 
        y2=[]; 
        return
    end

end