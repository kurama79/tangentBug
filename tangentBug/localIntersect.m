% localIntersect.m

%% Inicio de funcion
function [x, y] = localIntersect(x1, y1, x2, y2)
    if ~isequal(x1,x2)
       xx = unique([x1 x2]);
       xx = xx(xx >= max(min(x1), min(x2)) & xx <= min(max(x1), max(x2)));
       
       if numel(xx) < 2
          x=[];
          y=[];
          return
       end
       
       yy = interp1(x1, y1, xx) - interp1(x2, y2, xx);
       
    else
       xx = x1;
       yy = y1 - y2;
    end
    
    x = interpInver(xx, yy, 0);
    
    if ~isempty(x)
       y = interp1(x1, y1, x);
    else
        
    x=[];
    y=[];
    
    end
end