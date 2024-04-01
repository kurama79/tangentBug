% interpInver.m

%% Inicio de funcion
function [xo, yo] = interpInver(x, y, yo)
    n = numel(y);
    x = x(:);
    y = y(:);

    if yo < min(y) || yo > max(y)
       xo = [];
       yo = [];
    else

       below = y < yo; 
       above = y > yo;
       on = y == yo;

       kth = (below(1:n-1) & above(2:n)) | (above(1:n-1) & below(2:n));
       kp1 = [false; kth];

       xo = [];
       if any(kth)                                                     
           alpha = (yo - y(kth)) ./ (y(kp1) - y(kth));
           xo = alpha .* (x(kp1) - x(kth)) + x(kth);
       end        
       
       xo = sort([xo; x(on)]);
       yo = repmat(yo, size(xo));
       
    end 
end