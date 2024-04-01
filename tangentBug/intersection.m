% intersection.m
%
% Funcion para determinar la interseccion del radio del sensor con algun
% obstaculo.

%% Inicio de funcion
function [x, y] = intersection(obs, sensor)
    x=[]; 
    y=[];
    
    [obs_x, obs_y, sensor_x, sensor_y] = inputVerif(obs, sensor);
    ind_obs_x = sign(diff(obs_x)); 
    ind_sensor_x = sign(diff(sensor_x));

    ind_1 = 1;
    while ind_1 < length(obs_x)
        ind_max = ind_1 + find(ind_obs_x(ind_1:end) ~= ind_obs_x(ind_1), 1) - 1;
        if isempty(ind_max) || ind_max == ind_1
            ind_max = length(obs_x); 
        end
        ind_1 = ind_1:ind_max;

        ind_2 = 1;
        while ind_2 < length(sensor_x)
            ind_max = ind_2 + find(ind_sensor_x(ind_2:end) ~= ind_sensor_x(ind_2), 1 ) - 1;
            if isempty(ind_max) || ind_max == ind_2
                ind_max = length(sensor_x); 
            end
            ind_2 = ind_2:ind_max;

            % Distinción de casos
            if ind_obs_x(ind_1(1)) == 0 && ind_sensor_x(ind_2(1)) ~= 0 
                x_loc = obs_x(ind_1(1));
                y_loc = interp1(sensor_x(ind_2), sensor_y(ind_2), x_loc);
                
                if ~(y_loc >= min(obs_y(ind_1)) && y_loc <= max(obs_y(ind_1)))
                    y_loc=[]; 
                    x_loc=[]; 
                end
                
            elseif ind_sensor_x(ind_2(1)) ~= 0 && ind_obs_x(ind_1(1)) ~= 0
                
                [x_loc, y_loc] = localIntersect(obs_x(ind_1), obs_y(ind_1), sensor_x(ind_2), sensor_y(ind_2));

            end
            
            x = [x; x_loc(:)];
            y = [y; y_loc(:)];
            
            ind_2 = ind_2(end);
        end
        
        ind_1 = ind_1(end);
    end
end