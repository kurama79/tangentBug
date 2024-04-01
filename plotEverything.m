% plotEverithing.m
%
% Funcion para graficar el movimiento del robot

%% Inicio de funcion
function plotEverything(x, y, sensor_lecture, obs, map, start, goal)
    %%
    clf;
    title('Algoritmo Tangent Bug');
    hold on;
    grid on;
    
    % Limites del mapa
    limit_min_x = map(1);
    limit_min_y = map(3);
    limit_max_x = map(2);
    limit_max_y = map(4);
    conect_limits = [limit_min_x limit_min_x limit_max_x limit_max_x limit_min_x;...
                     limit_min_y limit_max_y limit_max_y limit_min_y limit_min_y];
    
    line(conect_limits(1,:), conect_limits(2,:), 'Color', 'black');
    
    % Obstaculos
    for i = 1:length(obs)                  
        obstacle = obs{i};                      
        patch(obstacle(:,1), obstacle(:,2),'black');  
    end 
    
    % Posicion inicial y posicion final
    plot(start(1), start(2), 'go', ...
                             'MarkerSize', 10, ...
                             'MarkerEdgeColor', 'g', ...
                             'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'ro', ...
                           'MarkerSize', 10, ...
                           'MarkerEdgeColor', 'r', ...
                           'MarkerFaceColor', 'r');
    
    axis tight;
    axis square;
    
    %%    
    % Rango del sensor
    plot(sensor_lecture(:,1), sensor_lecture(:,2), 'c--')
    plot(x, y, 'mo', ...
               'MarkerSize', 5, ...
               'MarkerEdgeColor', 'm', ...
               'MarkerFaceColor', 'm');
    
    hold off
end