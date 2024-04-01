% sensor_data.m
%
% Esta funcion retorna la distancia sensada entre el robot y obstaculo, o
% limite del mapa a cierto angulo ingresado.

%% Inicio de la funcion
function [dist_sensed, sensor_lecture] = sensor_data(angle, position, range, obs, map, limit)
    sensor_lecture = [];
    dist_sensed = limit;
    sensor_dist_sensed = min(norm([map(2) - map(1) map(4) - map(3)]), range);
    sensor_end = position + sensor_dist_sensed * [cos(angle) sin(angle)];
    sensor_ray = [position; sensor_end];

    obstacle = [map(1) map(1) map(2) map(2); ... % Tomando los bordes del mapa como obstaculo
                map(3) map(4) map(4) map(3)]';
    [pos_inter_x, pos_inter_y] = intersection(obstacle, sensor_ray);
                         
    distances = dist_sensed;
    for j=1:length(pos_inter_x)
        distances(j+1) = dist_function([pos_inter_x(j) pos_inter_y(j)], position);
        sensor_lecture = [sensor_lecture; pos_inter_x pos_inter_y];
    end
    
    dist_sensed = min(distances);
    for i=1:length(obs)
        obstacle = obs{i};
        obstacle = [obstacle; obstacle(1,:)];
        [pos_inter_x,pos_inter_y] = intersection(obstacle, sensor_ray);
      
        distances = dist_sensed;
        for j=1:length(pos_inter_x)
            distances(j+1) = dist_function([pos_inter_x(j) pos_inter_y(j)], position);
            sensor_lecture = [sensor_lecture; pos_inter_x pos_inter_y];
        end
      
	dist_sensed = min(distances);

    end

    if isempty(sensor_lecture)
        sensor_lecture = sensor_end;
    end
end
