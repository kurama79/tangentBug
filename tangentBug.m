% tangentBug.m
%
% Esta funcion retorna los puntos del camino desde la configuracion inicial
% a la meta basandose en algoritmo Tangent Bug del libro: Principles of
% Robot Motion, referencia de la clase de Robotica I - CIMAT
%
% Por Luis Enrique Ruiz Fernandez
% Febrero - 2021

%% Inicio de funcion
function tangentBug(start, goal, complex_task, sensor_range, limit, vel_max, vel_min, t, safety, obs, map)
    current_pos = start;
    dist_min = dist_function(start, goal); % Distancia minimia entre posicion inicial y la meta
    dist_prece = dist_min;
    dist_follow = dist_min;
    dist_reach = dist_min;
    dir_prece = 0;
    angle_sample = 73; % Resolucion del sensor
    path_x = [];
    path_y = [];
    
    % Variables para el seguimiento del perimetro
    follow_bound = 0;
    count_bound = 0; % Conteo de iteraciones que se ha seguido el perimetro
    start_bound = start; % Punto de inicio el seguimiento del obstaculo
    start_bound_indx = 0;
    cicle_bound = 0; % Determinar si se ciclo el siguimiento de perimetro y declarar que no hay solucion
    dist_min = limit + 1;
    
    %% Inicio de bucle
    while and((current_pos ~= goal),(cicle_bound == 0))
        sensor_angles = linspace(0, pi, angle_sample);
        dist_sensed = zeros(1, angle_sample);
        dir_current = atan2((goal(2)-current_pos(2)), (goal(1)-current_pos(1))); % Orientacion actual del robot
        
        % Lecturas del sensor
        sensor_lectures = []; % General
        sensor_lectures_front = []; % Frontales
        sensor_lectures_back = []; % Traseras

        for i=1 : angle_sample
            sensor_angles(i) = sensor_angles(i) + dir_current - (pi/2);
            
            % Obtenemos lecturas del sensor (frotnales)
            [distance, sensor_lecture] = sensor_data( sensor_angles(i), current_pos, sensor_range, obs, map, limit );
            sensor_lectures_front = [sensor_lectures_front; sensor_lecture];
            
            dist_sensed(i) = distance;
            if dist_sensed(i) < dist_min
                dist_min = dist_sensed(i);
            end
            
            % Obtenemos lecturas del sensor (traseras)
            [~, sensor_lecture] = sensor_data( (sensor_angles(i) + pi), current_pos, sensor_range, obs, map, limit );
            sensor_lectures_back = [sensor_lectures_back; sensor_lecture];
            
        end   
        
        sensor_lectures = [sensor_lectures_front; sensor_lectures_back];
        
        %% Movimiento directo a la meta o los puntos O_i's
        if follow_bound == 0 
            
            if dist_sensed((angle_sample + 1)/2) > (limit/2) % Camino despejado
                current_pos = current_pos + vel_max * t * [cos(dir_current) sin(dir_current)]; % Movimiento en linea recta
                dir_prece = dir_current;
                
            else % Obstaculo en el camino directo
                end_points = [];        
                sense_any = true;
                
                for j=1 : angle_sample
                    if sense_any && dist_sensed(j) < limit/2 
                        end_points = [end_points j];
                        sense_any = false;
                        
                    elseif ~sense_any && dist_sensed(j) > limit/2
                        end_points = [end_points (j-1)];
                        sense_any = true;
                        
                    end 
                end
                
                dist_costs = [];
                cost_min = limit;
                cost_min_indx = 1;
                
                for j=1 : length(end_points)
                    local_end_point = current_pos + dist_sensed(end_points(j)) * ...
                                      [cos(sensor_angles(end_points(j))) sin(sensor_angles(end_points(j)))];
                                  
                    dist_endpoint2goal = dist_function(local_end_point, goal);
                    dist_costs = [dist_costs (dist_sensed(end_points(j)) + dist_endpoint2goal)];
                    
                    if dist_costs(j) < cost_min
                        cost_min = dist_costs(j);
                        cost_min_indx = end_points(j);
                    end
                    
                end
                
                % Por si llegamos a un obstaculo concavo
                current_dist = cost_min;
                if current_dist <= dist_prece 
                    
                    if dist_sensed(cost_min_indx) > (4*safety / 5) && dist_min > safety 
                        current_pos = current_pos + vel_max * (dist_sensed(cost_min_indx) / (2 * sensor_range)) * ...
                                      t * [cos(sensor_angles(cost_min_indx)) sin(sensor_angles(cost_min_indx))];
                                  
                        dist_min = current_dist;
                        dir_prece = sensor_angles(cost_min_indx);
                        
                    else % Hacemos el cambio al seguimiento d perimetro
                        follow_bound = 1;
                        
                        dist_follow = current_dist - dist_sensed(cost_min_indx);
                        dist_reach = dist_function(current_pos, goal); 
                        
                        count_bound = 0;
                        start_bound = current_pos;
                        start_bound_indx = length(path_x);
                    end
                    
                else % Hacemos el cambio al seguimiento d perimetro
                    follow_bound = 1;
                    
                    dist_follow = current_dist - dist_sensed(cost_min_indx);
                    dist_reach = dist_function(current_pos, goal);
                    
                    count_bound = 0;
                    start_bound = current_pos;
                    start_bound_indx = length(path_x);
                end
                
                dist_prece = current_dist;

            end
            
        %% Seguimiento del perimetro del obstaculo
        else
            % Verificamos lo sensado del robot por la parte trasera
            back_angle = sensor_angles((angle_sample + 1)/2) + pi;
            right_angle = sensor_angles((angle_sample + 1)/2) + 3*pi/4;
            left_angle = sensor_angles((angle_sample + 1)/2) - 3*pi/4;
            
            sensor_angles = [sensor_angles back_angle];
            sensor_angles = [sensor_angles right_angle];
            sensor_angles = [sensor_angles left_angle];
            
            [back_sensed, ~] = sensor_data(back_angle, current_pos, sensor_range, obs, map, limit);
            [right_sensed, ~] = sensor_data(right_angle, current_pos, sensor_range, obs, map, limit);
            [left_sensed, ~] = sensor_data(left_angle, current_pos, sensor_range, obs, map, limit);

            dist_sensed = [dist_sensed back_sensed];
            dist_sensed = [dist_sensed right_sensed];
            dist_sensed = [dist_sensed left_sensed];

            sense_min = limit + 1;
            sense_min_indx = 0;
            for i=1 : (angle_sample + 3)
                
                if dist_sensed(i) < sense_min
                    sense_min = dist_sensed(i);
                    sense_min_indx = i;
                end
                
            end

            % Determinamos la direccion que debe seguir en el perimetro
            vec_1 = sqrt((cos(dir_prece) + cos(sensor_angles(sense_min_indx) + pi/2))^2 + ...
                      (sin(dir_prece) + sin(sensor_angles(sense_min_indx) + pi/2))^2);
            vec_2 = sqrt((cos(dir_prece) + cos(sensor_angles(sense_min_indx) - pi/2))^2 + ...
                      (sin(dir_prece) + sin(sensor_angles(sense_min_indx) - pi/2))^2);

            if vec_1 >= vec_2
                dir2move = sensor_angles(sense_min_indx) + (pi/2);
            else
                dir2move = sensor_angles(sense_min_indx) - (pi/2);
            end
            
            % Determinar la velocidad del robot de acuerdo a la cercania de los osbtaculos
            close_obs = (sense_min - safety) * [cos(sensor_angles(sense_min_indx)) sin(sensor_angles(sense_min_indx))];
            
            if sense_min < safety
                keep_close = vel_min * t * [cos(dir2move) sin(dir2move)]; 
            else
                keep_close = (sense_min - safety + (vel_min * t)) * [cos(dir2move) sin(dir2move)];    
            end

            current_pos = current_pos + ((keep_close) + (close_obs * 0.19));  
            dir_prece = atan2(keep_close(2), keep_close(1));

            % Determinamos si existe solucion
            count_bound = count_bound + 1;
            if count_bound > 100
                loopdist = dist_function(current_pos, start_bound);
                if loopdist < safety/2
                    cicle_bound=1;
                    
                end

                for i=start_bound_indx : (length(path_x) - 100)
                    loopdist = dist_function([path_x(i) path_y(i)], current_pos);
                    if loopdist < safety/10
                        cicle_bound = 1;
                        
                    end
                end
            end
            
            dist_reach = dist_function(current_pos, goal);
            if ~complex_task
                dist2follow = [];
                
                for i=1 : size(sensor_lectures, 1)
                    dist2follow = [dist2follow; dist_function(sensor_lectures(i,:), goal)];
                    
                end
                
                [~, indx2follow] = min(dist2follow);
                dist_follow = sensor_lectures(indx2follow,:);
                
            end
            
            % Salimos del seguimiento de perimetro
            if any(dist_reach < (dist_follow)) 
               follow_bound = 0; 
            end

        end

        if ((current_pos(1)-goal(1))^2  + (current_pos(2)-goal(2))^2) < ((vel_max * t)^2)/2
            current_pos = goal;
            
        end 
        
        path_x = [path_x current_pos(1)];
        path_y = [path_y current_pos(2)];
        
        %% Graficamos el entorno y el movimiento del robot
        figure(1)
        plotEverything(path_x(end), path_y(end), sensor_lectures, obs, map, start, goal);
        
    end
end