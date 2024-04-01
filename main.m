% Tarea 1 - Algoritmo TangentBug 
%   Se hara la simulacion del algoritmo Tangent Bug visto en clase, en el
%   cual se mostrara graficamente los resultados ante obstaculos.
% 
% Por Luis Enrique Ruiz Fernandez
% Febrero - 2021

clear; clc;
addpath('tangentBug')

%% Inicio de programa
task = 5;                          % Numero de caso para la generacion de obstaculos
                                   %    1. Multilpes Obstaculos
                                   %    2. Un solo obstaculo
                                   %    3. Obstaculos mas sencillos
                                   %    4. Obstaculo Caracol 
                                   %    5. No se puede resolver...
area_limits = [0 10 0 10];         % Limites del area donde se encuntra el robot
vel_max = 0.5;                     % Velocidad maxima
vel_min = 0.3;                     % Velocidad minima
t = 0.1;                           % Tiempo
sensor_range = 1.0;                % Rango del sensor para considerar interseccion
limit = 15;                        % Rango para determinar si la distancia es infinito
dist_safe = 0.2;                   % Distancia entre obstaculo y robot para recorrerlo


%% Ejecuacion del algoritmo y simulacion

% Iniciamos los parametros
[start, goal, obstacles, complex_task] = initialConfig(task);

% Inicio del algoritmo
tangentBug(start, goal, complex_task, sensor_range, limit, vel_max, vel_min, t, dist_safe, obstacles, area_limits);