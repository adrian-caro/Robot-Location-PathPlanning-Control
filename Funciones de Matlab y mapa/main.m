clear all
close all
clc

% Proyecto creado por 
% Hugo Rajado Sánchez (M19205), 
% Adrián Caro Zapata (M19032) y 
% Pablo Dopazo Cordones (M19066)


% Este es el código central de la implementación. Desde aquí se interacciona 
% con el resto de las funciones, se hacen las comprobaciones necesarias, 
% se lleva el control de flujo del programa y se declaran las pausas 
% necesarias para garantizar la correcta sincronización con los procesos de 
% la simulación en Apolo. Para lanzar los algoritmos y seguidamente la 
% simulación de Apolo se deben configurar los parámetros en el archivo 
% ‘configuración.m’ y ejecutar esta script.


global robot world xo yo thetao algoritmo

if exist('apoloGetLocation')==0
    waitfor(msgbox('Add Apolo Matlab Functions to path ','¡Atention!','warn'));
    return
end

configuracion();
apoloPlaceMRobot(robot,[xo,yo,0],thetao,world);

switch algoritmo
    case 1
        fprintf('Ejecutando algoritmo RRT...\n');
        hilo=RRT();
    case 2
        fprintf('Ejecutando algoritmo RRT-connect...\n');
        hilo=RRT_CONNECT();
end

fprintf('Algoritmo finalizado.\n');
pause(0.5);

fprintf('Optimizando la ruta...\n');
hilo=smoothing(hilo,0.3);
fprintf('Ruta optimizada.\n');

apoloPlaceMRobot(robot,[xo,yo,0],thetao,world);
apoloResetOdometry(robot);
apoloUpdate();
fprintf('Esperando a que termine la ejecución de Apolo...\n');
for i=15:-1:1
    pause(1);
    fprintf('%d, ',i);
end
disp('0.')
fprintf('Simulación lista.\n');

fprintf('Simulación en ejecución...\n');
controlador(hilo);
fprintf('Fin.\n');