clear all
close all
clc

% Proyecto creado por 
% Hugo Rajado S�nchez (M19205), 
% Adri�n Caro Zapata (M19032) y 
% Pablo Dopazo Cordones (M19066)


% Este es el c�digo central de la implementaci�n. Desde aqu� se interacciona 
% con el resto de las funciones, se hacen las comprobaciones necesarias, 
% se lleva el control de flujo del programa y se declaran las pausas 
% necesarias para garantizar la correcta sincronizaci�n con los procesos de 
% la simulaci�n en Apolo. Para lanzar los algoritmos y seguidamente la 
% simulaci�n de Apolo se deben configurar los par�metros en el archivo 
% �configuraci�n.m� y ejecutar esta script.


global robot world xo yo thetao algoritmo

if exist('apoloGetLocation')==0
    waitfor(msgbox('Add Apolo Matlab Functions to path ','�Atention!','warn'));
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
fprintf('Esperando a que termine la ejecuci�n de Apolo...\n');
for i=15:-1:1
    pause(1);
    fprintf('%d, ',i);
end
disp('0.')
fprintf('Simulaci�n lista.\n');

fprintf('Simulaci�n en ejecuci�n...\n');
controlador(hilo);
fprintf('Fin.\n');