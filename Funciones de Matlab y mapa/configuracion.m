function configuracion()

%Carga todos los parámetros necesarios para poder ejecutar todas las otras
%funciones  de MATLAB y simulaciones de Apolo en el ‘workspace’ de MATLAB.

global robot world sensorCentral sensorIzquierdo sensorDerecho laser
global A B t algoritmo
global xo yo thetao xg yg
global Pkini R Qlin Qrot

pause(1)                    % Tiempo para correcta inicialización

xo=-7;                      % Nodo origen (posición inicial)
yo=1;
thetao=pi/2;

xg=10;                     % Nodo goal
yg=1;

algoritmo=2;                % 1 = RRT, 2 = RRT-connect
     
world='World 1';            % Mundo de apolo
robot='Paco';               % Robot de apolo
sensorCentral='uc0';        % Sensor central ultrasónico
sensorDerecho='ur1';        % Sensor derecho ultrasónico
sensorIzquierdo='ul1';      % Sensor izquierdo ultrasónico
laser='LMS100';             % Sensor laser

A=12;                       % Ancho del mapa [-A,A] 'x'
B=17;                       % Alto del mapa [0,B] 'y'

R=1.4625e-04;               % Varianza del sensor (distancia)
Qlin=3.0462e-04;                % Varianza odometría (lineal)
Qrot=2.9334e-04;            % Varianza odometría (angular)
Pkini= [0.1 0 0;
        0 0.1 0;            % Matriz de varianzas del estado inicial
        0 0 0.1];

t =[-11.99,00.01; %1        % Posición (x,y) de las balizas
    -05.01,00.01; %2
    00.00,00.01; %3
    07.00,00.01; %4
    11.99,01.00; %5
    -04.99,01.00; %6
    -10.01,02.50; %7
    -05.01,04.00; %8
    -04.99,04.00; %9
    11.99,04.00; %10
    11.99,07.00; %11
    -04.99,07.00; %12
    -11.99,05.51; %13
    -09.01,08.50; %14
    -08.99,08.51; %15
    -04.99,10.00; %16
    11.99,10.00; %17
    11.99,12.01; %18
    07.00,13.99; %19
    00.00,13.99; %20
    -08.99,14.99; %21
    -09.01,15.00; %22
    -11.99,16.99; %23
    -00.01,15.01; %24
    -00.01,16.99; %25
    02.00,16.99; %26
    05.00,16.99; %27
    07.01,15.01; %28
    11.99,16.99]; %29

end
