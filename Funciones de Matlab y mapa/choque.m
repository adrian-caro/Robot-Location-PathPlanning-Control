function [bool]=choque(x,y)

% Esta funci�n recibe un punto (x,y) y devuelve un valor booleano: 1 si no choca, 0 si choca.
% Para comprobar si choca, primeramente, se realiza un posicionamiento del robot con la funci�n 
% �apoloPlaceMRobot� para comprobar la colisi�n en ese punto.
% Para detectar si choca con una pared en una regi�n pr�xima de movimiento, se ha utilizado la 
% funci�n �apoloMoveMRobot� ya que devuelve un 0 si choca. Para un mejor resultado, se ha elegido
% que solo se mueva de forma lineal, y que el robot se oriente cada ?/3 radianes en un rango de [-?,?],
% comprobando si choca en todas esas direcciones. Esta �ltima funci�n nos permite crear un margen de seguridad
% respecto a las paredes ya que se puede configurar la distancia que se mueve el robot comprobando si
% choca mediante el ajuste de su velocidad y tiempo de ejecuci�n del movimiento, descartando puntos 
% demasiado pr�ximos a las paredes que puedan causar colisi�n dada las dimensiones del robot.


global robot world
bool=1;

for theta=-pi:pi/3:pi
    
    bool=apoloPlaceMRobot(robot,[x,y,0],theta,world);
    
    if bool==0 %choca por ca�da
        break
    end
    
    bool=apoloMoveMRobot(robot,[0.25,0],1,world); %funciona con t=1
    
    if bool==0 %choca por movimiento
        break
    end
    
end