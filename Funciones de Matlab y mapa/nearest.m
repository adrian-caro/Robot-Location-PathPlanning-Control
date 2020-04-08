function [x,y,ganador]=nearest(rx,ry,arbol)

% Recibe como par�metros un punto (x,y) y una estructura �rbol.
% Va uno a uno recorriendo los puntos del �rbol y calculando su
% distancia respecto al punto proporcionado. Devuelve el punto 
% del �rbol m�s cercano a dicho punto y su �ndice (posici�n) 
% dentro de la estructura �rbol.

ganador=0;
distanciaMinima=inf;

for i=1:length(arbol.x)
    dist=sqrt((arbol.x(i)-rx)^2+(arbol.y(i)-ry)^2);
    if dist<distanciaMinima
        distanciaMinima=dist;
        ganador=i;
    end
end

x=arbol.x(ganador);
y=arbol.y(ganador);

end