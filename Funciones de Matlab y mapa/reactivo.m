function reactivo

% Esta función la llama el controlador principal cuando los sensores 
% de ultrasonido detectan que el robot está cerca de colisionar. Entra 
% en un bucle que bloquea el avance lineal y procede a rotar en el sentido 
% correspondiente hasta que los tres sensores dejen de detectar un obstáculo 
% próximo. Al ocurrir esto, vuelve al cuerpo principal de la función
% controlador y salta a la iteración del bucle que corresponda con el 
% punto de la ruta más cercano a la localización estimada actual. Es decir,
% su nuevo punto objetivo será el más cercano de la ruta a la localización 
% del robot

global robot sensorCentral sensorIzquierdo sensorDerecho world
cen=apoloGetUltrasonicSensor(sensorCentral);
izq=apoloGetUltrasonicSensor(sensorIzquierdo);
drch=apoloGetUltrasonicSensor(sensorDerecho);

while cen<0.35 || izq<0.35 || drch<0.35
    
    cen=apoloGetUltrasonicSensor(sensorCentral);
    izq=apoloGetUltrasonicSensor(sensorIzquierdo);
    drch=apoloGetUltrasonicSensor(sensorDerecho);
    
    vf=0;
    
    if drch<izq
        vg=0.1;
    elseif drch>izq
        vg=-0.1;
    elseif drch==izq
        apoloMoveMRobot(robot,[vf,0.1],1,world);
    end
    
    apoloMoveMRobot(robot,[vf,vg],0.1,world);
    apoloUpdate();
    pause(0.001)
    
end
end