function controlador(hilo)



% Envía los comandos de velocidad y giro al simulador de Apolo en 
% función de la posición y orientación actual del robot (información 
% proporcionada por la función de localización) y las distintas posiciones 
% que debe ir alcanzando, dada la ruta establecida por el planificador.
% La metodología se basa en los siguientes pasos:

% 1.	Gira el robot hasta alinear su eje de movimiento con el punto de 
% destino siguiente, con una cierta tolerancia.
% 2.	Calcula la distancia hasta ese punto y ejecuta los comandos de 
% movimiento lineal necesarios mediante la función ‘apoloMoveMRobot’.
% 3.	Repite el bucle para el siguiente punto de la ruta hasta que alcanza 
% la posición de destino.

% En el caso de que se cumplan ciertas condiciones de proximidad en función
% de las medidas de los sensores de ultrasonidos, llama a la función 
% encargada del control reactivo, interrumpiendo el bucle anterior. 



global robot world xo yo xg yg thetao Pkini sensorCentral sensorIzquierdo sensorDerecho

n=1;                            % Proporcionalidad tiempo/giro
parada=0.01;                   % Tiempo de ejecución del apolo
t=0.1*n;                        % Tiempo de los comandos de velocidad
velR=0.1/n;                     % Velocidad de rotación
velL=0.35;                      % Velocidad lineal

Pk=Pkini;                       % Valor inicial de P
Xk=[xo+0.1;yo-0.1;thetao];  % Valor inicial de X

plot(xo,yo,'color','g','marker','o','MarkerSize',8,'linewidth',2);

i=1;
while i<(size(hilo,2)-1) % -1 para que no de error en la última iteración.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cen=apoloGetUltrasonicSensor(sensorCentral);
    izq=apoloGetUltrasonicSensor(sensorIzquierdo);
    drch=apoloGetUltrasonicSensor(sensorDerecho);
    
    if cen<0.2 || izq<0.2 || drch<0.2
        reactivo();
        [Xk,Pk]=localizacionDistancia(Xk,Pk);
        ganador=0;
        distanciaMinima=inf;
        for j=1:(size(hilo,2)-1)
            dist=sqrt((Xk(1)-hilo(1,j))^2+(Xk(2)-hilo(2,j))^2);
            if dist<distanciaMinima
                distanciaMinima=dist;
                ganador=j;
            end
        end 
        i=ganador;  
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    thetaObjetivo=atan2(-Xk(2)+hilo(2,i+1),-Xk(1)+hilo(1,i+1));
    thetaObjetivo=round(thetaObjetivo*100)/100;
    
    [Xk,Pk]=localizacionDistancia(Xk,Pk);
    theta= round(Xk(3)*100)/100;
    
    %% Comprobación de ángulo (con signos distintos)
    
    if sign(theta)~=sign(thetaObjetivo) && theta~=0
        
        if thetaObjetivo<0 && thetaObjetivo<-pi/2
            
            while thetaObjetivo~=theta
                apoloMoveMRobot(robot,[0,velR],t,world);
                pause(parada)
                [Xk,Pk]=localizacionDistancia(Xk,Pk);
                theta= round(Xk(3)*100)/100;
                apoloUpdate()
            end
            
        elseif (thetaObjetivo<0) && (thetaObjetivo>-pi/2)
            
            while (thetaObjetivo~=theta)
                apoloMoveMRobot(robot,[0,-velR],t,world);
                pause(parada)
                [Xk,Pk]=localizacionDistancia(Xk,Pk);
                theta= round(Xk(3)*100)/100;
                apoloUpdate()
            end
            
        elseif (thetaObjetivo>=0) && (thetaObjetivo<pi/2)
            
            while thetaObjetivo~=theta
                apoloMoveMRobot(robot,[0,velR],t,world);
                pause(parada)
                [Xk,Pk]=localizacionDistancia(Xk,Pk);
                theta= round(Xk(3)*100)/100;
                apoloUpdate()
            end
            
        elseif (thetaObjetivo>=0) && (thetaObjetivo>pi/2)
            
            while thetaObjetivo~=theta
                apoloMoveMRobot(robot,[0,-velR],t,world);
                pause(parada)
                [Xk,Pk]=localizacionDistancia(Xk,Pk);
                theta= round(Xk(3)*100)/100;
                apoloUpdate()
            end
            
        end
    end
    
    %% Comprobación de ángulo (con mismo signo)
    
    while thetaObjetivo~=theta
        if thetaObjetivo>theta && thetaObjetivo~=theta
            apoloMoveMRobot(robot,[0,velR],t,world);  % Signo positivo, gira a la izquierda
        elseif thetaObjetivo<theta && thetaObjetivo~=theta
            apoloMoveMRobot(robot,[0,-velR],t,world); % Signo positivo, gira a la derecha
        end
        
        pause(parada)
        [Xk,Pk]=localizacionDistancia(Xk,Pk);
        theta= round(Xk(3)*100)/100;
        apoloUpdate()
    end
    
    c=0; % Contador para el plot de posición.
    
    %% Comprobación de coordenadas
    
    [Xk,Pk]=localizacionDistancia(Xk,Pk);
    x=Xk(1);
    y=Xk(2);
    
%     figure(1);
%     hold on;
%     plot(Xk(1),Xk(2),'color','m','marker','.','markersize',12);
%     x=Xk(1)+(0.1*cos(Xk(3)));
%     y=Xk(2)+(0.1*sin(Xk(3)));
%     plot([Xk(1) x],[Xk(2) y],'color','m','markersize',2)
    
    xObjetivo=hilo(1,i+1);
    yObjetivo=hilo(2,i+1);
    plot(xObjetivo,yObjetivo,'Color',[0.5 0.5 0.5],'marker','.','MarkerSize',12);
    distancia=floor(sqrt((x-xObjetivo)^2+(y-yObjetivo)^2)*10)/10;
    
    for j=0:velL*t:distancia
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        cen=apoloGetUltrasonicSensor(sensorCentral);
        izq=apoloGetUltrasonicSensor(sensorIzquierdo);
        drch=apoloGetUltrasonicSensor(sensorDerecho);
        
        if cen<0.2 || izq<0.2 || drch<0.2
            reactivo();
            [Xk,Pk]=localizacionDistancia(Xk,Pk);
            ganador=0;
            distanciaMinima=inf;
            for j=1:(size(hilo,2)-1)
                dist=sqrt((Xk(1)-hilo(1,j))^2+(Xk(2)-hilo(2,j))^2);
                if dist<distanciaMinima
                    distanciaMinima=dist;
                    ganador=j;
                end
            end
            i=ganador;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        apoloMoveMRobot(robot,[velL,0],t,world);
        pause(parada)
        apoloUpdate();
        
        [Xk,Pk]=localizacionDistancia(Xk,Pk);
        
        c=c+1;
        if c==2
            figure(1);
            hold on;
            plot(Xk(1),Xk(2),'color','m','marker','.','markersize',12);
            x=Xk(1)+(0.1*cos(Xk(3)));
            y=Xk(2)+(0.1*sin(Xk(3)));
            plot([Xk(1) x],[Xk(2) y],'color','m','markersize',2)
            c=0;
        end
        posicionreal=apoloGetLocationMRobot(robot,world);
        plot(posicionreal(1),posicionreal(2),'color','k','marker','.','MarkerSize',5);
    end
    
    i=i+1;
end
    plot(xg,yg,'color','m','marker','o','MarkerSize',8,'linewidth',2);
end