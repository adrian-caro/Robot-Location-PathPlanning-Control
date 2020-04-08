clc
clear all

% El script de calibración se encarga de realizar una serie de 
% pruebas para comprobar la fiabilidad de las medidas hechas por 
% los sensores del robot. Dicho script está compuesto por cuatro 
% módulos, que el usuario puede ejecutar por separado, para llevar
% a cabo pruebas correspondientes a la calibración de la odometría 
% lineal y angular, a la calibración de los sensores láser y a la
% calibración de los sensores de ultrasonidos.


world='World 1';    % Mundo de apolo
robot='Paco';       % Robot de apolo
tipo=4; %1:calibración lineal, 2:calibración angular, 3: calibración láser, 4:calibración ultrasónica

switch tipo
    case 1 
        %% Odometría lineal(desplazamientos de 0.5 m)
        velLineal = 0.5;  
        apoloPlaceMRobot(robot,[0,1,0],1.570796,world);
        
        for i=1:1000
            if (rem(i,10)==0) % Cada 10 iteraciones se reposiciona el robot.
                apoloPlaceMRobot(robot,[0,1,0],1.570796,world);
                pause(0.001)
            end
            apoloResetOdometry(robot);
            apoloMoveMRobot(robot,[velLineal,0],1,world);
            pause(0.001)
            odometria=apoloGetOdometry(robot,world);
            calLineal(i)=odometria(1);
           
        end
        message = sprintf('La varianza dada por la calibración es de:\n %e con un error de la media de %e', var(calLineal),mean(calLineal)-0.5);
        h=msgbox(message,'Finalizado.','help');
        
    case 2 
        %% Odometría angular(giros de 1 rad)
        velGiro = pi;
        apoloPlaceMRobot(robot,[0,1,0],1.570796,world);
        for i=1:1000       
            apoloResetOdometry(robot);
            apoloMoveMRobot(robot,[0,velGiro],1,world);
            pause(0.001)
            odometria=apoloGetOdometry(robot,world);
            calAngular(i)=odometria(3);
        end
        message = sprintf('La varianza dada por la calibración es de:\n %e con un error de la media de %e', var(calAngular),mean(calAngular)-1);
        h=msgbox(message,'Finalizado.','help');
    case 3 
        %% Sensor láser      
        x=0;
        distancias=[1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10]; %distancias del sensor del robot a la pared 
        for j=-3.9:0.5:5.1
            apoloPlaceMRobot(robot,[j,4,0],pi,world);           
            for i=1:1000
                pause(0.001)
                Landmark=apoloGetLaserLandMarks('LMS100',world);
                cal(i)=Landmark.distance(find(Landmark.id==13)); %para tomar la distancia a la baliza 13
            end
            x=x+1;
            varLandmark(x)=var(cal);
            meanLandmark(x)=mean(cal);
            
            
        end
        errorDistancias=meanLandmark-distancias;
        for i=1:1:x
            vecMeanVarLaser(i)=mean(varLandmark);
            vecMeanErrorLaser(i)=mean(errorDistancias);
        end
        
       
        subplot(1,2,1)
        hold on
        plot(distancias,varLandmark)
        plot(distancias,vecMeanVarLaser)
        title('Calibración sensor láser')
        xlabel('Distancia')
        ylabel('Varianza')
        axis([1 10 -inf inf])
        
        
        subplot(1,2,2)
        hold on
        plot(distancias,errorDistancias)
        plot(distancias,vecMeanErrorLaser)
        title('Calibración sensor láser')
        xlabel('Distancia')
        axis([1 10 -inf inf])
        ylabel('Error de la media')
        
        message = sprintf('La varianza dada por la calibración es de:\n %e con un error de la media de %e', mean(varLandmark),mean(errorDistancias));
        h=msgbox(message,'Finalizado.','help');
    case 4 
        %% Sensor ultrasonido
        distancias=[0.5,0.75,1,1.25,1.5,1.75,2,2.25,2.5,2.75,3];
        x=0;
        for j=-4.32:0.25:-1.82
            apoloPlaceMRobot(robot,[j,4,0],pi,world);
            for i=1:1000
                pause(0.001)
                ultra=apoloGetUltrasonicSensor('uc0',world);
                cal(i)=ultra;
            end
            x=x+1;
            varUltra(x)=var(cal);
            meanUltra(x)=mean(cal);
        end
        errorDistancias=meanUltra-distancias;
        for i=1:1:x
            vecMeanVarUltra(i)=mean(varUltra);
            vecMeanErrorUltra(i)=mean(errorDistancias);
        end
        subplot(1,2,1)
        hold on
        plot(distancias,varUltra)
        plot(distancias,vecMeanVarUltra)
        title('Calibración sensor ultrasonido')
        xlabel('Distancia')
        ylabel('Varianza')
        axis([0.5 3 -inf inf])
        
        
        subplot(1,2,2)
        hold on
        plot(distancias,errorDistancias)
        plot(distancias,vecMeanErrorUltra)
        title('Calibración sensor ultrasonido')
        xlabel('Distancia')
        axis([0.5 3 -inf inf])
        ylabel('Error de la media')
        
        message = sprintf('La varianza dada por la calibración es de:\n %e con un error de la media de %e',mean(varUltra),mean(errorDistancias));
        h=msgbox(message,'Finalizado.','help');
end
