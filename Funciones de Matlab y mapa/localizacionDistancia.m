function [Xk,Pk]=localizacionDistancia(Xk_1,Pk_1)

% Esta es una implementaci�n iterativa del Filtro Extendido de Kalman,
% es decir, un estimador din�mico que en este caso utiliza las medidas 
% de la odometr�a del robot y la distancia a las balizas que le sean 
% visibles. La funci�n recoge la matriz de covarianzas del estado P_k y
% el estado X_k del momento anterior y devuelve sus valores para el momento
% actual.

global robot world laser R Qlin Qrot t

n=size(t,1); % N�mero de balizas

Qk = [Qlin 0 0; 0 Qlin 0; 0 0 Qrot];
Rk = eye(n)*R;


%% (1) Prediccion (dado k-1, predecimos k)
% Predicci�n del estado

odom=apoloGetOdometry(robot,world);

Xk_ = [(Xk_1(1) + odom(1)*cos(Xk_1(3)));  % Predici�n del estado
       (Xk_1(2) + odom(1)*sin(Xk_1(3)));
       (Xk_1(3) + odom(3))];

Ak = [1 0 -odom(1)*sin(Xk_1(3));  % Jacobiana respecto de Xk
      0 1 odom(1)*cos(Xk_1(3));
      0 1 0];

Bk = [cos(Xk_1(3)) 0 0;  % Jacobiana respecto de odom
      sin(Xk_1(3)) 0 0;
      0 0 1];

Pk_ = Ak*Pk_1*((Ak)') + Bk*Qk*((Bk)');  % Predicci�n de Pk

% Prediccion de la medida

Zk_=zeros(n,1);
Hk=zeros(n,3);

for i=1:n
    Zk_(i,1) = sqrt((t(i,2)-Xk_(2))^2+(t(i,1)-Xk_(1))^2);
    Hk(i,:) = [ -(2*t(i,1) - 2*Xk_(1))/(2*((t(i,1) - Xk_(1))^2 + (t(i,2) - Xk_(2))^2)^(1/2)), -(2*t(i,2) - 2*Xk_(2))/(2*((t(i,1) - Xk_(1))^2 + (t(i,2) - Xk_(2))^2)^(1/2)), 0];
end

%% (2) Observacion de las balizas (k). Tres �ngulos con ruido a�adido.

medida=apoloGetLaserLandMarks(laser,world);
Zk=zeros(n,1);

for i=1:length(medida.id)
    k=medida.id(i);
    Zk(k,1) = medida.distance(i);
end

%% (3) Comparaci�n

Yk = Zk-Zk_;  % Innovaci�n de la medida.


for i=1:n
    encontrado=0;
    for j=1:length(medida.id)
        if i==medida.id(j)
            encontrado=1;
        end
        
    end
    if encontrado==0
        Yk(i,1) = 0;
    end
end

Sk = Hk*Pk_*((Hk)') + Rk;  % Varianza de la innnovaci�n.
Wk = Pk_*((Hk)')*inv(Sk);  % Ganancia de Kalman


%% (4) Correcci�n

Xk = Xk_ + Wk*Yk;
Pk = (eye(3)-Wk*Hk)*Pk_;

if Xk(3)>pi
    Xk(3)=Xk(3)-2*pi;
end
if Xk(3)<(-pi)
    Xk(3)=Xk(3)+2*pi;
end

apoloResetOdometry(robot);

end


