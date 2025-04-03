% Trabajo final Sistemas de Control I
% Sistema de control de temperatura para un horno electrico de metales.
%
% Sistemas de control I - 2023
%  - Ramirez Valentin
%  - Gil Cernich Manuel

clc ; close all

%------ Primer Parte : Obtención Función de Transferencia lazo abierto -----
% Declaramos las variables del modelo matematico para el control de temperatura del horno HM-1:
s = tf('s');
h = 0.11;               % Alto
a = 0.19;               % Ancho
l = 0.23;               % Largo
V = h*a*l;              % Volumen interno del Horno en m^3

R = 9.77;               % Resistencia termica
d = 1.225;              % Densidad del aire
m = d*V;                % Masa de aire contenida en el horno
%c = 0.25;               % Calor especifico del aire [Kcal/Kg C]
c = 1006;               % Calor especifico del aire [J/Kg K]
C = m*c;                % Capacitancia termica
TEMP = 1473;            % Temperatura que buscamos [K]

FdTLA = minreal(R/(C*R*s+1))

%------ Segunda Parte : Obtención Función de Transferencia lazo cerrado -----
% Dimensionamos el sensor y el amplificador
Gs_sensor = 0.01;
Gs_ampli = 100;
FdTLC = minreal(feedback(FdTLA, Gs_sensor*Gs_ampli))
step(TEMP*FdTLC); grid;
title('Step response closed loop'); ylabel('Temperature [K]');
% Sistema de orden 1 tiene un unico polo
pole(FdTLC)

%------ Tercera Parte : Error estado estable -----
% Calculo de error y diseno de controlador
Kp = evalfr(FdTLA, 0);
ess_porcentual = (1/(1+Kp))*100;
%------Lugar de Raices------
rlocus(FdTLA); sgrid
%-------Seleccionamos la ley PI-------------
% PI= Kp(s+1/Ti)/s Formula PI, determinar Kp y Ti
% Ajustar el cero del compensador para cancelar el polo de mi ft
% FdTLA2= PI(S) * FdTLA(s) = 
Ti=(-1/pole(FdTLA)) % Elegimos tiempo de integracion de tal forma que se cancele el polo dominante
PI=(s+(1/Ti))/s ; %Forma general del controlador proporcional
FdTLA2= PI * FdTLA;
%rlocus(FdTLA2); sgrid % Funcion de transferencia a lazo abierto con compensador
% Punto de disenio:
s1= -0.0022222;
G1= evalfr(FdTLA,-0.002222); %Evaluo la funcion de transferencia en el punto de trabajo
invK=abs(((s1+(1/Ti))/s1)*G1) %Condicion de modulo
%rlocus(FdTLA2)
Kp= 1/invK;
PI= Kp * PI; %Valor final de nuestro controlador
FdTLApi= minreal( PI * FdTLA);
FdTLCpi= minreal(feedback(FdTLApi,1))
% Error estado estable con controlador
Kp_pi = evalfr(FdTLApi, 0);
ep_pi = 1/(1+Kp_pi) 
%Verificacion de datos obtenidos
step(1475*FdTLCpi);grid
ylabel('Temperatura °K')


