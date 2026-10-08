% mex load1.cpp
% mex laser1.cpp
% mex fforce1.cpp
% mex rope1.cpp
% mex simulation1.cpp
% mex force1.cpp
% mex quad1.cpp
% mex wind1.cpp

clearvars;
clearvars global
close all;
clc;

T = 100;%tiempo de simulacion
frameRate = 50;
deltat = 1/frameRate;

% Trayectoria: 1=circ/elipse, 2=recta, 3=espiral, 4=Ocho, 5=Ocho inclinado, 6=punto. [] = preguntar
TrayectoriaDefault = 1;
if isempty(TrayectoriaDefault)
    [xd,yd,zd,pd,xdp,ydp,zdp,pdp,xdpp,ydpp,zdpp,pdpp] = ReferenciasCarga_ST(T,deltat);
else
    [xd,yd,zd,pd,xdp,ydp,zdp,pdp,xdpp,ydpp,zdpp,pdpp] = ReferenciasCarga_ST(T,deltat,TrayectoriaDefault);
end

quad_mass = 0.5;
spring_friction = 0.6;%0.2;  % Spring Inner Friction Constant (0.2 default)
spring_constant = 5000;% Spring Constant In The Rope (10000.0 default)
air_friction = 0.01/20;      % Air Friction Constant (0.02 default/20);
ground_repel = 50/20;     % Ground Repel Constant (100 default/20)
ground_slide = 0.1/20;     % Ground Slide Friction Constant (0.2 default/20)
ground_absorption = 1/20;  % Ground Absoption Constant (2.0 default/20)

wind = [0 0 0];

lcuerda = 2; %Largo de la cuerda
payload = 0.1;% Peso de la carga (AHORA NO! incluye el peso de las cuerdas, 0.04kg c/u)
% payload = 0.0001;% menor a un gramo (0.001 kg) deshabilita la cuerda
tam_carga = 0.1; %Tama�o de la carga (lado de cubo)

filtro_n = 1;
filtro_fn = 0.1;
filtro_exil = 0.35;
filtro_epsi = 0.25;
filtro_hp = 0.2;
epsilon1 = 0.14;

%Ganancias del controlador de seguimiento de trayectoria
Ktraj  = diag([4.0 4.0 2.0]);   %Ganancia fuera de la tanh (error x,y,z)
Ktrajt = diag([0.35 0.35 0.35]);   %Ganancia dentro de la tanh (error x,y,z)
Kvload  = diag([0.45 0.45 0.18]);
Kvloadt = diag([0.90 0.90 0.60]);
Kpsi = 1.0;             %Ganancia fuera de la tanh (error psi)
Kpsit = 0.5;            %Ganancia dentro de la tanh (error psi)

tieneLaser = 0; %arreglo que indica que heli tiene l�ser
% 1 si tiene y 0 si no tiene

nObs = 0;       % N�mero de obst�culos

ErrorInicialMax = 0;
ControlCarga = true; %Si es true, la trayectoria la sigue la carga, si
%es false, la trayectoria la sigue el helic�ptero

% Flags para activar/desactivar etapas de control
UsarKinematicoI        = false;   % Propuesta I: -k_osc*l*dot(theta)
UsarKinematicoII       = false;   % Propuesta II: compensacion cinematica (R, R_dot)
UsarCompensadorDinamico = false;  % Compensador dinamico Super-Twisting adaptativo
UsarAdaptacionST = false; % Se activa la adaptacion de ganancias del ST
% Ganancias del compensador cinemático II propuesto
% Kr_kinII = diag([0.15 0.15 0.0]);   % término restaurador horizontal
% Kv_kinII = diag([1.2 1.2 0.0]);   % término amortiguante horizontal

% Proyector horizontal
Ph = diag([1 1 0]);

%% Exportar figuras y datos para el paper (carpeta ResultadosPaper)
ExportarResultadosPaper = false;   % true = guardar figuras .png/.fig y datos .mat
CarpetaResultados = 'ResultadosPaper';  % carpeta dentro de Simulaciones
% Opcion para cargar ganancias de una simulacion previa
CargarGananciasPrevias = false;  % true = cargar ganancias guardadas, false = usar valores iniciales por defecto
GuardarGananciasFinales = false;   % true = guardar ganancias al final, false = no guardar

% Nombre del archivo para guardar/cargar ganancias
NombreArchivoGanancias = 'GananciasAdaptativas_CompCarga2.mat';

% Cargar ganancias previas si se solicita
if CargarGananciasPrevias
    if exist(NombreArchivoGanancias, 'file')
        fprintf('Cargando ganancias adaptativas desde: %s\n', NombreArchivoGanancias);
        datos_cargados = load(NombreArchivoGanancias);
        if isfield(datos_cargados, 'Ku_final') && isfield(datos_cargados, 'Kv_final')
            Ku_inicial = datos_cargados.Ku_final;
            Kv_inicial = datos_cargados.Kv_final;
            fprintf('  Ku inicial cargado: [%.4f, %.4f, %.4f, %.4f]\n', diag(Ku_inicial));
            fprintf('  Kv inicial cargado: [%.4f, %.4f, %.4f, %.4f]\n', diag(Kv_inicial));
        else
            warning('El archivo no contiene Ku_final o Kv_final. Usando valores por defecto.');
            Ku_inicial = diag([0.8 0.8 1.0 0.4]);
            Kv_inicial = diag([0.2 0.2 0.3 0.1]);
        end
    else
        warning('Archivo de ganancias no encontrado: %s. Usando valores por defecto.', NombreArchivoGanancias);
        Ku_inicial = diag([0.8 0.8 1.0 0.4]);
        Kv_inicial = diag([0.2 0.2 0.3 0.1]);
    end
else
    Ku_inicial = diag([0.8 0.8 1.0 0.4]);
    Kv_inicial = diag([0.2 0.2 0.3 0.1]);
end
%%
% valores iniciales
yXo = zeros(4,1); %[x,y,z,orient]
yXo(1) = xd(1)+ErrorInicialMax*2*(rand-0.5);
yXo(2) = yd(1)+ErrorInicialMax*2*(rand-0.5);
if(ControlCarga)
    yXo(3) = zd(1)+lcuerda+ErrorInicialMax*2*(rand-0.5); %altura inicial
else
    yXo(3) = zd(1)+ErrorInicialMax*2*(rand-0.5); %altura inicial
end
if(ErrorInicialMax)
    yXo(4) = 360*rand; %0� de orientaci�n inicial
else
    yXo(4) = pd(1); %0� de orientaci�n inicial
end

% Inicio Simulaci�n
time_sim = T*2;         % Duplico el tiempo de simulaci�n de los helis para
% que siga C++ por si no se cumple el per�odo de
% muestreo de matlab (por ej. al graficar en l�nea)

ExeName = 'SimQuad1.exe';
[~,msg] = system(['tasklist /fi "ImageName eq ' ExeName '"']);
IsOpen  = contains(msg,ExeName);

quad1(1,[0 0 0 0]);

if(IsOpen)
    simulation1(1);
    pause(0.5);
else
    system('SimQuad1.exe &');
end
simulation1([1 time_sim payload tam_carga lcuerda ...
    spring_friction spring_constant air_friction ground_repel...
    ground_slide ground_absorption nObs], yXo, tieneLaser, [],[]);


pause(1); %Para tiempo a que inicialice la simulaci�n en C++

tt=0:deltat:T;

yQuad=zeros(24,length(tt));
numParticles = 40; %Cantidad de eslabones de la cuerda
yRope=zeros(numParticles,3,length(tt));
yForce=zeros(3,length(tt));
yFnorm=zeros(1,length(tt));

yXi=zeros(3,length(tt));         %Posiciones de los veh�culos
yXip=zeros(3,length(tt));        %Velocidades de los veh�culos

yXil=zeros(3,length(tt));
yXilp=zeros(3,length(tt));
yXid=zeros(3,length(tt));
yXidp=zeros(3,length(tt));
yXidpp=zeros(3,length(tt));
yPsid=zeros(1,length(tt));
yPsidp=zeros(1,length(tt));

yH=zeros(3,length(tt));
yHe=zeros(3,length(tt));
yHp=zeros(3,length(tt));

yWind=zeros(3,length(tt));

yExil=zeros(3,length(tt));
yEpsi=zeros(1,length(tt));
yExilp=zeros(3,length(tt)); % Errores de velocidad x,y,z

yPsi = zeros(1,length(tt));      %Orientaciones de los veh�culos (Yaw)
yPsip= zeros(1,length(tt));      %Velcidad Angular de los veh�culos (Yaw_p)

yU=zeros(4,length(tt));
yUc=zeros(4,length(tt));
yUk=zeros(4,length(tt));
yUcLocal = zeros(4,length(tt));
yUcWorld = zeros(4,length(tt));
yULocal  = zeros(4,length(tt));
yUWorld  = zeros(4,length(tt));
Xdpp=zeros(4,length(tt));
Xdpp_fil=zeros(4,length(tt));

yTraj_vc=zeros(3,length(tt));
yPsi_vc=zeros(1,length(tt));
yComp=zeros(3,length(tt)); %Accion de control de compensacion Oscilaciones

yObs=zeros(nObs,3,length(tt));
ydObs=zeros(nObs,3,length(tt));

yTsim=zeros(1,length(tt));
yTFind=1;

yTsimRope=zeros(1,length(tt));
yTsimHeli=zeros(1,length(tt));

ySimCountRope=zeros(1,length(tt));
ySimCountHeli=zeros(1,length(tt));

% trajectoria de los obstaculos

for i=1:nObs
    yObs(i,1,:)= Obs((i-1)*3+1);
    yObs(i,2,:)= Obs((i-1)*3+2);
    yObs(i,3,:)= Obs((i-1)*3+3);

    ydObs(i,1,:)= dObs((i-1)*3+1);
    ydObs(i,2,:)= dObs((i-1)*3+2);
    ydObs(i,3,:)= dObs((i-1)*3+3);
end

Kfiltro = zeros(1,filtro_n);
fn_ant = 0;
exil_ant = zeros(3,1);
exilp_ant = zeros(3,1);
epsi_ant = 0;
he_ant = zeros(3,1);
hp_ant = zeros(3,1);
yComp_ant = zeros(3,1);

lastsize = 0; %para sobreescribir el string (mensaje) de salida
tic;
finalCamino = false;
hay_viento = false;
k=1;

hay_joy = false;

if(hay_joy)
    joy = vrjoystick(1);
end
% Graficar en L�nea %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig=figure;
% fig.WindowState = 'maximized';
s = get(0, 'ScreenSize');
fig.Position = [s(3)*0.125 s(4)*0.125 s(3)*0.75 s(4)*0.75];
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid;
% axis image;
% view(2);
% view(3);
%     view([-90,0]); %view -Y-Z
%     view([0,0]); %view X-Z
%     view([-180,0]); %view -X-Z

plot3(xd,yd,zd,'--r');
scale=0.573;  %2 normal size
colorFrenteQuad=[1 0 1];  %magenta
color_matlab = get(groot,'defaultAxesColorOrder');
colorquad = color_matlab(1,:);
Nquadrotor_parametros(1,scale,colorFrenteQuad,colorquad);
colorBox = [0.38 0.1 0.035];
colorCuerda = [0.5430 0.2695 0.0742];

H1 = Nquadrotor(1,[0 0 0 0 0 0]);
vent_des=frameRate*6;
if(payload >= 0.001)
    H2 = line(0,0,0);
    H3 = line(0,0,0);
    H4 = cube_plot([0,0,0],.1,.1,.1,'r');
    H6 = animatedline('MaximumNumPoints',vent_des,'Color',colorBox,'LineWidth',1.5);
end
H5 = animatedline('MaximumNumPoints',vent_des,'Color',colorquad,'LineWidth',1.5);
H7 = animatedline('MaximumNumPoints',1,'LineWidth',2,'Color',[1 0 0],'Marker','s','MarkerSize',5);
% x = zeros(1,6);
% r = zeros(10,3);
view(45,20);
axis image;
ErrorAcum = zeros(4,1);
ant_Uk = zeros(4,1);
ant_acc = zeros(4,1);
ant_vel = zeros(4,1);
w = zeros(4,1);
s = zeros(4,length(tt)); % Superficie deslizante

% Ku = Ku_inicial; % Valores iniciales de Ku (cargados o por defecto)
% Kv = Kv_inicial; % Valores iniciales de Kv (cargados o por defecto)
Ku = diag([1.25 1.25 1.15 0.05]);
Kv = diag([0.18 0.18 0.25 0.01]);

Gan_U = zeros(4,length(tt)); % Parametros de Ku
Gan_V = zeros(4,length(tt)); % Parametros de Kv

% Variables en marco local
yXpLocal   = zeros(4,length(tt));   % velocidad real en marco local
yUcLocal   = zeros(4,length(tt));   % referencia cinemática en marco local
yUcFil     = zeros(4,length(tt));   % referencia cinemática filtrada en marco local
yULocal    = zeros(4,length(tt));   % control aplicado en marco local

% Variables en marco mundo
yUcWorld   = zeros(4,length(tt));
yUWorld    = zeros(4,length(tt));

% Variables del compensador dinámico solo para XY
ErrorAcum_xy = zeros(2,1);
nu_xy        = zeros(2,1);                 % estado interno del super-twisting
s_xy         = zeros(2,length(tt));
UcDotFil_xy  = zeros(2,length(tt));        % derivada filtrada de la referencia cinemática local

% Mantengo s global para compatibilidad con tus plots
s = zeros(4,length(tt));

% Parámetro de fuga del integrador
lambda_i = 0.03;

% Parámetros del ST consistente
Lambda_xy = diag([0.08 0.08]);   % superficie deslizante
K1_xy     = diag([0.5 0.5]);   % término sqrt(|s|)
K2_xy     = diag([0.75 0.75]);   % término integral del ST
tau_d     = 0.05;                % convierte pseudoaceleración en corrección de velocidad
alpha_udot = 0.15;               % filtro para derivada de Uc

gamma1 = 0.03;
gamma2 = 0.04;
sigma1 = 0.08;
sigma2 = 0.08;

K1_min = [0.08; 0.08];
K1_max = [1.45; 1.45];

K2_min = [0.10; 0.10];
K2_max = [1.60; 1.60];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inicio Simulaci�n
while (~finalCamino) && (k <= length(tt))
    % Inicio de Simulaci�n
    while (toc < deltat)
    end
    tic;

    if(~simulation1)
        finalCamino = true; %fin de camino, terminar la simulaci�n
        yTFind = k-1;
        break;
    end

    %Leo variables del programa SimHelicop por memoria compartida
    ySimCountHeli(k) = quad1;
    ySimCountRope(k) = rope1;
    yQuad(:,k) = quad1(1);
    yRope(:,:,k) = rope1(1);
    yForce(:,k) = force1(1);
    Kfiltro(1) = norm(yForce(:,k));
    yFnorm(k) = norm(Kfiltro)/filtro_n;
    Kfiltro = circshift(Kfiltro,1,2);

    % Variables del Controlador
    yXi(:,k) = [yQuad(1,k);yQuad(2,k);yQuad(3,k)];
    yXip(:,k) = [yQuad(4,k);yQuad(5,k);yQuad(6,k)];
    yPsi(k) = yQuad(15,k);
    yPsip(k) = yQuad(23,k);
    yTsimHeli(1,k) = yQuad(24,k);

    yTsim(k) = k*deltat;

    if(payload >= 0.001)
        yXil(:,k) = [yRope(end,1,k),yRope(end,2,k),yRope(end,3,k)];   %Posici�n de la Carga
        yH(:,k) = yXi(:,k) - yXil(:,k);
    end

    yXid(:,k) = [xd(k);yd(k);zd(k)];
    yXidp(:,k) = [xdp(k);ydp(k);zdp(k)];
    yXidpp(:,k) = [xdpp(k);ydpp(k);zdpp(k)];
    yPsid(k) = pd(k);
    yPsidp(k) = pdp(k);
    yPsidpp(k) = pdpp(k);

    % Calculo error entre la/el carga/heli y la trayectoria
    if(ControlCarga)
        if(payload >= 0.001)
            yExil(:,k) = yXid(:,k) - yXil(:,k);

            % Velocidad de la carga: v_l = v_uav - h_dot
            v_payload = yXip(:,k) - yHp(:,k);
            yExilp(:,k) = yXidp(:,k) - v_payload;
        end
    else
        yExil(:,k)  = yXid(:,k)  - yXi(:,k);
        yExilp(:,k) = yXidp(:,k) - yXip(:,k);
    end

    % Filtro error entre la carga y el nuevo camino
    if(k==1)
        exil_ant = yExil(:,k);
        exilp_ant = yExilp(:,k);
    end
    yExil(:,k) = filtro_exil*yExil(:,k)+ (1-filtro_exil)*exil_ant;
    yExilp(:,k) = filtro_exil*yExilp(:,k)+ (1-filtro_exil)*exilp_ant;
    exil_ant = yExil(:,k);
    exilp_ant = yExilp(:,k);

    % Filtro norma de la fuerza en cada veh�culo
    yFnorm(k) = filtro_fn*yFnorm(k) + (1-filtro_fn)*fn_ant;
    fn_ant = yFnorm(k);

    % Calculo error de orientaci�n
    yEpsi(k) = yPsid(k) - yPsi(k);
    while abs(yEpsi(k))>pi
        if yEpsi(k)>pi
            yEpsi(k)=yEpsi(k)-2*pi;
        elseif  yEpsi(k)<-pi
            yEpsi(k)=yEpsi(k)+2*pi;
        end
    end

    % Filtro error de orientaci�n
    if(k==1)
        epsi_ant = yEpsi(k);
    end
    yEpsi(k) = filtro_epsi*yEpsi(k)+ (1-filtro_epsi)*epsi_ant;
    epsi_ant = yEpsi(k);

    %% Observador de estado suavizado para estimar h y hp = dh/dt
    % Reemplazar el observador actual por un ST diferenciador
    L1 = 3.0;   % ganancia capa 1
    L2 = 4.5;   % ganancia capa 2
    % En el bucle:
    e_obs = yH(:,k) - yHe(:,k);
    yHe_dot = yHp(:,k) - L1*sqrt(abs(e_obs)).*sign(e_obs);
    yHp_dot = -L2*sign(e_obs);
    yHe(:,k+1) = yHe(:,k) + yHe_dot*deltat;
    yHp(:,k)   = yHp(:,k) + yHp_dot*deltat;
    %% Calculo comandos de velocidad para cada objetivo de control
    % Parte base de seguimiento de trayectoria de la carga
    %     v_base = yXidp(:,k) + Ktraj*tanh(Ktrajt*yExil(:,k));
%     Ktraj  = diag([3.6 3.6 2.0]);
%     Ktrajt = diag([0.35 0.35 0.35]);

    v_base = yXidp(:,k) ...
        + Ktraj *tanh(Ktrajt *yExil(:,k))...
        + Kvload*tanh(Kvloadt*yExilp(:,k));

    % Inicialización
    %     yComp(:,k) = zeros(3,1);

    yComp(:,k) = zeros(3,1);

    if payload >= 0.001
        if UsarKinematicoII
            % Aceleración deseada horizontal de la carga
            a_ref_h = yXidpp(1:2,k);

            % Referencia cuasiestática de inclinación horizontal
            h_ref_h = (lcuerda/9.81) * a_ref_h;

            % Derivada de h_ref (aprox. usando diferencia filtrada)
            if k == 1
                h_ref_dot_h = [0;0];
                h_ref_h_ant = h_ref_h;
                h_ref_dot_fil = [0;0];
            else
                h_ref_dot_raw = (h_ref_h - h_ref_h_ant)/deltat;
                alpha_hrefdot = 0.60;
                h_ref_dot_fil = alpha_hrefdot*h_ref_dot_raw + (1-alpha_hrefdot)*h_ref_dot_fil;
                h_ref_dot_h = h_ref_dot_fil;
                h_ref_h_ant = h_ref_h;
            end

            H_h  = yH(1:2,k);
            Hp_h = yHp(1:2,k);

            % Ganancias recomendadas
            Kh  = diag([1.8 1.8]);
            Kdh = diag([0.6 0.6]);
%             Kh  = diag([1.8 1.8]);
%             Kdh = diag([0.55 0.55]);

            % Error respecto a la referencia de inclinación dinámica
            eh_h  = H_h  - h_ref_h;
            ehp_h = Hp_h - h_ref_dot_h;

            comp_xy = -Kh*eh_h - Kdh*ehp_h;

            % Saturación suave
            comp_max = 0.9;
            comp_xy = comp_max * tanh(comp_xy/comp_max);

            yComp(:,k) = [comp_xy; 0];
        elseif UsarKinematicoI
            Kosc = diag([0.2 0.2 0.0]);
            yComp(:,k) = Kosc*diag([-lcuerda -lcuerda 0])*yHp(:,k);
            yComp(:,k) = yComp_ant + yComp(:,k)*deltat;
            yComp_ant = yComp(:,k);
        end
    end

    %------------------ RAMPA SUAVE DE ARRANQUE ------------------%
    Tsoft = 3.0;                      % [s]
    sigma = 1 - exp(-yTsim(k)/Tsoft);

    v_base = sigma * v_base;
    yComp(:,k) = sigma * yComp(:,k);
    %-------------------------------------------------------------%

    % Referencia cinemática global en marco inercial
    yTraj_vc(:,k) = v_base + yComp(:,k);

    % Referencia de yaw
    yPsi_vc(:,k) = yPsidp(k) + Kpsi*tanh(Kpsit*yEpsi(k));

    % Armo los Jacobianos necesarios
    Jl = horzcat(eye(3),[0;0;0]);
    Jp = zeros(1,4);
    Jp(1,4)=1;

    J = [Jl;Jp];

    % Armo el vector error total
    El = yTraj_vc(:,k);
    Ep = yPsi_vc(:,k);
    E = [El;Ep];

    % Acción de Control Global
    yUk(:,k) = pinv(J)*E;

    % Transformación mundo -> marco local del UAV
    Juc = [cos(yPsi(k)) sin(yPsi(k)); -sin(yPsi(k)) cos(yPsi(k))];
    R_1 = [Juc zeros(2,2); zeros(2,2) eye(2)];

    %% Referencia cinemática final
    % yUk está en marco inercial. Se transforma al marco local
    yUcLocal(:,k) = R_1*yUk(:,k);

    % Filtrado de la referencia cinemática local
    if k == 1
        yUcFil(:,k) = yUcLocal(:,k);
    else
        alpha_uc = 0.35;
        yUcFil(:,k) = alpha_uc*yUcLocal(:,k) + (1-alpha_uc)*yUcFil(:,k-1);
    end

    %% Compensador Dinamico corregido (solo XY)
    Xp = R_1*[yXip(:,k); yPsip(k)];   % velocidad real en marco local
    yXpLocal(:,k) = Xp;

    % Error dinámico en el mismo marco local
    e_v = Xp - yUcFil(:,k);
    e_v_xy = e_v(1:2);

    if UsarCompensadorDinamico
        % Integrador con fuga
        ErrorAcum_xy = (1 - lambda_i*deltat)*ErrorAcum_xy + e_v_xy*deltat;

        % Superficie deslizante
        s_xy(:,k) = e_v_xy + Lambda_xy*ErrorAcum_xy;

        % Derivada de la referencia de velocidad realmente seguida
        if k == 1
            UcDot_xy = zeros(2,1);
            UcDotFil_xy(:,k) = UcDot_xy;
        else
            UcDot_xy = (yUcFil(1:2,k) - yUcFil(1:2,k-1))/deltat;
            UcDotFil_xy(:,k) = alpha_udot*UcDot_xy + (1-alpha_udot)*UcDotFil_xy(:,k-1);
        end

        if UsarAdaptacionST
            % Ley de adaptación tipo regresor (estilo: Theta = -Gamma*Phi'*s)
            % Parametrización: Theta = [k1_x; k2_x; k1_y; k2_y]
            Phi = [ (UcDotFil_xy(1,k) - Lambda_xy(1,1)*e_v_xy(1)), Xp(1), 0, 0; ...
                    0, 0, (UcDotFil_xy(2,k) - Lambda_xy(2,2)*e_v_xy(2)), Xp(2)];

            Gamma_st = diag([gamma1, gamma2, gamma1, gamma2]);
            Theta = -Gamma_st * (Phi' * s_xy(:,k));

            k1_vec = [K1_xy(1,1); K1_xy(2,2)] + deltat*[Theta(1); Theta(3)];
            k2_vec = [K2_xy(1,1); K2_xy(2,2)] + deltat*[Theta(2); Theta(4)];

            k1_vec = min(max(k1_vec, K1_min), K1_max);
            k2_vec = min(max(k2_vec, K2_min), K2_max);

            K1_xy = diag(k1_vec);
            K2_xy = diag(k2_vec);
        end

        % Super-twisting estándar
        sqrt_s_xy = sqrt(abs(s_xy(:,k))) .* sign(s_xy(:,k));
        nu_xy = nu_xy - K2_xy*sign(s_xy(:,k))*deltat;
        u_st_xy = -K1_xy*sqrt_s_xy + nu_xy;

        % Pseudoaceleración comandada
        a_cmd_xy = UcDotFil_xy(:,k) - Lambda_xy*e_v_xy + u_st_xy;

        % Entrada final tipo velocidad corregida
        yULocal(1:2,k) = yUcFil(1:2,k) + tau_d*a_cmd_xy;

        % Z y yaw pasan directo
        yULocal(3,k) = yUcFil(3,k);
        yULocal(4,k) = yUcFil(4,k);

        % Para plots
        s(1:2,k) = s_xy(:,k);
        s(3,k)   = 0;
        s(4,k)   = 0;

        % Guardado de “ganancias” para visualización
        Gan_U(:,k) = [K1_xy(1,1); K1_xy(2,2); 0; 0];
        Gan_V(:,k) = [K2_xy(1,1); K2_xy(2,2); 0; 0];

    else
        % Sin compensador dinámico: usar referencia cinemática filtrada
        yULocal(:,k) = yUcFil(:,k);

        % Compatibilidad con plots
        Gan_U(:,k) = [0; 0; 0; 0];
        Gan_V(:,k) = [0; 0; 0; 0];
        s(:,k) = 0;
    end

    %% Accion de Control Local
    %     Juc = [cos(yPsi(k)) sin(yPsi(k)); -sin(yPsi(k)) cos(yPsi(k))];
    %     yUc(1:2,k) = Juc*[yU(1,k);yU(2,k)];
    %     yUc(3:4,k) = [yU(3,k);0*yU(4,k)];
    if(~hay_joy)
        quad1(1, yULocal(:,k));
    else
        [axes, buttons,~] = read(joy);
        if(buttons(1))
            finalCamino = true; % Botón A, fin de camino, termina la simulación
            yTFind = k;
        end
        quad1(1,[-2*axes(5) -2*axes(4) -axes(2) -axes(1)]);
    end

    % Guardado consistente en marco mundo para análisis posterior
    yUWorld(:,k)  = R_1\yULocal(:,k);
    yUcWorld(:,k) = R_1\yUcFil(:,k);

    % Mantengo compatibilidad con el resto del script
    yU(:,k)  = yUWorld(:,k);
    yUc(:,k) = yUcWorld(:,k);
    if(norm(wind))
        yWind(:,k) = wind';
        wind1(1,1,wind);
        wind1(2,1,wind);
        wind1(3,wind);
    else
        yWind(:,k) = [0;0;0];
        wind1(1,1,[0 0 0]);
        wind1(2,1,[0 0 0]);
        wind1(3,[0 0 0]);
    end

    if(mod(k,20)==0)
        fprintf(repmat('\b', 1, lastsize));
        lastsize = fprintf('Time = %3.2f [s] - Path = %3.2f [%%]',k*deltat,k/length(xd)*100);
    end

    if(k == length(xd))
        finalCamino = true; %fin de camino, terminar la simulaci�n
        yTFind = k;
    end

    % Graficar en L�nea %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    delete(H1);

    if(payload >= 0.001)
        delete(H2);
        delete(H3);
        delete(H4);
        r = yRope(1:3:40,:,k);  %ver que coincida justo en el �ltimo eslab�n el arreglo
        %         r = yRope(:,:,k);  %ver que coincida justo en el �ltimo eslab�n el arreglo
    end

    x = [yQuad(1,k) yQuad(2,k) yQuad(3,k) yQuad(13,k) yQuad(14,k) yQuad(15,k)];
    H1 = Nquadrotor(1,[x(1),x(2),x(3),x(4),x(5),x(6)]);

    if(payload >= 0.001)
        H2 = line(r(:,1),r(:,2),r(:,3)+0.0025);
        H2.Color = colorCuerda;
        H2.LineWidth = 1.5;

        H3 = line([r(1,1) x(1)],[r(1,2) x(2)],[r(1,3) x(3)]);
        H3.Color = colorquad;
        H3.LineWidth = 2;

        dirBoxT = squeeze(r(end-1,:)-r(end,:))/norm(squeeze(r(end-1,:)-r(end,:)));
        moverBox = dirBoxT*0;
        %     moverBox = dirBoxT*tam_carga/2; %Box translation
        if(yXil(3,k) < tam_carga/2)
            H4=cube_plot([r(end,1)-moverBox(1),r(end,2)-moverBox(2),tam_carga/2],tam_carga,tam_carga,tam_carga,colorBox);
        else
            H4=cube_plot([r(end,1)-moverBox(1),r(end,2)-moverBox(2),r(end,3)-moverBox(3)-tam_carga/2],tam_carga,tam_carga,tam_carga,colorBox);
        end
        rotate(H4,[1 0 0],rad2deg(-asin(dirBoxT(2))),[r(end,1)-moverBox(1),r(end,2)-moverBox(2),r(end,3)-moverBox(3)]);
        rotate(H4,[0 1 0],rad2deg(asin(dirBoxT(1))),[r(end,1)-moverBox(1),r(end,2)-moverBox(2),r(end,3)-moverBox(3)]);
        addpoints(H6,yXil(1,k),yXil(2,k),yXil(3,k));
    end
    addpoints(H5,x(1),x(2),x(3));
    addpoints(H7,xd(k),yd(k),zd(k));
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ant_Uk = yUk(:,k);
    drawnow;

    k=k+1;
end
%%
fprintf('\n');
if(~finalCamino)
    yTFind = k-1;
end

quad1(1,[0 0 0 0]);
simulation1(1);

if(hay_joy)
    close(joy)
end

%% Cálculo de métricas de oscilación de la carga
v_payload = yXip(:,1:yTFind) - yHp(:,1:yTFind);
eV_payload = yXidp(:,1:yTFind) - v_payload;
RMSE_vel_payload = sqrt(mean(sum(eV_payload.^2,1)));
%%
RMSE_pos = sqrt(1/length(yH(1,1:yTFind))*sum(yH(1,1:yTFind).^2+yH(2,1:yTFind).^2));
fprintf('\nRMSE Posicion Carga: %s\n', RMSE_pos);
%RMSE_vel = sqrt(1/length(yHp(1,1:yTFind))*sum(yHp(1,1:yTFind).^2+yHp(2,1:yTFind).^2+yHp(3,1:yTFind).^2));
fprintf('\nRMSE Error de Control Velocidad: %s\n', RMSE_vel_payload);
RMSE_ErrorControl = sqrt(1/length(yExil(1,1:yTFind))*sum(yExil(1,1:yTFind).^2+yExil(2,1:yTFind).^2+yExil(3,1:yTFind).^2));
fprintf('\nRMSE Error de Control: %s\n', RMSE_ErrorControl);

% Ángulos de oscilación: norma de h en cada instante (no de la serie temporal)
hx = yH(1,1:yTFind);
hy = yH(2,1:yTFind);
hz = yH(3,1:yTFind);
normH = sqrt(hx.^2 + hy.^2 + hz.^2 + 1e-12);
r_xy = sqrt(hx.^2 + hy.^2 + 1e-12);
rho = sqrt(hx.^2 + hy.^2);
% vU y vL: velocidades UAV y payload (3xN)
vrel = yHp(:,1:yTFind);
hhat  = [hx;hy;hz] ./ normH;
dotv  = sum(vrel .* hhat, 1);
vperp = vrel - hhat .* dotv;
vperp_norm = sqrt(sum(vperp.^2,1));

% alpha: inclinación desde la vertical (0 = carga bajo dron)
alpha_l = atan2(r_xy, hz);
% vperp_norm : vector 1xN con ||v_perp(t)||
vperp_rms = sqrt(mean(sum(vperp_norm.^2,1)));
vperp_peak = max(abs(vperp_norm));

RMSE_alpha = sqrt(mean(alpha_l(1:yTFind).^2));
fprintf('\nRMS_alpha: %s\n', RMSE_alpha);
max_alpha = max(abs(alpha_l(1:yTFind)));
fprintf('\nMax alpha: %s\n', max_alpha);

rho_rms = sqrt(mean(rho.^2));
max_rho  = max(abs(rho));
fprintf('\nRMS_rho: %s\n', rho_rms);
fprintf('\nmax(Rho(t)): %s\n', max_rho);

fprintf('\nRMS_velT: %s\n', vperp_rms);
fprintf('\nmax(Vel_Tangencial): %s\n', vperp_peak);

%% Calculo Metricas para el Compensador Dinamico
% Error dinámico consistente: ambas señales en marco local
TrackingErrorDyn = yXpLocal(:,1:yTFind) - yUcFil(:,1:yTFind);

RMSE_vx   = sqrt(mean(TrackingErrorDyn(1,:).^2));
RMSE_vy   = sqrt(mean(TrackingErrorDyn(2,:).^2));
RMSE_vz   = sqrt(mean(TrackingErrorDyn(3,:).^2));
RMSE_vpsi = sqrt(mean(TrackingErrorDyn(4,:).^2));

fprintf('\nRMSE evx   = %.6f', RMSE_vx);
fprintf('\nRMSE evy   = %.6f', RMSE_vy);
fprintf('\nRMSE evz   = %.6f', RMSE_vz);
fprintf('\nRMSE evpsi = %.6f\n', RMSE_vpsi);

LinAncho = 1.5;
FntSize = 12;
colors = [0 0.45 0.74;   % azul
    0.85 0.33 0.10; % naranja
    0.47 0.67 0.19; % verde
    0.93 0.69 0.13; % amarillo-marrón
    0.49 0.18 0.56];% morado
%% Guardar ganancias adaptativas finales
if GuardarGananciasFinales
    % Obtener valores finales de Ku y Kv
    Ku_final = Ku;
    Kv_final = Kv;

    % Guardar en archivo .mat
    save(NombreArchivoGanancias, 'Ku_final', 'Kv_final', '-v7.3');
    fprintf('\nGanancias adaptativas guardadas en: %s\n', NombreArchivoGanancias);
    fprintf('  Ku final: [%.4f, %.4f, %.4f, %.4f]\n', diag(Ku_final));
    fprintf('  Kv final: [%.4f, %.4f, %.4f, %.4f]\n', diag(Kv_final));
    fprintf('  Para usar estas ganancias en la proxima simulacion, configure CargarGananciasPrevias = true\n');
end
% pause(0.5); % Para darle tiempo a que finalice la simulaci�n en C++
%% Plot Figures
figure('Position',[50 50 1200 600]);

subplot(4,1,1); hold on; grid on;
plot(tt(1:yTFind), yExil(1,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\tilde{X}$ [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,2); hold on; grid on;
plot(tt(1:yTFind), yExil(2,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\tilde{Y}$ [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,3); hold on; grid on;
plot(tt(1:yTFind), yExil(3,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\tilde{Z}$ [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,4); hold on; grid on;
plot(tt(1:yTFind), yEpsi(1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\tilde{\psi}$ [rad]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
%%
TrackingError = [yXip;yPsip] - yU;
figure('Position',[50 50 1200 600]);
subplot 411
plot(tt(1:yTFind), TrackingError(1,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\dot{\tilde{X}}$ [m/s]','Interpreter','Latex','FontSize',FntSize);
grid;
subplot 412
plot(tt(1:yTFind), TrackingError(2,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\dot{\tilde{Y}}$ [m/s]','Interpreter','Latex','FontSize',FntSize);
grid;
subplot 413
plot(tt(1:yTFind), TrackingError(3,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\dot{\tilde{Z}}$ [m/s]','Interpreter','Latex','FontSize',FntSize);
grid;
subplot 414
plot(tt(1:yTFind), TrackingError(4,1:yTFind),'k','LineWidth',LinAncho, 'Color', colors(1,:));
ylabel('$\dot{\tilde{\psi}}$ [rad/s]','Interpreter','Latex','FontSize',FntSize);
grid;
%% Angul de oscilacion \alpha y velocidad tangencial relativa carga-drone
win = max(5, round(0.5/mean(diff(tt(1:yTFind))))); % ventana ~0.5 s (ajustable)
rho_rms_mov = sqrt(movmean(rho.^2, win));

figure('Position',[50 50 1200 600]);
subplot(3,1,1); hold on; grid on;
plot(tt(1:yTFind), alpha_l(1:yTFind), 'LineWidth', LinAncho, 'Color', colors(1,:));
ylabel('$\alpha$ [rad]','Interpreter','Latex','FontSize',FntSize);
legend('$\alpha_\ell$','Interpreter','Latex','FontSize',FntSize);

subplot(3,1,2); hold on; grid on;
% plot(tt(1:yTFind), beta_l(1:yTFind), 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), rho, 'LineWidth', LinAncho, 'Color', colors(1,:));
% plot(tt(1:yTFind), rho_rms_mov, 'LineWidth', LinAncho, 'Color', colors(2,:));
ylabel('$\rho$ [m]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
legend('$\rho(t)$','Interpreter','Latex','FontSize',FntSize);

subplot(3,1,3); hold on; grid on;
plot(tt(1:yTFind), vperp_norm, 'LineWidth', LinAncho, 'Color', colors(1,:));
% plot(tt(1:yTFind), rho_rms_mov, 'LineWidth', LinAncho, 'Color', colors(2,:));
ylabel('$||v_\perp||$ [m/s]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
legend('$\rho(t)$','Interpreter','Latex','FontSize',FntSize);
%%
figure('Position',[50 50 1200 600]);

subplot(2,1,1); hold on; grid on;
plot(tt(1:yTFind), yXid(1,1:yTFind), '--', 'LineWidth', LinAncho, 'Color', colors(2,:));
plot(tt(1:yTFind), yXil(1,1:yTFind), '-',  'LineWidth', LinAncho, 'Color', colors(1,:));
ylabel('$x_\ell$ [m]','Interpreter','Latex','FontSize',FntSize);
legend('$x_d$','$x_\ell$','Interpreter','Latex','FontSize',FntSize);

subplot(2,1,2); hold on; grid on;
plot(tt(1:yTFind), yXid(2,1:yTFind), '--', 'LineWidth', LinAncho, 'Color', colors(2,:));
plot(tt(1:yTFind), yXil(2,1:yTFind), '-',  'LineWidth', LinAncho, 'Color', colors(1,:));
ylabel('$y_\ell$ [m]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
legend('$y_d$','$y_\ell$','Interpreter','Latex','FontSize',FntSize);

%% Figura Plot 3D
fig=figure;
% fig.WindowState = 'maximized';
s = get(0, 'ScreenSize');
fig.Position = [s(3)*0.125 s(4)*0.125 s(3)*0.75 s(4)*0.75];
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid;
% axis image;
% view(2);
% view(3);
%     view([-90,0]); %view -Y-Z
%     view([0,0]); %view X-Z
%     view([-180,0]); %view -X-Z

plot3(xd,yd,zd,'--r');
scale=0.573;  %2 normal size
colorFrenteQuad=[1 0 1];  %magenta
color_matlab = get(groot,'defaultAxesColorOrder');
colorquad = color_matlab(1,:);
Nquadrotor_parametros(1,scale,colorFrenteQuad,colorquad);
colorBox = [0.38 0.1 0.035];
colorCuerda = [0.5430 0.2695 0.0742];

H1 = Nquadrotor(1,[0 0 0 0 0 0]);
vent_des=frameRate*6;
if(payload >= 0.001)
    H2 = line(0,0,0);
    H3 = line(0,0,0);
    H4 = cube_plot([0,0,0],.1,.1,.1,'r');
    H6 = animatedline('MaximumNumPoints',vent_des,'Color',colorBox,'LineWidth',1.5);
end
H5 = animatedline('MaximumNumPoints',vent_des,'Color',colorquad,'LineWidth',1.5);
H7 = animatedline('MaximumNumPoints',1,'LineWidth',2,'Color',[1 0 0],'Marker','s','MarkerSize',5);
% x = zeros(1,6);
% r = zeros(10,3);
view(45,20);
axis image;
%%
figure('Position',[50 50 1200 600]);
% Variable de interés: posición de la carga o del dron según ControlCarga
if ControlCarga
    yX_plot = yXil;   % posición de la carga [3 x N]
    leg_actual = '$x_\ell$';
    leg_actualY = '$y_\ell$';
    leg_actualZ = '$z_\ell$';
    leg_actualPsi = '$\psi_\ell$';
    %tit_pos = 'Posiciones X,Y,Z (trayectoria de la carga)';
else
    yX_plot = yQuad(1:3,1:length(tt));  % posición del dron
    leg_actual = '$x$';
%     tit_pos = 'Posiciones X,Y,Z (trayectoria del dron)';
end

subplot(4,1,1); hold on; grid on;
plot(tt(1:yTFind), yXid(1,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), yX_plot(1,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
legend('$x_d$', leg_actual, 'Interpreter','Latex','FontSize',FntSize);
% title(tit_pos,'FontSize',FntSize);
ylabel('x [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,2); hold on; grid on;
plot(tt(1:yTFind), yXid(2,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), yX_plot(2,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
legend('$y_d$', leg_actualY, 'Interpreter','Latex','FontSize',FntSize);
ylabel('y [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,3); hold on; grid on;
plot(tt(1:yTFind), yXid(3,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), yX_plot(3,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
legend('$z_d$', leg_actualZ, 'Interpreter','Latex','FontSize',FntSize);
ylabel('z [m]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,4); hold on; grid on;
plot(tt(1:yTFind), yPsid(1,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), yQuad(15,1:yTFind), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
legend('$\psi_d$', leg_actualPsi, 'Interpreter','Latex','FontSize',FntSize);
ylabel('$\psi$ [rad]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
%% Parametros Ku
figure('Position',[50 50 1200 600]);
subplot 211; hold on; grid on;
plot(tt(1:yTFind), Gan_U(1,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), Gan_U(2,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(2,:));
plot(tt(1:yTFind), Gan_U(3,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(3,:));
plot(tt(1:yTFind), Gan_U(4,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(4,:));
legend('$Ku_{1}$','$Ku_{2}$','$Ku_{3}$','$Ku_{4}$','Interpreter','Latex');
subplot 212; hold on; grid on;
plot(tt(1:yTFind), Gan_V(1,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), Gan_V(2,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(2,:));
plot(tt(1:yTFind), Gan_V(3,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(3,:));
plot(tt(1:yTFind), Gan_V(4,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(4,:));
legend('$Kv_{1}$','$Kv_{2}$','$Kv_{3}$','$Kv_{4}$','Interpreter','Latex');

%% Superficie deslizante
figure('Position',[50 50 1200 600]);
hold on; grid on;
plot(tt(1:yTFind), s(1,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(1,:));
plot(tt(1:yTFind), s(2,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(2,:));
plot(tt(1:yTFind), s(3,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(3,:));
plot(tt(1:yTFind), s(4,1:yTFind), 'LineWidth', LinAncho, 'Color', colors(4,:));
legend('$s_x$','$s_y$','$s_z$','$s_{psi}$','Interpreter','Latex')

%% Evoluacion de la referncia vs la posición real
figure('Position',[50 50 1200 600]);
subplot 311; hold on; grid on;
plot(yXid(1,1:yTFind),'r');
plot(yQuad(1,1:yTFind),'k');
plot(yXil(1,1:yTFind));
legend('$x_d$','$x$', '$x_\ell$','Interpreter','Latex');
title('Posiciones X,Y,Z y Yaw');
ylabel('X [m]');

subplot 312; hold on; grid on;
plot(yXid(2,1:yTFind),'r');
plot(yQuad(2,1:yTFind),'k');
plot(yXil(2,1:yTFind));
ylabel('Y [m]');

subplot 313; hold on; grid on;
plot(yXid(3,1:yTFind),'r');
plot(yQuad(3,1:yTFind),'k');
plot(yXil(3,1:yTFind));
ylabel('Z [m]');
%%
figure('Position',[50 50 1200 600]);
subplot 311; hold on; grid on;
plot(yXidp(1,1:yTFind),'r');
plot(yQuad(4,1:yTFind),'k');
plot(yHp(1,1:yTFind)+yQuad(4,1:yTFind));
legend('$\dot{x}_d$','$\dot{x}$', '$\dot{x}_\ell$','Interpreter','Latex');
title('Velocidades X,Y,Z y Yaw');
ylabel('$\dot{\mathbf{x}}$ [m]','Interpreter','Latex','FontSize',FntSize);

subplot 312; hold on; grid on;
plot(yXidp(2,1:yTFind),'r');
plot(yQuad(5,1:yTFind),'k');
plot(yHp(2,1:yTFind)+yQuad(5,1:yTFind));
ylabel('$\dot{\mathbf{y}}$ [m]','Interpreter','Latex','FontSize',FntSize);

subplot 313; hold on; grid on;
plot(yXidp(3,1:yTFind),'r');
plot(yQuad(6,1:yTFind),'k');
plot(yHp(3,1:yTFind)+yQuad(6,1:yTFind));
ylabel('$\dot{\mathbf{z}}$ [m]','Interpreter','Latex','FontSize',FntSize);
%% Error seguimiento de velocidad en el marco de referencia local

figure('Position',[50 50 1200 600]);
subplot(4,1,1); grid on; hold on;
plot(tt(1:yTFind), TrackingErrorDyn(1,:), 'LineWidth', LinAncho, 'Color', colors(1,:));
ylabel('$e_{vx}$ [m/s]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,2); grid on; hold on;
plot(tt(1:yTFind), TrackingErrorDyn(2,:), 'LineWidth', LinAncho, 'Color', colors(2,:));
ylabel('$e_{vy}$ [m/s]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,3); grid on; hold on;
plot(tt(1:yTFind), TrackingErrorDyn(3,:), 'LineWidth', LinAncho, 'Color', colors(3,:));
ylabel('$e_{vz}$ [m/s]','Interpreter','Latex','FontSize',FntSize);

subplot(4,1,4); grid on; hold on;
plot(tt(1:yTFind), TrackingErrorDyn(4,:), 'LineWidth', LinAncho, 'Color', colors(4,:));
ylabel('$e_{v\psi}$ [rad/s]','Interpreter','Latex','FontSize',FntSize);
xlabel('Time [s]','Interpreter','Latex','FontSize',FntSize);
%% Exportar figuras y datos para el paper
if ExportarResultadosPaper
    if ~exist(CarpetaResultados,'dir'), mkdir(CarpetaResultados); end
    prefijo = sprintf('%s_%s', mfilename, datestr(now,'yyyymmdd_HHMMSS'));
    % Guardar figuras (todas las abiertas tras los plots)
    figs = findobj('Type','figure');
    % La primera es la animación 3D, el resto son las de resultados
    for i = 2:length(figs)
        f = figs(i);
        nombre = sprintf('%s_fig%d', prefijo, i-1);
        saveas(f, fullfile(CarpetaResultados, [nombre '.fig']));
        exportgraphics(f, fullfile(CarpetaResultados, [nombre '.png']), 'Resolution', 150);
    end
    % Guardar datos para reproducir gráficos o incluir en paper
    archivo_mat = fullfile(CarpetaResultados, [prefijo '_datos.mat']);
    save(archivo_mat, 'tt', 'yTFind', 'deltat', 'yQuad', 'yXil', 'yRope', 'yExil', 'yEpsi', ...
        'yU', 'yUc', 'yUk', 'yXid', 'yXidp', 'yPsid', 'yH', 'yHp', 's', ...
        'RMSE_pos', 'RMSE_ErrorControl', 'RMSE_vel_payload', ...
        'RMSE_alpha', 'max_alpha', 'rho_rms', 'max_rho', ...
        'vperp_rms', 'vperp_peak', ...
        'Gan_U', 'Gan_V', 'Ku', 'Kv', 'xd', 'yd', 'zd', 'pd', ...
        'lcuerda', 'payload', 'tam_carga', 'frameRate', 'TrayectoriaDefault', '-v7.3');
    fprintf('\nResultados exportados en %s:\n  %s\n  %s\n', CarpetaResultados, ...
        sprintf('%d figuras', length(figs)-1), archivo_mat);
end
%% Grabo en archivo todos los datos
% formatSpec = 'Sim_%d_Payload_%#1.1f_Cuerda_%s.mat';
% str = sprintf(formatSpec,payload,l,datestr(now,30));
% save(str,'-v7.3');

%% Terminar Simulaci�n
robot = java.awt.Robot;
robot.keyPress(java.awt.event.KeyEvent.VK_F10);
pause(0.1);
robot.keyRelease(java.awt.event.KeyEvent.VK_F10);