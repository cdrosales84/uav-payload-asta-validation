%% =========================================================================
%  Comparativa_Klausen_Backstepping_vs_STA_Propuesto.m
%
%  Objetivo:
%    Comparar dos controladores full-cascade en el MISMO simulador
%    (SimQuad1.exe + rope/quad/wind MEX):
%
%      (A) BASELINE  : Backstepping no lineal con swing damping,
%                      inspirado en Klausen, Fossen & Johansen (2017),
%                      "Nonlinear Control with Swing Damping of a
%                       Multirotor UAV with Suspended Load",
%                       J. Intell. Robot. Syst. (Springer).
%
%      (B) PROPUESTO : Kinematic II (payload-aware) + STA compuesto
%                      adaptativo (este paper).
%
%  A diferencia de la comparación contra SMC (que solo cambiaba el
%  lazo interno), aquí las dos arquitecturas son distintas de punta a
%  punta y se compara el sistema completo:
%
%     - Klausen: outer position/velocity backstepping + swing damping
%                usando (q, q_dot); NO adaptación, NO sliding mode,
%                NO tanh; comando de velocidad directo al simulador.
%
%     - Propuesto: outer Kinematic II con saturaciones tanh y
%                   compensación cuasi-estática; inner STA con
%                   adaptación (regressor phi'*s).
%
%  Se conservan idénticos: simulador, trayectoria, condiciones
%  iniciales, propiedades físicas, observador ST del cable, filtro
%  de la referencia local y rampa de arranque suave sigma(t).
%
%  Referencia teórica de las ganancias del baseline:
%     Small-signal equivalence con las ganancias del STA:
%       K1_kl  <->  Ktraj * Ktrajt   ~ 4.0 * 0.35 = 1.4
%       K2_kl  <->  Kvload * Kvloadt ~ 0.45 * 0.9 = 0.4
%       Kq_kl  <->  Kh * lcuerda     ~ 1.8 * 2.0 = 3.6
%       Kw_kl  <->  Kdh * lcuerda    ~ 0.6 * 2.0 = 1.2
%
%  Salidas:
%     - Tabla de métricas: RMSE_x, RMSE_v, RMS_alpha, Max_alpha,
%       RMS_rho, Max_rho, RMS_vT, Max_vT + chattering RMS(dU/dt) y
%       Total Variation TV(U).
%     - Figuras comparativas: 3D, error de posición, oscilaciones
%       alpha/rho/||v_perp||, superficies s (solo STA), acción de
%       comando yULocal, evolución de ganancias.
%     - Todo se exporta a ResultadosPaper/Comp_Klausen_vs_STA_<ts>_*.
%
% =========================================================================

clearvars;
clearvars global;
close all;
clc;

%% =========================================================================
%% 0. Selector de escenario
%%    Cambiar entre las tres opciones para reproducir cada corrida sin
%%    duplicar codigo. Cada escenario exporta a una subcarpeta distinta.
%%
%%      'nominal'    -> sin viento, payload 0.1 kg (escenario base).
%%      'viento'     -> viento persistente [1.5 0 0], payload 0.1 kg.
%%      'parametrico'-> sin viento, payload 0.3 kg (drift de masa 3x).
%%                       Este es el escenario donde la adaptacion online
%%                       del STA deberia romper a Klausen (ganancias fijas).
%% =========================================================================
escenario = 'parametrico';   % opciones: 'nominal' | 'viento' | 'parametrico'

switch lower(escenario)
    case 'nominal'
        wind_case    = [0 0 0];
        payload_case = 0.1;
        tag_case     = 'Nominal';
    case 'viento'
        wind_case    = [1.5 0 0];   % rafaga lateral persistente
        payload_case = 0.1;
        tag_case     = 'Viento';
    case 'parametrico'
        wind_case    = [0 0 0];
        payload_case = 0.3;         % masa 3x la nominal
        tag_case     = 'Parametrico';
    otherwise
        error('escenario debe ser ''nominal'' | ''viento'' | ''parametrico''.');
end
fprintf(['\n>>> ESCENARIO SELECCIONADO: %s\n' ...
         '    wind    = [%.2f %.2f %.2f]\n' ...
         '    payload = %.2f kg\n\n'], ...
        tag_case, wind_case(1), wind_case(2), wind_case(3), payload_case);

%% =========================================================================
%% 1. Parámetros compartidos por ambos controladores
%% =========================================================================
T         = 100;
frameRate = 50;
deltat    = 1/frameRate;

TrayectoriaDefault = 1;
[xd,yd,zd,pd,xdp,ydp,zdp,pdp,xdpp,ydpp,zdpp,pdpp] = ...
    ReferenciasCarga_ST(T,deltat,TrayectoriaDefault);

lcuerda           = 2;
payload           = payload_case;
tam_carga         = 0.1;
spring_friction   = 0.6;
spring_constant   = 5000;
air_friction      = 0.01/20;
ground_repel      = 50/20;
ground_slide      = 0.1/20;
ground_absorption = 1/20;
nObs              = 0;
tieneLaser        = 0;
ErrorInicialMax   = 0;
wind              = wind_case;

Kpsi  = 1.0;
Kpsit = 0.5;

filtro_exil = 0.35;
filtro_epsi = 0.25;
alpha_uc    = 0.35;

Tsoft = 3.0;
g     = 9.81;

%% ----- Parámetros del STA propuesto (idénticos al paper) -----
Ktraj_STA   = diag([4.0 4.0 2.0]);
Ktrajt_STA  = diag([0.35 0.35 0.35]);
Kvload_STA  = diag([0.45 0.45 0.18]);
Kvloadt_STA = diag([0.90 0.90 0.60]);
Kh_STA      = diag([1.8 1.8]);
Kdh_STA     = diag([0.6 0.6]);

Lambda_xy   = diag([0.08 0.08]);
tau_d       = 0.05;
alpha_udot  = 0.15;
lambda_i    = 0.03;

K1_STA_init = diag([0.5 0.5]);
K2_STA_init = diag([0.75 0.75]);
gamma1 = 0.03;
gamma2 = 0.04;
K1_min = [0.08; 0.08];   K1_max = [1.45; 1.45];
K2_min = [0.10; 0.10];   K2_max = [1.60; 1.60];

%% ----- Parámetros del Backstepping de Klausen 2017 -----
%  Ganancias elegidas para tener equivalencia de pequeña señal con
%  las ganancias del STA (ver encabezado). Al no haber tanh, estos
%  valores actúan linealmente sobre el error total; si el transitorio
%  es muy agresivo, reducir K1_kl.
K1_kl = diag([1.4 1.4 1.0]);   % ganancia error posición carga
K2_kl = diag([0.4 0.4 0.3]);   % ganancia error velocidad carga
Kq_kl = diag([3.6 3.6]);       % swing damping en q_h
Kw_kl = diag([1.2 1.2]);       % swing damping en q_dot_h

%% ----- Condición inicial (idéntica para ambas corridas) -----
rng(0);
yXo    = zeros(4,1);
yXo(1) = xd(1);
yXo(2) = yd(1);
yXo(3) = zd(1) + lcuerda;
yXo(4) = pd(1);

time_sim = T*2;
tt = 0:deltat:T;
N  = length(tt);

UsarPlot3DEnVivo   = false;
ExportarResultados = true;
CarpetaResultados  = 'ResultadosPaper';

%% =========================================================================
%% 2. Arranque del simulador (una vez)
%% =========================================================================
ExeName = 'SimQuad1.exe';
[~,msg] = system(['tasklist /fi "ImageName eq ' ExeName '"']);
IsOpen  = contains(msg,ExeName);
quad1(1,[0 0 0 0]);
if(IsOpen)
    simulation1(1);
    pause(0.5);
else
    system('SimQuad1.exe &');
    pause(1);
end

%% =========================================================================
%% 3. Corrida secuencial de los dos controladores
%% =========================================================================
Modes    = {'Klausen_Backstepping', 'STA_Proposed'};
Results  = struct();

for iCtrl = 1:length(Modes)
    Mode = Modes{iCtrl};
    fprintf('\n\n=========================================================\n');
    fprintf('  Corrida %d/%d: %s\n', iCtrl, length(Modes), Mode);
    fprintf('=========================================================\n');

    quad1(1,[0 0 0 0]);
    simulation1(1);
    pause(0.5);
    simulation1([1 time_sim payload tam_carga lcuerda ...
        spring_friction spring_constant air_friction ground_repel ...
        ground_slide ground_absorption nObs], yXo, tieneLaser, [],[]);
    pause(1);

    % ----- Buffers -----
    yQuad  = zeros(24,N);
    yRope  = zeros(40,3,N);
    yForce = zeros(3,N);
    yXi   = zeros(3,N);   yXip  = zeros(3,N);
    yXil  = zeros(3,N);   yXilp = zeros(3,N);
    yXid  = zeros(3,N);   yXidp = zeros(3,N);   yXidpp = zeros(3,N);
    yPsid = zeros(1,N);   yPsidp = zeros(1,N);
    yH    = zeros(3,N);   yHe   = zeros(3,N);   yHp   = zeros(3,N);
    yExil = zeros(3,N);   yEpsi = zeros(1,N);   yExilp = zeros(3,N);
    yPsi  = zeros(1,N);   yPsip = zeros(1,N);
    yU        = zeros(4,N);   yUc     = zeros(4,N);   yUk = zeros(4,N);
    yUcLocal  = zeros(4,N);   yUcWorld = zeros(4,N);
    yULocal   = zeros(4,N);   yUWorld  = zeros(4,N);
    yUcFil    = zeros(4,N);   yXpLocal = zeros(4,N);
    yTraj_vc  = zeros(3,N);   yPsi_vc  = zeros(1,N);
    yComp     = zeros(3,N);
    s_all      = zeros(4,N);
    u_corr_all = zeros(2,N);
    UcDotFil_xy = zeros(2,N);
    K1_hist = zeros(2,N);
    K2_hist = zeros(2,N);
    yTsim   = zeros(1,N);

    % ----- Estados internos -----
    exil_ant     = zeros(3,1);
    exilp_ant    = zeros(3,1);
    epsi_ant     = 0;
    h_ref_h_ant  = [0;0];
    h_ref_dot_fil = [0;0];
    ErrorAcum_xy = [0;0];
    nu_xy        = [0;0];
    K1_STA       = K1_STA_init;
    K2_STA       = K2_STA_init;
    R_1          = eye(4);

    if UsarPlot3DEnVivo
        figLive = figure('Name', ['3D live: ' Mode]);
        set(figLive, 'Position', [50 50 900 700]);
        plot3(xd,yd,zd,'--r'); hold on; grid on; view(45,20); axis image;
        xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    end

    finalCamino = false;
    yTFind = N;
    k = 1;
    tic;
    lastsize = 0;

    while (~finalCamino) && (k <= N)
        while (toc < deltat); end
        tic;

        if(~simulation1)
            finalCamino = true;
            yTFind = k-1;
            break;
        end

        % ---------- Lectura del simulador ----------
        yQuad(:,k)   = quad1(1);
        yRope(:,:,k) = rope1(1);
        yForce(:,k)  = force1(1);

        yXi(:,k)  = yQuad(1:3,k);
        yXip(:,k) = yQuad(4:6,k);
        yPsi(k)   = yQuad(15,k);
        yPsip(k)  = yQuad(23,k);
        yTsim(k)  = k*deltat;

        if(payload >= 0.001)
            yXil(:,k) = [yRope(end,1,k), yRope(end,2,k), yRope(end,3,k)];
            yH(:,k)   = yXi(:,k) - yXil(:,k);
        end

        yXid(:,k)   = [xd(k);   yd(k);   zd(k)];
        yXidp(:,k)  = [xdp(k);  ydp(k);  zdp(k)];
        yXidpp(:,k) = [xdpp(k); ydpp(k); zdpp(k)];
        yPsid(k)    = pd(k);
        yPsidp(k)   = pdp(k);

        % ---------- Errores payload ----------
        yExil(:,k) = yXid(:,k) - yXil(:,k);
        v_payload  = yXip(:,k) - yHp(:,k);
        yExilp(:,k) = yXidp(:,k) - v_payload;

        if(k==1)
            exil_ant  = yExil(:,k);
            exilp_ant = yExilp(:,k);
        end
        yExil(:,k)  = filtro_exil*yExil(:,k)  + (1-filtro_exil)*exil_ant;
        yExilp(:,k) = filtro_exil*yExilp(:,k) + (1-filtro_exil)*exilp_ant;
        exil_ant    = yExil(:,k);
        exilp_ant   = yExilp(:,k);

        % ---------- Error yaw ----------
        yEpsi(k) = yPsid(k) - yPsi(k);
        while abs(yEpsi(k))>pi
            if     yEpsi(k)> pi, yEpsi(k) = yEpsi(k) - 2*pi;
            elseif yEpsi(k)<-pi, yEpsi(k) = yEpsi(k) + 2*pi;
            end
        end
        if(k==1), epsi_ant = yEpsi(k); end
        yEpsi(k) = filtro_epsi*yEpsi(k) + (1-filtro_epsi)*epsi_ant;
        epsi_ant = yEpsi(k);

        % ---------- Observador ST del cable (compartido) ----------
        L1_obs = 3.0;   L2_obs = 4.5;
        e_obs   = yH(:,k) - yHe(:,k);
        yHe_dot = yHp(:,k) - L1_obs*sqrt(abs(e_obs)).*sign(e_obs);
        yHp_dot = -L2_obs*sign(e_obs);
        if k < N
            yHe(:,k+1) = yHe(:,k) + yHe_dot*deltat;
        end
        yHp(:,k) = yHp(:,k) + yHp_dot*deltat;

        % ---------- Sub-lazo yaw (compartido) ----------
        yPsi_vc(k) = yPsidp(k) + Kpsi*tanh(Kpsit*yEpsi(k));

        % ---------- Rampa de arranque suave (compartida) ----------
        sigma_t = 1 - exp(-yTsim(k)/Tsoft);

        %% ================== SWITCH DE LEY DE CONTROL =====================
        switch Mode
            %% ----------------------------------------------------------
            %% (A) BASELINE : Klausen 2017 backstepping + swing damping
            %% ----------------------------------------------------------
            case 'Klausen_Backstepping'
                % Step 1: Position + velocity backstepping (nivel carga)
                v_star = yXidp(:,k) + K1_kl*yExil(:,k) + K2_kl*yExilp(:,k);

                % Step 2: Swing damping usando q y q_dot horizontales
                if payload >= 0.001
                    q_h     = yH(1:2,k)  / lcuerda;
                    q_dot_h = yHp(1:2,k) / lcuerda;
                    q_d_h   = yXidpp(1:2,k) / g;     % q_h cuasi-estático
                    e_q_h   = q_h - q_d_h;
                    v_swing_h = -Kq_kl*e_q_h - Kw_kl*q_dot_h;
                else
                    v_swing_h = [0;0];
                    e_q_h     = [0;0];
                end

                yTraj_vc(:,k) = v_star + [v_swing_h; 0];
                yComp(:,k)    = [v_swing_h; 0];
                yTraj_vc(:,k) = sigma_t * yTraj_vc(:,k);

                % Armado 4-vector y transformación a marco local
                Jl = horzcat(eye(3),[0;0;0]);
                Jp = zeros(1,4); Jp(1,4) = 1;
                J  = [Jl; Jp];
                E  = [yTraj_vc(:,k); yPsi_vc(k)];
                yUk(:,k) = pinv(J)*E;

                Juc = [cos(yPsi(k)) sin(yPsi(k)); -sin(yPsi(k)) cos(yPsi(k))];
                R_1 = [Juc zeros(2,2); zeros(2,2) eye(2)];
                yUcLocal(:,k) = R_1*yUk(:,k);

                if k == 1
                    yUcFil(:,k) = yUcLocal(:,k);
                else
                    yUcFil(:,k) = alpha_uc*yUcLocal(:,k) + ...
                                  (1-alpha_uc)*yUcFil(:,k-1);
                end

                % Klausen NO tiene lazo interno adaptativo:
                % la referencia filtrada se envía directo al simulador.
                yULocal(:,k) = yUcFil(:,k);

                Xp = R_1*[yXip(:,k); yPsip(k)];
                yXpLocal(:,k) = Xp;

                % Diagnósticos consistentes para plotting
                s_all(:,k) = 0;
                u_corr_all(:,k) = [0;0];

                K1_hist(:,k) = [K1_kl(1,1); K1_kl(2,2)];
                K2_hist(:,k) = [Kq_kl(1,1); Kq_kl(2,2)];

            %% ----------------------------------------------------------
            %% (B) PROPUESTO : Kinematic II + Adaptive STA
            %% ----------------------------------------------------------
            case 'STA_Proposed'
                % ---- Outer: Kinematic II payload-aware ----
                v_base = yXidp(:,k) ...
                    + Ktraj_STA *tanh(Ktrajt_STA *yExil(:,k)) ...
                    + Kvload_STA*tanh(Kvloadt_STA*yExilp(:,k));

                yComp(:,k) = zeros(3,1);
                if payload >= 0.001
                    a_ref_h = yXidpp(1:2,k);
                    h_ref_h = (lcuerda/g) * a_ref_h;
                    if k == 1
                        h_ref_dot_h  = [0;0];
                        h_ref_h_ant  = h_ref_h;
                        h_ref_dot_fil = [0;0];
                    else
                        h_ref_dot_raw = (h_ref_h - h_ref_h_ant)/deltat;
                        alpha_hrefdot = 0.60;
                        h_ref_dot_fil = alpha_hrefdot*h_ref_dot_raw + ...
                                        (1-alpha_hrefdot)*h_ref_dot_fil;
                        h_ref_dot_h   = h_ref_dot_fil;
                        h_ref_h_ant   = h_ref_h;
                    end

                    H_h   = yH(1:2,k);
                    Hp_h  = yHp(1:2,k);
                    eh_h  = H_h  - h_ref_h;
                    ehp_h = Hp_h - h_ref_dot_h;
                    comp_xy = -Kh_STA*eh_h - Kdh_STA*ehp_h;
                    comp_max = 0.9;
                    comp_xy  = comp_max * tanh(comp_xy/comp_max);
                    yComp(:,k) = [comp_xy; 0];
                end

                v_base     = sigma_t * v_base;
                yComp(:,k) = sigma_t * yComp(:,k);
                yTraj_vc(:,k) = v_base + yComp(:,k);

                Jl = horzcat(eye(3),[0;0;0]);
                Jp = zeros(1,4); Jp(1,4) = 1;
                J  = [Jl; Jp];
                E  = [yTraj_vc(:,k); yPsi_vc(k)];
                yUk(:,k) = pinv(J)*E;

                Juc = [cos(yPsi(k)) sin(yPsi(k)); -sin(yPsi(k)) cos(yPsi(k))];
                R_1 = [Juc zeros(2,2); zeros(2,2) eye(2)];
                yUcLocal(:,k) = R_1*yUk(:,k);

                if k == 1
                    yUcFil(:,k) = yUcLocal(:,k);
                else
                    yUcFil(:,k) = alpha_uc*yUcLocal(:,k) + ...
                                  (1-alpha_uc)*yUcFil(:,k-1);
                end

                % ---- Inner: STA adaptativo ----
                Xp = R_1*[yXip(:,k); yPsip(k)];
                yXpLocal(:,k) = Xp;
                e_v    = Xp - yUcFil(:,k);
                e_v_xy = e_v(1:2);
                ErrorAcum_xy = (1 - lambda_i*deltat)*ErrorAcum_xy + e_v_xy*deltat;
                s_xy_k       = e_v_xy + Lambda_xy*ErrorAcum_xy;

                if k == 1
                    UcDot_xy         = [0;0];
                    UcDotFil_xy(:,k) = [0;0];
                else
                    UcDot_xy         = (yUcFil(1:2,k) - yUcFil(1:2,k-1))/deltat;
                    UcDotFil_xy(:,k) = alpha_udot*UcDot_xy + ...
                                       (1-alpha_udot)*UcDotFil_xy(:,k-1);
                end

                Phi = [ (UcDotFil_xy(1,k) - Lambda_xy(1,1)*e_v_xy(1)), ...
                         Xp(1),   0,       0; ...
                         0,       0,   (UcDotFil_xy(2,k) - ...
                                        Lambda_xy(2,2)*e_v_xy(2)),  Xp(2)];
                Gamma_st = diag([gamma1, gamma2, gamma1, gamma2]);
                Theta = -Gamma_st * (Phi' * s_xy_k);

                k1_vec = [K1_STA(1,1); K1_STA(2,2)] + deltat*[Theta(1); Theta(3)];
                k2_vec = [K2_STA(1,1); K2_STA(2,2)] + deltat*[Theta(2); Theta(4)];
                k1_vec = min(max(k1_vec, K1_min), K1_max);
                k2_vec = min(max(k2_vec, K2_min), K2_max);
                K1_STA = diag(k1_vec);
                K2_STA = diag(k2_vec);

                sqrt_s   = sqrt(abs(s_xy_k)) .* sign(s_xy_k);
                nu_xy    = nu_xy - K2_STA*sign(s_xy_k)*deltat;
                u_corr_xy = -K1_STA*sqrt_s + nu_xy;

                a_cmd_xy = UcDotFil_xy(:,k) - Lambda_xy*e_v_xy + u_corr_xy;
                yULocal(1:2,k) = yUcFil(1:2,k) + tau_d*a_cmd_xy;
                yULocal(3,k)   = yUcFil(3,k);
                yULocal(4,k)   = yUcFil(4,k);

                s_all(1:2,k)    = s_xy_k;
                u_corr_all(:,k) = u_corr_xy;
                K1_hist(:,k) = [K1_STA(1,1); K1_STA(2,2)];
                K2_hist(:,k) = [K2_STA(1,1); K2_STA(2,2)];
        end

        %% ---------- Envío al simulador ----------
        quad1(1, yULocal(:,k));
        yUWorld(:,k)  = R_1\yULocal(:,k);
        yUcWorld(:,k) = R_1\yUcFil(:,k);
        yU(:,k)  = yUWorld(:,k);
        yUc(:,k) = yUcWorld(:,k);

        if(norm(wind))
            wind1(1,1,wind); wind1(2,1,wind); wind1(3,wind);
        else
            wind1(1,1,[0 0 0]); wind1(2,1,[0 0 0]); wind1(3,[0 0 0]);
        end

        if(mod(k,20)==0)
            fprintf(repmat('\b', 1, lastsize));
            lastsize = fprintf('    Time = %6.2f [s] - Path = %5.2f [%%]', ...
                               k*deltat, k/N*100);
        end

        if(k == length(xd))
            finalCamino = true;
            yTFind = k;
        end

        if any(isnan(yQuad(1:6,k))) || norm(yXi(:,k) - yXid(:,k)) > 30
            fprintf('\n  [!] Inestabilidad detectada en k=%d, abortando corrida.\n',k);
            finalCamino = true;
            yTFind = k;
        end

        if UsarPlot3DEnVivo && mod(k,5) == 0
            figure(figLive);
            plot3(yXi(1,k), yXi(2,k), yXi(3,k), 'b.');
            plot3(yXil(1,k), yXil(2,k), yXil(3,k), 'r.');
            drawnow limitrate;
        end

        k = k + 1;
    end
    fprintf('\n');

    if(~finalCamino), yTFind = k-1; end
    quad1(1,[0 0 0 0]);

    %% Guardar resultados
    R = struct();
    R.Mode      = Mode;
    R.tt        = tt(1:yTFind);
    R.yXid      = yXid(:,1:yTFind);
    R.yXidp     = yXidp(:,1:yTFind);
    R.yXil      = yXil(:,1:yTFind);
    R.yQuad     = yQuad(:,1:yTFind);
    R.yXi       = yXi(:,1:yTFind);
    R.yXip      = yXip(:,1:yTFind);
    R.yH        = yH(:,1:yTFind);
    R.yHp       = yHp(:,1:yTFind);
    R.yExil     = yExil(:,1:yTFind);
    R.yEpsi     = yEpsi(1:yTFind);
    R.yPsi      = yPsi(1:yTFind);
    R.yU        = yU(:,1:yTFind);
    R.yUc       = yUc(:,1:yTFind);
    R.yULocal   = yULocal(:,1:yTFind);
    R.yUcFil    = yUcFil(:,1:yTFind);
    R.yXpLocal  = yXpLocal(:,1:yTFind);
    R.s         = s_all(:,1:yTFind);
    R.u_corr    = u_corr_all(:,1:yTFind);
    R.K1_hist   = K1_hist(:,1:yTFind);
    R.K2_hist   = K2_hist(:,1:yTFind);
    R.yTFind    = yTFind;
    R.deltat    = deltat;

    R = compute_metrics(R);
    Results.(Mode) = R;
    fprintf('  Completado: %s (RMSE_pos=%.4f, RMS_alpha=%.4f, Max_vT=%.4f)\n', ...
        Mode, R.RMSE_pos, R.RMS_alpha, R.Max_vT);
end

%% =========================================================================
%% 4. Tabla comparativa
%% =========================================================================
print_metrics_table(Results);

%% =========================================================================
%% 5. Figuras comparativas
%% =========================================================================
plot_comparison(Results);

%% =========================================================================
%% 6. Exportación
%% =========================================================================
if ExportarResultados
    if ~exist(CarpetaResultados,'dir'), mkdir(CarpetaResultados); end
    prefix = sprintf('Comp_Klausen_vs_STA_%s_%s', tag_case, datestr(now,'yyyymmdd_HHMMSS'));
    figs = findobj('Type','figure');
    for i = 1:length(figs)
        f = figs(i);
        nombre = sprintf('%s_fig%02d', prefix, i);
        try
            saveas(f, fullfile(CarpetaResultados,[nombre '.fig']));
            exportgraphics(f, fullfile(CarpetaResultados,[nombre '.png']), ...
                'Resolution', 150);
        catch ME
            warning('No se pudo exportar figura %d: %s', i, ME.message);
        end
    end
    archivo_mat = fullfile(CarpetaResultados,[prefix '_datos.mat']);
    save(archivo_mat, 'Results', 'K1_kl', 'K2_kl', 'Kq_kl', 'Kw_kl', ...
        'K1_STA_init', 'K2_STA_init', 'Lambda_xy', 'tau_d', ...
        'lcuerda', 'payload', 'tam_carga', 'frameRate', ...
        'TrayectoriaDefault', 'xd', 'yd', 'zd', 'pd', '-v7.3');
    fprintf('\nResultados exportados en %s/:\n', CarpetaResultados);
    fprintf('  %d figuras y %s\n', length(figs), archivo_mat);
end

quad1(1,[0 0 0 0]);
simulation1(1);
fprintf('\nSimulacion comparativa finalizada.\n');


%% =========================================================================
%% ================== FUNCIONES LOCALES ====================================
%% =========================================================================
function R = compute_metrics(R)
    deltat = R.deltat;

    hx = R.yH(1,:); hy = R.yH(2,:); hz = R.yH(3,:);
    normH = sqrt(hx.^2 + hy.^2 + hz.^2 + 1e-12);
    r_xy  = sqrt(hx.^2 + hy.^2 + 1e-12);
    rho_t = sqrt(hx.^2 + hy.^2);
    hhat  = [hx; hy; hz] ./ normH;
    vrel  = R.yHp;
    dotv  = sum(vrel .* hhat, 1);
    vperp = vrel - hhat .* dotv;
    vperp_norm = sqrt(sum(vperp.^2, 1));
    alpha_l = atan2(r_xy, hz);

    R.alpha      = alpha_l;
    R.rho        = rho_t;
    R.vperp_norm = vperp_norm;

    R.RMSE_pos = sqrt(mean(R.yExil(1,:).^2 + R.yExil(2,:).^2 + R.yExil(3,:).^2));
    v_payload  = R.yXip - R.yHp;
    eV         = R.yXidp - v_payload;
    R.RMSE_vel = sqrt(mean(sum(eV.^2, 1)));

    R.RMS_alpha = sqrt(mean(alpha_l.^2));
    R.Max_alpha = max(abs(alpha_l));
    R.RMS_rho   = sqrt(mean(rho_t.^2));
    R.Max_rho   = max(abs(rho_t));
    R.RMS_vT    = sqrt(mean(vperp_norm.^2));
    R.Max_vT    = max(abs(vperp_norm));

    % Chattering en el comando final enviado al UAV
    duL = diff(R.yULocal(1:2,:), 1, 2) / deltat;
    R.TV_uL   = sum(abs(diff(R.yULocal(1:2,:), 1, 2)), 2);
    R.RMS_duL = sqrt(mean(duL.^2, 2));
end


function print_metrics_table(Results)
    R1 = Results.Klausen_Backstepping;
    R2 = Results.STA_Proposed;

    fprintf('\n\n===============================================================================\n');
    fprintf('   Metricas: Klausen 2017 (Backstepping+Swing Damping) vs STA compuesto adaptativo\n');
    fprintf('===============================================================================\n');
    fprintf(' %-24s %-16s %-16s %-12s\n', ...
        'Metrica', 'Klausen 2017', 'STA (Propuesto)', 'Mejora');
    fprintf('-------------------------------------------------------------------------------\n');

    names  = {'RMSE_pos','RMSE_vel','RMS_alpha','Max_alpha', ...
              'RMS_rho','Max_rho','RMS_vT','Max_vT'};
    labels = {'RMSE xtilde [m]',   'RMSE vtilde [m/s]', ...
              'RMS  alpha [rad]',  'Max  alpha [rad]', ...
              'RMS  rho  [m]',     'Max  rho  [m]', ...
              'RMS  v_T  [m/s]',   'Max  v_T  [m/s]'};
    for i = 1:length(names)
        v1  = R1.(names{i});
        v2  = R2.(names{i});
        pct = 100*(v1-v2)/(abs(v1)+eps);
        fprintf(' %-24s %-16.4f %-16.4f %+7.1f %%\n', labels{i}, v1, v2, pct);
    end
    fprintf('-------------------------------------------------------------------------------\n');
    fprintf(' Chattering  RMS(dU/dt) comando final X : Kla = %8.3f | STA = %8.3f\n', ...
        R1.RMS_duL(1), R2.RMS_duL(1));
    fprintf(' Chattering  RMS(dU/dt) comando final Y : Kla = %8.3f | STA = %8.3f\n', ...
        R1.RMS_duL(2), R2.RMS_duL(2));
    fprintf(' Total variation |U| X                  : Kla = %8.3f | STA = %8.3f\n', ...
        R1.TV_uL(1),  R2.TV_uL(1));
    fprintf(' Total variation |U| Y                  : Kla = %8.3f | STA = %8.3f\n', ...
        R1.TV_uL(2),  R2.TV_uL(2));
    fprintf('===============================================================================\n\n');
end


function plot_comparison(Results)
    R1 = Results.Klausen_Backstepping;
    R2 = Results.STA_Proposed;

    colKla = [0.47 0.67 0.19];    % verde: Klausen
    colSTA = [0.00 0.45 0.74];    % azul : STA
    colRef = [0.20 0.20 0.20];
    lw = 1.5;
    fs = 11;

    % --- Fig 1: Trayectoria 3D ---
    figure('Position',[50 50 900 700]); hold on; grid on;
    plot3(R1.yXid(1,:), R1.yXid(2,:), R1.yXid(3,:), '--', ...
        'Color', colRef, 'LineWidth', 1.2);
    plot3(R1.yXil(1,:), R1.yXil(2,:), R1.yXil(3,:), '-', ...
        'Color', colKla, 'LineWidth', lw);
    plot3(R2.yXil(1,:), R2.yXil(2,:), R2.yXil(3,:), '-', ...
        'Color', colSTA, 'LineWidth', lw);
    legend('Reference $x_d$', 'Klausen 2017 (backstepping)', 'STA (Proposed)', ...
        'Interpreter','Latex','Location','best','FontSize',fs);
    xlabel('x [m]','Interpreter','Latex');
    ylabel('y [m]','Interpreter','Latex');
    zlabel('z [m]','Interpreter','Latex');
    title('Payload 3D trajectory','Interpreter','Latex','FontSize',fs+1);
    view(45,20); axis image;

    % --- Fig 2: Errores de posición ---
    figure('Position',[50 50 1200 700]);
    labs = {'$\tilde x$ [m]', '$\tilde y$ [m]', ...
            '$\tilde z$ [m]', '$\tilde \psi$ [rad]'};
    for i = 1:4
        subplot(4,1,i); hold on; grid on;
        if i <= 3
            plot(R1.tt, R1.yExil(i,:), 'LineWidth', lw, 'Color', colKla);
            plot(R2.tt, R2.yExil(i,:), 'LineWidth', lw, 'Color', colSTA);
        else
            plot(R1.tt, R1.yEpsi, 'LineWidth', lw, 'Color', colKla);
            plot(R2.tt, R2.yEpsi, 'LineWidth', lw, 'Color', colSTA);
        end
        ylabel(labs{i},'Interpreter','Latex','FontSize',fs);
        if i == 1
            legend('Klausen 2017 (backstepping)', 'STA (Proposed)', ...
                'Interpreter','Latex','FontSize',fs,'Location','best');
            title('Payload tracking errors','Interpreter','Latex','FontSize',fs+1);
        end
        if i == 4, xlabel('Time [s]','Interpreter','Latex'); end
    end

    % --- Fig 3: Métricas de oscilación ---
    figure('Position',[50 50 1200 700]);
    subplot(3,1,1); hold on; grid on;
    plot(R1.tt, R1.alpha, 'LineWidth', lw, 'Color', colKla);
    plot(R2.tt, R2.alpha, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\alpha$ [rad]','Interpreter','Latex','FontSize',fs);
    legend('Klausen 2017 (backstepping)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Oscillation-related performance metrics','Interpreter','Latex','FontSize',fs+1);

    subplot(3,1,2); hold on; grid on;
    plot(R1.tt, R1.rho, 'LineWidth', lw, 'Color', colKla);
    plot(R2.tt, R2.rho, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\rho$ [m]','Interpreter','Latex','FontSize',fs);

    subplot(3,1,3); hold on; grid on;
    plot(R1.tt, R1.vperp_norm, 'LineWidth', lw, 'Color', colKla);
    plot(R2.tt, R2.vperp_norm, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\|v_\perp\|$ [m/s]','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % --- Fig 4: Comando enviado al UAV (chattering en la salida) ---
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.yULocal(1,:), 'LineWidth', lw, 'Color', colKla);
    plot(R2.tt, R2.yULocal(1,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$U_{x}$','Interpreter','Latex','FontSize',fs);
    legend('Klausen 2017 (backstepping)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Normalized velocity command sent to UAV', ...
        'Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.yULocal(2,:), 'LineWidth', lw, 'Color', colKla);
    plot(R2.tt, R2.yULocal(2,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$U_{y}$','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % --- Fig 5: Trayectoria X, Y del payload ---
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.yXid(1,:), '--', 'Color', colRef, 'LineWidth', 1.2);
    plot(R1.tt, R1.yXil(1,:), '-',  'Color', colKla, 'LineWidth', lw);
    plot(R2.tt, R2.yXil(1,:), '-',  'Color', colSTA, 'LineWidth', lw);
    ylabel('$x_\ell$ [m]','Interpreter','Latex','FontSize',fs);
    legend('Reference', 'Klausen 2017 (backstepping)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Payload horizontal tracking','Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.yXid(2,:), '--', 'Color', colRef, 'LineWidth', 1.2);
    plot(R1.tt, R1.yXil(2,:), '-',  'Color', colKla, 'LineWidth', lw);
    plot(R2.tt, R2.yXil(2,:), '-',  'Color', colSTA, 'LineWidth', lw);
    ylabel('$y_\ell$ [m]','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');
end
