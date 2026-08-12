%% =========================================================================
%  Comparativa_SMC_Kusznir_vs_STA_Propuesto.m
%
%  Objetivo:
%    Comparar dos controladores para transporte de carga suspendida con
%    UAV, en el MISMO simulador (SimQuad1.exe + rope/quad/wind MEX):
%
%      (A) BASELINE  : SMC de primer orden con boundary layer
%                      inspirado en Kusznir & Smoczek (2020),
%                      "Sliding mode-based control of a UAV with a
%                       cable-suspended load".
%      (B) PROPUESTO : STA compuesto adaptativo (paper actual),
%                      con capa cinemática payload-aware y adaptación.
%
%  Ambos controladores comparten:
%     - Trayectoria de referencia (idéntica).
%     - Modelo dinámico del simulador (misma cuerda, misma carga, mismo κ_u/κ_v).
%     - Lazo cinemático externo (compensación cuasi-estática de swing),
%       para AISLAR el efecto del lazo dinámico (SMC vs. STA+adapt).
%     - Observador ST del cable (h, dot(h)).
%
%  Salidas:
%     - Tabla de métricas: RMSE_x, RMSE_v, RMS_alpha, Max_alpha, RMS_rho,
%       Max_rho, RMS_vT, Max_vT + índices de chattering (RMS de dU/dt).
%     - Figuras comparativas: 3D, error de posición X/Y/Z/psi, oscilaciones
%       α/ρ/‖v_⊥‖, superficies s_x/s_y, acción de control u_corr y evolución
%       de ganancias adaptativas del STA.
%     - Todo se exporta a ResultadosPaper/Comp_SMC_vs_STA_<timestamp>_*.
%
%  Nota: es un script; requiere MATLAB R2016b+ (funciones locales al final).
% =========================================================================

clearvars;
clearvars global;
close all;
clc;

%% =========================================================================
%% 1. Parámetros compartidos por ambos controladores
%% =========================================================================
T         = 100;        % tiempo total de simulación [s]
frameRate = 50;         % 50 Hz => deltat = 0.02 s (igual al paper)
deltat    = 1/frameRate;

% Trayectoria: 1 = círculo/elipse ascendente (misma que la Sec. 3.1 del paper)
TrayectoriaDefault = 1;
[xd,yd,zd,pd,xdp,ydp,zdp,pdp,xdpp,ydpp,zdpp,pdpp] = ...
    ReferenciasCarga_ST(T,deltat,TrayectoriaDefault);

% ----- Parámetros físicos (Tabla sim_params del paper) -----
lcuerda           = 2;        % largo del cable [m]
payload           = 0.1;      % masa de la carga [kg]
tam_carga         = 0.1;      % lado del cubo [m]
spring_friction   = 0.6;
spring_constant   = 5000;
air_friction      = 0.01/20;
ground_repel      = 50/20;
ground_slide      = 0.1/20;
ground_absorption = 1/20;
nObs              = 0;
tieneLaser        = 0;
ErrorInicialMax   = 0;
wind              = [0 0 0];  % escenario nominal (sin viento)

% ----- Ganancias del lazo cinemático (idénticas para ambos) -----
Ktraj   = diag([4.0 4.0 2.0]);
Ktrajt  = diag([0.35 0.35 0.35]);
Kvload  = diag([0.45 0.45 0.18]);
Kvloadt = diag([0.90 0.90 0.60]);
Kpsi    = 1.0;
Kpsit   = 0.5;
Kh      = diag([1.8 1.8]);
Kdh     = diag([0.6 0.6]);

filtro_exil = 0.35;
filtro_epsi = 0.25;

% ----- Parámetros comunes del lazo interno -----
Lambda_xy  = diag([0.08 0.08]);   % superficie s = e_v + Lambda*int(e_v)
tau_d      = 0.05;                % pseudoaceleración -> corrección velocidad
alpha_udot = 0.15;                % filtro d/dt de la referencia de vel
alpha_uc   = 0.35;                % filtro de la referencia cinemática local
lambda_i   = 0.03;                % fuga del integrador (anti-windup suave)

% ----- Parámetros del STA (propuesto) -----
K1_STA_init = diag([0.5 0.5]);    % ganancia sqrt(|s|)
K2_STA_init = diag([0.75 0.75]);  % ganancia integral
gamma1      = 0.03;
gamma2      = 0.04;
K1_min = [0.08; 0.08];   K1_max = [1.45; 1.45];
K2_min = [0.10; 0.10];   K2_max = [1.60; 1.60];

% ----- Parámetros del SMC de primer orden (Kusznir 2020, baseline) -----
%  u_smc = -K_SMC * s / (|s| + phi_bl)      (boundary layer suave)
%  Se usa K_SMC ligeramente mayor que K1_STA para compensar la falta del
%  término integral (segundo orden) del STA. phi_bl controla el chattering
%  (típico en la literatura de UAV+carga con SMC: 0.02–0.10).
K_SMC   = diag([1.20 1.20]);
phi_bl  = 0.05;

% ----- Condición inicial (idéntica para ambas corridas) -----
rng(0);
yXo    = zeros(4,1);
yXo(1) = xd(1);
yXo(2) = yd(1);
yXo(3) = zd(1) + lcuerda;         % dron arriba de la carga
yXo(4) = pd(1);

time_sim = T*2;                   % margen para el ejecutable C++
tt = 0:deltat:T;
N  = length(tt);

% ----- Opciones -----
UsarPlot3DEnVivo = false;         % true: animación 3D (lento); false: rápido
ExportarResultados = true;
CarpetaResultados  = 'ResultadosPaper';

%% =========================================================================
%% 2. Arranque del simulador (una sola vez)
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
Modes    = {'SMC_Kusznir', 'STA_Proposed'};
Results  = struct();

for iCtrl = 1:length(Modes)
    Mode = Modes{iCtrl};
    fprintf('\n\n========================================================\n');
    fprintf('  Corrida %d/%d: %s\n', iCtrl, length(Modes), Mode);
    fprintf('========================================================\n');

    % Reinicio del simulador para cada corrida
    quad1(1,[0 0 0 0]);
    simulation1(1);
    pause(0.5);
    simulation1([1 time_sim payload tam_carga lcuerda ...
        spring_friction spring_constant air_friction ground_repel ...
        ground_slide ground_absorption nObs], yXo, tieneLaser, [],[]);
    pause(1);

    % Buffers
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
    u_corr_all = zeros(2,N);   % acción de control del lazo interno
    UcDotFil_xy = zeros(2,N);
    K1_hist = zeros(2,N);
    K2_hist = zeros(2,N);
    yTsim   = zeros(1,N);

    % Estados internos
    exil_ant     = zeros(3,1);
    exilp_ant    = zeros(3,1);
    epsi_ant     = 0;
    h_ref_h_ant  = [0;0];
    h_ref_dot_fil = [0;0];
    ErrorAcum_xy = [0;0];
    nu_xy        = [0;0];
    K1_STA       = K1_STA_init;
    K2_STA       = K2_STA_init;
    Tsoft        = 3.0;

    % Figura 3D opcional
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

        % Lectura del simulador
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

        %% Errores de posición y velocidad de la carga
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

        %% Error de yaw
        yEpsi(k) = yPsid(k) - yPsi(k);
        while abs(yEpsi(k))>pi
            if     yEpsi(k)> pi, yEpsi(k) = yEpsi(k) - 2*pi;
            elseif yEpsi(k)<-pi, yEpsi(k) = yEpsi(k) + 2*pi;
            end
        end
        if(k==1), epsi_ant = yEpsi(k); end
        yEpsi(k) = filtro_epsi*yEpsi(k) + (1-filtro_epsi)*epsi_ant;
        epsi_ant = yEpsi(k);

        %% Observador ST diferenciador para (h, dot h)
        L1_obs = 3.0;   L2_obs = 4.5;
        e_obs   = yH(:,k) - yHe(:,k);
        yHe_dot = yHp(:,k) - L1_obs*sqrt(abs(e_obs)).*sign(e_obs);
        yHp_dot = -L2_obs*sign(e_obs);
        if k < N
            yHe(:,k+1) = yHe(:,k) + yHe_dot*deltat;
        end
        yHp(:,k) = yHp(:,k) + yHp_dot*deltat;

        %% =============== LAZO CINEMÁTICO (COMÚN) ============================
        v_base = yXidp(:,k) ...
            + Ktraj *tanh(Ktrajt *yExil(:,k)) ...
            + Kvload*tanh(Kvloadt*yExilp(:,k));

        yComp(:,k) = zeros(3,1);
        if payload >= 0.001
            % Compensación cinemática payload-aware (Kinematic II del paper)
            a_ref_h = yXidpp(1:2,k);
            h_ref_h = (lcuerda/9.81) * a_ref_h;
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
            comp_xy = -Kh*eh_h - Kdh*ehp_h;
            comp_max = 0.9;
            comp_xy  = comp_max * tanh(comp_xy/comp_max);
            yComp(:,k) = [comp_xy; 0];
        end

        % Rampa de arranque suave
        sigma_t     = 1 - exp(-yTsim(k)/Tsoft);
        v_base      = sigma_t * v_base;
        yComp(:,k)  = sigma_t * yComp(:,k);

        yTraj_vc(:,k) = v_base + yComp(:,k);
        yPsi_vc(k)    = yPsidp(k) + Kpsi*tanh(Kpsit*yEpsi(k));

        Jl = horzcat(eye(3),[0;0;0]);
        Jp = zeros(1,4); Jp(1,4) = 1;
        J  = [Jl; Jp];
        E  = [yTraj_vc(:,k); yPsi_vc(k)];
        yUk(:,k) = pinv(J)*E;

        % Marco mundo -> marco local UAV
        Juc = [cos(yPsi(k)) sin(yPsi(k)); -sin(yPsi(k)) cos(yPsi(k))];
        R_1 = [Juc zeros(2,2); zeros(2,2) eye(2)];
        yUcLocal(:,k) = R_1*yUk(:,k);

        % Filtro de referencia cinemática local
        if k == 1
            yUcFil(:,k) = yUcLocal(:,k);
        else
            yUcFil(:,k) = alpha_uc*yUcLocal(:,k) + (1-alpha_uc)*yUcFil(:,k-1);
        end

        %% =============== LAZO DINÁMICO (SWITCH POR CONTROLADOR) =============
        Xp = R_1*[yXip(:,k); yPsip(k)];
        yXpLocal(:,k) = Xp;
        e_v    = Xp - yUcFil(:,k);
        e_v_xy = e_v(1:2);

        % Superficie deslizante común: s = e_v + Lambda * int(e_v)
        ErrorAcum_xy = (1 - lambda_i*deltat)*ErrorAcum_xy + e_v_xy*deltat;
        s_xy_k       = e_v_xy + Lambda_xy*ErrorAcum_xy;

        % Derivada filtrada de la referencia
        if k == 1
            UcDot_xy         = [0;0];
            UcDotFil_xy(:,k) = [0;0];
        else
            UcDot_xy         = (yUcFil(1:2,k) - yUcFil(1:2,k-1))/deltat;
            UcDotFil_xy(:,k) = alpha_udot*UcDot_xy + ...
                               (1-alpha_udot)*UcDotFil_xy(:,k-1);
        end

        switch Mode
            % --------------------------------------------------------
            %  (B) PROPUESTO: STA compuesto adaptativo (paper)
            % --------------------------------------------------------
            case 'STA_Proposed'
                UsarAdaptacionST = true;
                if UsarAdaptacionST
                    % Ley de adaptación estilo regresor
                    Phi = [ (UcDotFil_xy(1,k) - Lambda_xy(1,1)*e_v_xy(1)), ...
                             Xp(1),   0,       0; ...
                             0,       0,   (UcDotFil_xy(2,k) - ...
                                            Lambda_xy(2,2)*e_v_xy(2)),  Xp(2)];
                    Gamma_st = diag([gamma1, gamma2, gamma1, gamma2]);
                    Theta = -Gamma_st * (Phi' * s_xy_k);

                    k1_vec = [K1_STA(1,1); K1_STA(2,2)] + ...
                             deltat*[Theta(1); Theta(3)];
                    k2_vec = [K2_STA(1,1); K2_STA(2,2)] + ...
                             deltat*[Theta(2); Theta(4)];
                    k1_vec = min(max(k1_vec, K1_min), K1_max);
                    k2_vec = min(max(k2_vec, K2_min), K2_max);
                    K1_STA = diag(k1_vec);
                    K2_STA = diag(k2_vec);
                end

                % Super-twisting estándar
                sqrt_s   = sqrt(abs(s_xy_k)) .* sign(s_xy_k);
                nu_xy    = nu_xy - K2_STA*sign(s_xy_k)*deltat;
                u_corr_xy = -K1_STA*sqrt_s + nu_xy;

                K1_hist(:,k) = [K1_STA(1,1); K1_STA(2,2)];
                K2_hist(:,k) = [K2_STA(1,1); K2_STA(2,2)];

            % --------------------------------------------------------
            %  (A) BASELINE: SMC 1er orden con boundary layer
            %      Kusznir & Smoczek (2020)
            % --------------------------------------------------------
            case 'SMC_Kusznir'
                % Saturación suave: sign(s) sustituida por s/(|s|+phi_bl)
                sat_s = s_xy_k ./ (abs(s_xy_k) + phi_bl);
                u_corr_xy = -K_SMC * sat_s;

                K1_hist(:,k) = diag(K_SMC);
                K2_hist(:,k) = [phi_bl; phi_bl];
        end

        % Pseudo-aceleración -> corrección de velocidad
        a_cmd_xy = UcDotFil_xy(:,k) - Lambda_xy*e_v_xy + u_corr_xy;
        yULocal(1:2,k) = yUcFil(1:2,k) + tau_d*a_cmd_xy;
        yULocal(3,k)   = yUcFil(3,k);
        yULocal(4,k)   = yUcFil(4,k);

        s_all(1:2,k)     = s_xy_k;
        u_corr_all(:,k)  = u_corr_xy;

        %% Envío al simulador
        quad1(1, yULocal(:,k));
        yUWorld(:,k)  = R_1\yULocal(:,k);
        yUcWorld(:,k) = R_1\yUcFil(:,k);
        yU(:,k)  = yUWorld(:,k);
        yUc(:,k) = yUcWorld(:,k);

        % Viento (nominal)
        if(norm(wind))
            wind1(1,1,wind); wind1(2,1,wind); wind1(3,wind);
        else
            wind1(1,1,[0 0 0]); wind1(2,1,[0 0 0]); wind1(3,[0 0 0]);
        end

        % Feedback en consola
        if(mod(k,20)==0)
            fprintf(repmat('\b', 1, lastsize));
            lastsize = fprintf('    Time = %6.2f [s] - Path = %5.2f [%%]', ...
                               k*deltat, k/N*100);
        end

        if(k == length(xd))
            finalCamino = true;
            yTFind = k;
        end

        % Detección de inestabilidad (crash) para no arrastrar estados
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

    %% Guardar resultados de esta corrida
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
%% 4. Impresión de tabla de métricas
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
    prefix = sprintf('Comp_SMC_vs_STA_%s', datestr(now,'yyyymmdd_HHMMSS'));

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
    save(archivo_mat, 'Results', 'K_SMC', 'phi_bl', ...
        'K1_STA_init', 'K2_STA_init', 'Lambda_xy', 'tau_d', ...
        'lcuerda', 'payload', 'tam_carga', 'frameRate', ...
        'TrayectoriaDefault', 'xd', 'yd', 'zd', 'pd', '-v7.3');

    fprintf('\nResultados exportados en %s/:\n', CarpetaResultados);
    fprintf('  %d figuras y %s\n', length(figs), archivo_mat);
end

%% =========================================================================
%% 7. Cierre del simulador
%% =========================================================================
quad1(1,[0 0 0 0]);
simulation1(1);
fprintf('\nSimulacion comparativa finalizada.\n');


%% =========================================================================
%% ================== FUNCIONES LOCALES ====================================
%% =========================================================================
function R = compute_metrics(R)
    %COMPUTE_METRICS  Calcula todas las métricas de tracking y oscilación
    tt     = R.tt;
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

    % Chattering en la acción de corrección (lazo interno aislado)
    du = diff(R.u_corr, 1, 2) / deltat;
    R.TV_u_corr  = sum(abs(diff(R.u_corr, 1, 2)), 2);
    R.RMS_du_corr = sqrt(mean(du.^2, 2));

    % Chattering en el comando final enviado al UAV (más realista)
    duL = diff(R.yULocal(1:2,:), 1, 2) / deltat;
    R.TV_uL   = sum(abs(diff(R.yULocal(1:2,:), 1, 2)), 2);
    R.RMS_duL = sqrt(mean(duL.^2, 2));
end


function print_metrics_table(Results)
    %PRINT_METRICS_TABLE  Imprime tabla comparativa en consola
    R1 = Results.SMC_Kusznir;
    R2 = Results.STA_Proposed;

    fprintf('\n\n===============================================================================\n');
    fprintf('   Métricas comparativas: SMC (Kusznir 2020) vs STA compuesto adaptativo\n');
    fprintf('===============================================================================\n');
    fprintf(' %-24s %-16s %-16s %-12s\n', ...
        'Métrica', 'SMC (Kusznir)', 'STA (Propuesto)', 'Mejora');
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
    fprintf(' Chattering  RMS(dU/dt) comando final X : SMC = %8.3f | STA = %8.3f\n', ...
        R1.RMS_duL(1), R2.RMS_duL(1));
    fprintf(' Chattering  RMS(dU/dt) comando final Y : SMC = %8.3f | STA = %8.3f\n', ...
        R1.RMS_duL(2), R2.RMS_duL(2));
    fprintf(' Total variation |U| X                  : SMC = %8.3f | STA = %8.3f\n', ...
        R1.TV_uL(1),  R2.TV_uL(1));
    fprintf(' Total variation |U| Y                  : SMC = %8.3f | STA = %8.3f\n', ...
        R1.TV_uL(2),  R2.TV_uL(2));
    fprintf('===============================================================================\n\n');
end


function plot_comparison(Results)
    %PLOT_COMPARISON  Figuras comparativas listas para el paper
    R1 = Results.SMC_Kusznir;
    R2 = Results.STA_Proposed;

    colSMC = [0.85 0.33 0.10];    % naranja
    colSTA = [0.00 0.45 0.74];    % azul
    colRef = [0.20 0.20 0.20];    % gris oscuro
    lw = 1.5;
    fs = 11;

    % ---------- Fig 1: Trayectoria 3D ----------
    figure('Position',[50 50 900 700]); hold on; grid on;
    plot3(R1.yXid(1,:), R1.yXid(2,:), R1.yXid(3,:), '--', ...
        'Color', colRef, 'LineWidth', 1.2);
    plot3(R1.yXil(1,:), R1.yXil(2,:), R1.yXil(3,:), '-', ...
        'Color', colSMC, 'LineWidth', lw);
    plot3(R2.yXil(1,:), R2.yXil(2,:), R2.yXil(3,:), '-', ...
        'Color', colSTA, 'LineWidth', lw);
    legend('Reference $x_d$', 'SMC (Kusznir 2020)', 'STA (Proposed)', ...
        'Interpreter','Latex','Location','best','FontSize',fs);
    xlabel('x [m]','Interpreter','Latex'); 
    ylabel('y [m]','Interpreter','Latex'); 
    zlabel('z [m]','Interpreter','Latex');
    title('Payload 3D trajectory','Interpreter','Latex','FontSize',fs+1);
    view(45,20); axis image;

    % ---------- Fig 2: Errores de posición X, Y, Z y yaw ----------
    figure('Position',[50 50 1200 700]);
    labs = {'$\tilde x$ [m]', '$\tilde y$ [m]', ...
            '$\tilde z$ [m]', '$\tilde \psi$ [rad]'};
    for i = 1:4
        subplot(4,1,i); hold on; grid on;
        if i <= 3
            plot(R1.tt, R1.yExil(i,:), 'LineWidth', lw, 'Color', colSMC);
            plot(R2.tt, R2.yExil(i,:), 'LineWidth', lw, 'Color', colSTA);
        else
            plot(R1.tt, R1.yEpsi, 'LineWidth', lw, 'Color', colSMC);
            plot(R2.tt, R2.yEpsi, 'LineWidth', lw, 'Color', colSTA);
        end
        ylabel(labs{i},'Interpreter','Latex','FontSize',fs);
        if i == 1
            legend('SMC (Kusznir 2020)', 'STA (Proposed)', ...
                'Interpreter','Latex','FontSize',fs,'Location','best');
            title('Payload tracking errors','Interpreter','Latex','FontSize',fs+1);
        end
        if i == 4, xlabel('Time [s]','Interpreter','Latex'); end
    end

    % ---------- Fig 3: Métricas de oscilación alpha, rho, |v_perp| ----------
    figure('Position',[50 50 1200 700]);
    subplot(3,1,1); hold on; grid on;
    plot(R1.tt, R1.alpha, 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.alpha, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\alpha$ [rad]','Interpreter','Latex','FontSize',fs);
    legend('SMC (Kusznir 2020)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Oscillation-related performance metrics','Interpreter','Latex','FontSize',fs+1);

    subplot(3,1,2); hold on; grid on;
    plot(R1.tt, R1.rho, 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.rho, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\rho$ [m]','Interpreter','Latex','FontSize',fs);

    subplot(3,1,3); hold on; grid on;
    plot(R1.tt, R1.vperp_norm, 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.vperp_norm, 'LineWidth', lw, 'Color', colSTA);
    ylabel('$\|v_\perp\|$ [m/s]','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % ---------- Fig 4: Superficies deslizantes s_x, s_y ----------
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.s(1,:), 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.s(1,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$s_x$','Interpreter','Latex','FontSize',fs);
    legend('SMC (Kusznir 2020)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Sliding variable evolution','Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.s(2,:), 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.s(2,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$s_y$','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % ---------- Fig 5: Acción del lazo interno (chattering) ----------
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.u_corr(1,:), 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.u_corr(1,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$u_{\mathrm{corr},x}$','Interpreter','Latex','FontSize',fs);
    legend('SMC (Kusznir 2020)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Inner-loop correction action (chattering diagnostic)', ...
        'Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.u_corr(2,:), 'LineWidth', lw, 'Color', colSMC);
    plot(R2.tt, R2.u_corr(2,:), 'LineWidth', lw, 'Color', colSTA);
    ylabel('$u_{\mathrm{corr},y}$','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % ---------- Fig 6: Trayectoria X vs t y Y vs t (payload) ----------
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.yXid(1,:), '--', 'Color', colRef, 'LineWidth', 1.2);
    plot(R1.tt, R1.yXil(1,:), '-',  'Color', colSMC, 'LineWidth', lw);
    plot(R2.tt, R2.yXil(1,:), '-',  'Color', colSTA, 'LineWidth', lw);
    ylabel('$x_\ell$ [m]','Interpreter','Latex','FontSize',fs);
    legend('Reference', 'SMC (Kusznir 2020)', 'STA (Proposed)', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Payload horizontal tracking','Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.yXid(2,:), '--', 'Color', colRef, 'LineWidth', 1.2);
    plot(R1.tt, R1.yXil(2,:), '-',  'Color', colSMC, 'LineWidth', lw);
    plot(R2.tt, R2.yXil(2,:), '-',  'Color', colSTA, 'LineWidth', lw);
    ylabel('$y_\ell$ [m]','Interpreter','Latex','FontSize',fs);
    xlabel('Time [s]','Interpreter','Latex');

    % ---------- Fig 7: Ganancias adaptativas del STA vs SMC fijas ----------
    figure('Position',[50 50 1200 500]);
    subplot(2,1,1); hold on; grid on;
    plot(R1.tt, R1.K1_hist(1,:), '--', 'Color', colSMC, 'LineWidth', lw);
    plot(R1.tt, R1.K1_hist(2,:), '--', 'Color', colSMC*0.6, 'LineWidth', lw);
    plot(R2.tt, R2.K1_hist(1,:), '-',  'Color', colSTA, 'LineWidth', lw);
    plot(R2.tt, R2.K1_hist(2,:), '-',  'Color', colSTA*0.6, 'LineWidth', lw);
    ylabel('$K_1$','Interpreter','Latex','FontSize',fs);
    legend('SMC $K_{x}$','SMC $K_{y}$','STA $K_{1x}$','STA $K_{1y}$', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    title('Inner-loop gains (fixed vs adaptive)', ...
        'Interpreter','Latex','FontSize',fs+1);

    subplot(2,1,2); hold on; grid on;
    plot(R1.tt, R1.K2_hist(1,:), '--', 'Color', colSMC, 'LineWidth', lw);
    plot(R2.tt, R2.K2_hist(1,:), '-',  'Color', colSTA, 'LineWidth', lw);
    plot(R2.tt, R2.K2_hist(2,:), '-',  'Color', colSTA*0.6, 'LineWidth', lw);
    ylabel('$K_2$ (STA) / $\phi_{bl}$ (SMC)','Interpreter','Latex','FontSize',fs);
    legend('SMC $\phi_{bl}$','STA $K_{2x}$','STA $K_{2y}$', ...
        'Interpreter','Latex','FontSize',fs,'Location','best');
    xlabel('Time [s]','Interpreter','Latex');
end
