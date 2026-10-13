function cfg = cfg_paper(escenario)
% CFG_PAPER  Todos los parametros del controlador y de los experimentos.
%
%   cfg = cfg_paper()            -> configuracion nominal (circulo, cascada completa, A-STA)
%   cfg = cfg_paper('E4_circ_asta') -> configuracion de un escenario de run_paper_experiments
%
% Cada campo lleva la etiqueta de la ecuacion del manuscrito revisado en la que
% interviene. Esta estructura se guarda junto con cada resultado (.mat), de modo que
% cualquier figura o tabla puede rastrearse hasta los valores exactos usados.
%
% Convenciones:
%   h = x - x_l (Ec. cable_vector). Marco inercial z hacia arriba.
%   El lazo interno trabaja en el marco de yaw del UAV (el simulador recibe
%   velocidades [v_xh v_yh v_zh w_zh] en ese marco, ver Comandos en SimQuad.h).

if nargin < 1 || isempty(escenario), escenario = 'nominal'; end

cfg.version   = 'paper_revision 1.0';
cfg.escenario = escenario;

%% Planta / simulador
cfg.planta.backend      = 'simquad';   % 'simquad' (SimQuad1.exe + mex, Windows) | 'sustituta' (modelo MATLAB, para pruebas)
cfg.planta.T            = 100;         % [s] duracion
cfg.planta.dt           = 0.02;        % [s] periodo de control nominal (50 Hz), Tabla sim_params
cfg.planta.usar_dt_real = true;        % true: dt = diferencia del tiempo del simulador (yQuad(24))
cfg.planta.l            = 2.0;         % [m] largo de la cuerda
cfg.planta.mL           = 0.10;        % [kg] masa de la carga
cfg.planta.tam_carga    = 0.10;        % [m] lado del cubo
cfg.planta.spring_friction   = 0.6;
cfg.planta.spring_constant   = 5000;
cfg.planta.air_friction      = 0.01/20;
cfg.planta.ground_repel      = 50/20;
cfg.planta.ground_slide      = 0.1/20;
cfg.planta.ground_absorption = 1/20;
cfg.planta.grafico_en_linea  = false;  % true: Nquadrotor + cuerda + carga dentro del while de sim_payload_asta
cfg.planta.grafico_cada      = 5;      % redibuja cada N muestras (1 = cada paso; en tiempo real alarga el periodo)
cfg.planta.g            = 9.81;

%% Referencia (Ec. ref_circ / ref_lemn)
cfg.ref.tipo  = 'circulo';   % 'circulo' | 'lemniscata'
cfg.ref.circ  = struct('R', 3, 'Tp', 10, 'z0', 6, 'vz', 0.1);        % x=R sin(2pi t/Tp), y=R cos(.), z=z0+vz t
cfg.ref.lemn  = struct('a', 10, 'b', 7.5, 'c', 0.025*2*pi, 'd', 1, 'z0', 6);

%% Viento (escenario de robustez). Filas: [t_ini t_fin wx wy wz] en m/s
cfg.viento.bloques = zeros(0,5);

%% Lazo externo: controlador cinematico (Secc. kinematic)
cfg.ext.ley    = 'completo';            % 'completo' (Ecs. base_reference, swing_compensation) | 'sin_swing' (solo xd_dot + Ktraj*e)
cfg.ext.Ktraj  = diag([1.8 1.8 0.7]);   % Ec. base_reference
cfg.ext.Kh     = diag([3.0 3.0]);       % Ec. swing_compensation. E2_circ: 4.0 -> 3.0; el modo de cable a 1.42 s crecia (alpha -> 74 deg)
cfg.ext.Kdh    = diag([0.3 0.3]);       % Ec. swing_compensation
cfg.ext.Ts     = 5.0;                   % [s] Ec. final_reference, sigma(t) = 1 - exp(-t/Ts)
cfg.ext.kpsi   = 1.0;                   % Ec. yaw_reference
cfg.ext.kpsit  = 0.5;                   % Ec. yaw_reference
cfg.ext.usar_ff_hdot = true;            % termino [hdot*;0] en v_base (Ec. base_reference)
cfg.ext.seguimiento = 'lineal';          % 'lineal' (Ec. base_reference) | 'abril' (script 7-abr-2026, solo v_base)
cfg.ext.Ktraj_abril = diag([4.0 4.0 2.0]);    % ganancia fuera del tanh del script de abril
cfg.ext.Ktrajt      = diag([0.35 0.35 0.35]); % ganancia dentro del tanh, error de posicion
cfg.ext.Kvload      = diag([0.45 0.45 0.18]); % ganancia fuera del tanh, velocidad de la carga
cfg.ext.Kvloadt     = diag([0.90 0.90 0.60]); % ganancia dentro del tanh, velocidad de la carga

%% Estimador de velocidad del cable (Ecs. h_raw, h_hat_dyn, h_p_filter)
cfg.est.eps1  = 0.14;
cfg.est.K1    = 3.0 * eye(3);
cfg.est.tau_h = 0.06;                   % [s]

%% Filtros de implementacion discreta (Secc. discrete-time implementation)
% Constantes de tiempo en segundos. 0 = filtro desactivado.
% Cada filtro activo se discretiza como y(k) = y(k-1) + a*(x(k) - y(k-1)), a = 1 - exp(-dt/tau).
cfg.filt.tau_ex   = 0;      % error de posicion de la carga (antes filtro_exil)
cfg.filt.tau_epsi = 0;      % error de yaw (antes filtro_epsi)
cfg.filt.tau_vc   = 0;      % comando cinematico en marco local (antes alpha_uc)
cfg.filt.tau_vdot = 0.05;   % derivada numerica de v_d usada en a = vd_dot - beta*v_tilde

%% Lazo interno: compensador dinamico (Secc. dynamic), canales x,y en marco de yaw (n = 2)
cfg.int.ley   = 'asta';                 % 'asta' | 'sta_fijo' | 'smc' (ablacion) | 'ninguno' (solo cinematico)
cfg.int.beta  = 0.08;                   % Ec. superficie s = v_tilde + beta*int(v_tilde)
% K1/K2 = 1.0/1.5 (controller.m) compensaban la inestabilidad del lazo externo con
% Kh/Kdh = 1.8/0.6. Con Kh/Kdh = 3.0/0.3 (Kh > (1+Kdh)*Ktraj) vuelven a 0.5/0.75, que
% reduce el ciclo limite del STA (verificar en SimQuad: ver README).
cfg.int.K1    = [0.35; 0.35];             % Ec. AccionControl, termino K1*sqrt|s|*sign(s)
cfg.int.K2    = [0.525; 0.525];           % Ec. AccionControl, termino int(K2*sign(s))
cfg.int.phi_bl = 0;                     % 0 = sign(s) puro; >0 = sign(s) ~ s/(|s|+phi_bl) (declararlo si se usa)
cfg.int.u_max = 5.0;                    % [m/s] saturacion del comando; integracion condicional de nu
cfg.int.K_smc  = [1.2; 1.2];            % solo ablacion SMC: u = phi*theta0 - K_smc .* s./(|s|+phi_smc)
cfg.int.phi_smc = 0.05;

%% Parametros del modelo (Ec. ModeloLucas reparametrizada)
% theta_i = [1/kappa_u,i ; kappa_v,i/kappa_u,i], i en {x,y}
tau_vel = 0.4;                          % [s] constante de tiempo supuesta del lazo de velocidad
cfg.ad.theta0     = [tau_vel; 1.0];     % nominal (ganancia estatica unitaria): kappa_u = kappa_v = 1/tau
cfg.ad.theta_ini  = cfg.ad.theta0;      % valor inicial de theta_hat (igual para x e y)
cfg.ad.theta_prior= cfg.ad.theta0;      % prior de la sigma-modification

%% Ley de adaptacion (Ec. ajuste_theta)
% theta_hat_dot = Proj_Theta( -Gamma*phi'*sat(nu) - sigma*(theta_hat - theta_prior) ),
% activa solo si t > t_on y |s_i| <= s_on
cfg.ad.Gamma  = diag([0.1 0.1]);
cfg.ad.sigma  = 0.005;
cfg.ad.nu_bar = 5.0;
cfg.ad.s_on   = 0.05;
cfg.ad.t_on   = 5.0;
cfg.ad.Theta_min = [0.05; 0.2];         % proyeccion: theta1 > 0 garantiza kappa_u_hat > 0
cfg.ad.Theta_max = [3.0;  3.0];

%% Planta sustituta (solo backend 'sustituta'; no interviene en el controlador)
cfg.sust.tau_xy   = [0.30; 0.30];   % [s] lazo de velocidad "verdadero" (theta1 verdadero = 0.3)
cfg.sust.kv_ku    = [1.0; 1.0];     % kappa_v/kappa_u verdadero (theta2)
cfg.sust.tau_z    = 0.30;
cfg.sust.tau_psi  = 0.15;
cfg.sust.tau_att  = 0.05;           % [s] retardo de actitud (dinamica no modelada)
cfg.sust.retardo_muestras = 1;      % retardo de computo
cfg.sust.m_uav    = 0.48;
cfg.sust.k_cable  = 1000;           % [N/m]
cfg.sust.c_cable  = 2.0;            % [N s/m]
cfg.sust.cd_carga = 0.02;           % [N s/m] arrastre lineal de la carga respecto del aire
cfg.sust.cd_uav   = 0.05;
cfg.sust.dt_int   = 0.002;          % [s] paso de integracion interno
cfg.sust.ruido_h  = 0;              % [m] desvio del ruido sobre la posicion de la carga

%% Experimentos
cfg.exp.n_rep        = 5;           % repeticiones por caso (co-simulacion en tiempo real, no determinista)
cfg.exp.semilla      = 1;
cfg.exp.alpha_abort  = deg2rad(80); % corta la corrida si la inclinacion del cable supera este valor
cfg.exp.ex_abort     = 10;          % [m] o si el error de posicion de la carga supera este valor
cfg.met.t_inicio     = 0;           % [s] ventana de metricas (0 = corrida completa)
cfg.met.t_inicio_est = 1.0;         % [s] validacion del estimador (excluye el arranque)

%% Escenarios
switch escenario
    case 'nominal'
    case 'E1_circ_sin_swing'        % Fig. 6(a): ley cinematica sin compensacion de swing
        cfg.ext.ley = 'sin_swing'; cfg.int.ley = 'ninguno';
    case 'E2_circ_cinematico'       % Tabla metrics, fila "Kinematic"
        cfg.int.ley = 'ninguno';
        cfg.ext.seguimiento = 'lineal';   % E2: v_base del script 7-abr-2026, para ver el atraso de 10 s
    case 'E3_circ_sta_fijo'         % cascada con theta fija (theta0)
        cfg.int.ley = 'sta_fijo';
    case 'E4_circ_asta'             % controlador propuesto
        cfg.int.ley = 'asta';
        cfg.ext.seguimiento = 'lineal';   % E2: v_base del script 7-abr-2026, para ver el atraso de 10 s
    case 'E5_lemn_viento'           % robustez con viento
        cfg.ref.tipo = 'lemniscata'; cfg.planta.mL = 0.30; cfg.int.ley = 'asta';
        cfg.viento.bloques = [20 40 0 6 0; 60 80 3*sqrt(2) 3*sqrt(2) 0];
    case 'E6_circ_smc'              % ablacion del lazo interno (SMC con capa limite)
        cfg.int.ley = 'smc';
    otherwise
        % E7_masa_<gramos>_<fijo|asta>: barrido de masa con estimacion inicial desajustada.
        % Lemniscata sin viento: a diferencia del circulo (velocidad constante, aceleracion
        % nula en el marco de yaw) excita theta1 en el canal longitudinal.
        % theta_ini = [0.2; 0.5] subestima ambos parametros. Los valores "arbitrarios" previos
        % [2.0; 1.5] (kappa_u = 0.5, kappa_v = 0.75) no sirven con la ley de inversion de modelo:
        % theta2_hat > theta2 da realimentacion positiva de velocidad y theta1_hat >> theta1
        % amplifica el retardo; el STA entra en saturacion y la adaptacion (activa solo con
        % |s| <= s_on) nunca arranca. Ver ANALISIS_C1.md.
        tok = regexp(escenario, '^E7_masa_(\d+)_(fijo|asta)$', 'tokens');
        if isempty(tok)
            error('cfg_paper:escenario', 'Escenario desconocido: %s', escenario);
        end
        cfg.ref.tipo  = 'lemniscata';
        cfg.planta.T  = 160;
        cfg.planta.mL = str2double(tok{1}{1})/1000;
        cfg.ad.theta_ini = [0.2; 0.5];      % prior de la sigma-modification: theta0 nominal
        if strcmp(tok{1}{2}, 'fijo'), cfg.int.ley = 'sta_fijo'; else, cfg.int.ley = 'asta'; end
end
end
