function cfg = cfg_paper(escenario)
% CFG_PAPER  Every controller and experiment parameter.
%
%   cfg = cfg_paper()            -> nominal configuration (circle, full cascade, A-STA)
%   cfg = cfg_paper('E4_circ_asta') -> configuration of one run_paper_experiments scenario
%
% Each field carries the label of the revised-manuscript equation it enters.
% This structure is saved with every result (.mat), so any figure or table can
% be traced back to the exact values that were used.
%
% Conventions:
%   h = x - x_l (Eq. cable_vector). Inertial frame, z upward.
%   The inner loop works in the UAV yaw frame (the simulator receives
%   velocities [v_xh v_yh v_zh w_zh] in that frame; see Comandos in SimQuad.h).

if nargin < 1 || isempty(escenario), escenario = 'nominal'; end

cfg.version   = 'paper_revision 1.0';
cfg.escenario = escenario;

%% Plant / simulator
cfg.planta.backend      = 'simquad';   % 'simquad' (SimQuad1.exe + mex, Windows) | 'sustituta' (MATLAB model, for tests)
cfg.planta.T            = 100;         % [s] duration
cfg.planta.dt           = 0.02;        % [s] nominal control period (50 Hz), Table sim_params
cfg.planta.usar_dt_real = true;        % true: dt = difference of the simulator time (yQuad(24))
cfg.planta.l            = 2.0;         % [m] rope length
cfg.planta.mL           = 0.10;        % [kg] payload mass
cfg.planta.tam_carga    = 0.10;        % [m] cube side
cfg.planta.spring_friction   = 0.6;
cfg.planta.spring_constant   = 5000;
cfg.planta.air_friction      = 0.01/20;
cfg.planta.ground_repel      = 50/20;
cfg.planta.ground_slide      = 0.1/20;
cfg.planta.ground_absorption = 1/20;
cfg.planta.grafico_en_linea  = false;  % paper runs are done without online plotting
cfg.planta.g            = 9.81;

%% Reference (Eq. ref_circ / ref_lemn)
cfg.ref.tipo  = 'circulo';   % 'circulo' | 'lemniscata'
cfg.ref.circ  = struct('R', 3, 'Tp', 10, 'z0', 6, 'vz', 0.1);        % x=R sin(2pi t/Tp), y=R cos(.), z=z0+vz t
cfg.ref.lemn  = struct('a', 10, 'b', 7.5, 'c', 0.025*2*pi, 'd', 1, 'z0', 6);

%% Wind (robustness scenario). Rows: [t_ini t_fin wx wy wz] in m/s
cfg.viento.bloques = zeros(0,5);

%% Outer loop: kinematic controller (Sec. kinematic)
cfg.ext.ley    = 'completo';            % 'completo' (Eqs. base_reference, swing_compensation) | 'sin_swing' (xd_dot + Ktraj*e only)
cfg.ext.Ktraj  = diag([1.8 1.8 0.7]);   % Eq. base_reference
% Kh = 4.0 together with Kv = 0.4: stability condition Kh*(1+Kv) > (1+Kdh)*Ktraj with margin.
% (Previously 3.0 without Kv: zeta ~ 0.07 of the pendulum mode with ideal velocity tracking.)
cfg.ext.Kh     = diag([4.0 4.0]);       % Eq. swing_compensation
cfg.ext.Kdh    = diag([0.3 0.3]);       % Eq. swing_compensation
cfg.ext.Kv     = diag([0.4 0.4 0.0]);   % Eq. base_reference, term Kv*xdot_tilde (payload velocity, xy only)
cfg.ext.tau_w  = 4.0;                   % [s] washout of the Kh term in the yaw frame (0 = disabled)
cfg.ext.t_w_on = 2.0;                   % [s] the washout starts integrating after startup
cfg.ext.Ts     = 5.0;                   % [s] Eq. final_reference, sigma(t) = 1 - exp(-t/Ts)
cfg.ext.kpsi   = 1.0;                   % Eq. yaw_reference
cfg.ext.kpsit  = 0.5;                   % Eq. yaw_reference
cfg.ext.usar_ff_hdot = true;            % term [hdot*;0] in v_base (Eq. base_reference)
cfg.ext.seguimiento = 'lineal';          % 'lineal' (Eq. base_reference) | 'abril' (7 Apr 2026 script, v_base only)
cfg.ext.Ktraj_abril = diag([4.0 4.0 2.0]);    % gain outside the tanh of the April script
cfg.ext.Ktrajt      = diag([0.35 0.35 0.35]); % gain inside the tanh, position error
cfg.ext.Kvload      = diag([0.45 0.45 0.18]); % gain outside the tanh, payload velocity
cfg.ext.Kvloadt     = diag([0.90 0.90 0.60]); % gain inside the tanh, payload velocity

%% Cable-velocity estimator (Eqs. h_raw, h_hat_dyn, h_p_filter)
cfg.est.eps1  = 0.14;
cfg.est.K1    = 3.0 * eye(3);
cfg.est.tau_h = 0.06;                   % [s]

%% Discrete-implementation filters (Sec. discrete-time implementation)
% Time constants in seconds. 0 = filter disabled.
% Each active filter is discretized as y(k) = y(k-1) + a*(x(k) - y(k-1)), a = 1 - exp(-dt/tau).
cfg.filt.tau_ex   = 0;      % payload position error (formerly filtro_exil)
cfg.filt.tau_epsi = 0;      % yaw error (formerly filtro_epsi)
cfg.filt.tau_vc   = 0.03;   % kinematic command in the local frame (formerly alpha_uc ~ 46 ms). Attenuates
                            % the h noise (rope modes) that enters through Kh*h and is then differentiated in vd_dot
cfg.filt.tau_vdot = 0.05;   % numerical derivative of v_d used in a = vd_dot - beta*v_tilde

%% Inner loop: dynamic compensator (Sec. dynamic), x,y channels in the yaw frame (n = 2)
cfg.int.ley   = 'asta';                 % 'asta' | 'sta_fijo' | 'smc' (ablation) | 'ninguno' (kinematic only)
cfg.int.beta  = 0.08;                   % Eq. surface s = v_tilde + beta*int(v_tilde)
% K1/K2 = 1.0/1.5 (controller.m) compensated the outer-loop instability with
% Kh/Kdh = 1.8/0.6. With a stable outer loop they are not needed: 0.35/0.525 (0.7x of 0.5/0.75).
cfg.int.K1    = [0.3; 0.3];             % Eq. AccionControl, term K1*sqrt|s|*sign(s)
cfg.int.K2    = [0.45; 0.45];           % Eq. AccionControl, term int(K2*sign(s))
cfg.int.phi_bl = 0.05;                  % boundary layer: sign(s) ~ s/(|s|+phi_bl). With 0 the STA enters a
                                        % limit cycle from the delay and the attitude dynamics (chattering in u). State it.
cfg.int.u_max = 5.0;                    % [m/s] command saturation; conditional integration of nu
cfg.int.K_smc  = [1.2; 1.2];            % SMC ablation only: u = phi*theta0 - K_smc .* s./(|s|+phi_smc)
cfg.int.phi_smc = 0.05;

%% Model parameters (reparameterized Eq. ModeloLucas)
% theta_i = [1/kappa_u,i ; kappa_v,i/kappa_u,i], i in {x,y}
tau_vel = 0.4;                          % [s] assumed time constant of the velocity loop
cfg.ad.theta0     = [tau_vel; 1.0];     % nominal (unit static gain): kappa_u = kappa_v = 1/tau
cfg.ad.theta_ini  = cfg.ad.theta0;      % initial value of theta_hat (the same for x and y)
cfg.ad.theta_prior= cfg.ad.theta0;      % prior of the sigma-modification

%% Adaptation law (Eq. ajuste_theta)
% theta_hat_dot = Proj_Theta( -Gamma*phi'*sat(nu) - sigma*(theta_hat - theta_prior) ),
% active only if t > t_on and |s_i| <= s_on
cfg.ad.Gamma  = diag([0.1 0.1]);
cfg.ad.sigma  = 0.005;
cfg.ad.nu_bar = 5.0;
cfg.ad.s_on   = 0.05;
cfg.ad.t_on   = 5.0;
cfg.ad.Theta_min = [0.05; 0.2];         % projection: theta1 > 0 guarantees kappa_u_hat > 0
cfg.ad.Theta_max = [3.0;  3.0];

%% Surrogate plant (backend 'sustituta' only; it does not enter the controller)
cfg.sust.tau_xy   = [0.30; 0.30];   % [s] "true" velocity loop (true theta1 = 0.3)
cfg.sust.kv_ku    = [1.0; 1.0];     % true kappa_v/kappa_u (theta2)
cfg.sust.tau_z    = 0.30;
cfg.sust.tau_psi  = 0.15;
cfg.sust.tau_att  = 0.05;           % [s] attitude lag (unmodeled dynamics)
cfg.sust.retardo_muestras = 1;      % computation delay
cfg.sust.m_uav    = 0.48;
cfg.sust.k_cable  = 1000;           % [N/m]
cfg.sust.c_cable  = 2.0;            % [N s/m]
cfg.sust.cd_carga = 0.02;           % [N s/m] linear drag of the payload relative to the air
cfg.sust.cd_uav   = 0.05;
cfg.sust.dt_int   = 0.002;          % [s] internal integration step
cfg.sust.ruido_h  = 0;              % [m] std. dev. of the noise on the payload position

%% Experiments
cfg.exp.n_rep        = 5;           % repetitions per case (real-time co-simulation, not deterministic)
cfg.exp.semilla      = 1;
cfg.exp.alpha_abort  = deg2rad(80); % aborts the run if the cable tilt exceeds this value
cfg.exp.ex_abort     = 10;          % [m] or if the payload position error exceeds this value
cfg.met.t_inicio     = 1.5;           % [s] metrics window (0 = full run)
cfg.met.t_inicio_est = 1.0;         % [s] estimator validation (excludes startup)

%% Scenarios
switch escenario
    case 'nominal'
    case 'E1_circ_sin_swing'        % Fig. 6(a): kinematic law without swing compensation
        cfg.ext.ley = 'sin_swing'; cfg.int.ley = 'ninguno';
    case 'E2_circ_cinematico'       % Table metrics, row "Kinematic"
        cfg.int.ley = 'ninguno';
    case 'E3_circ_sta_fijo'         % cascade with fixed theta (theta0)
        cfg.int.ley = 'sta_fijo';
    case 'E4_circ_asta'             % proposed controller
        cfg.int.ley = 'asta';
    case 'E5_lemn_viento'           % robustness with wind
        cfg.ref.tipo = 'lemniscata'; cfg.planta.mL = 0.30; cfg.int.ley = 'asta';
        cfg.viento.bloques = [20 40 0 6 0; 60 80 3*sqrt(2) 3*sqrt(2) 0];
    case 'E6_circ_smc'              % inner-loop ablation (boundary-layer SMC)
        cfg.int.ley = 'smc';
    otherwise
        % E7_masa_<grams>_<fijo|asta>: mass sweep with a mismatched initial estimate.
        % Lemniscate without wind: unlike the circle (constant speed, zero acceleration
        % in the yaw frame) it excites theta1 on the longitudinal channel.
        % theta_ini = [0.2; 0.5] underestimates both parameters. The previous "arbitrary"
        % values [2.0; 1.5] (kappa_u = 0.5, kappa_v = 0.75) do not work with the model-inversion law:
        % theta2_hat > theta2 gives positive velocity feedback and theta1_hat >> theta1
        % amplifies the delay; the STA saturates and the adaptation (active only when
        % |s| <= s_on) never starts. See ANALISIS_C1.md.
        tok = regexp(escenario, '^E7_masa_(\d+)_(fijo|asta)$', 'tokens');
        if isempty(tok)
            error('cfg_paper:escenario', 'Escenario desconocido: %s', escenario);
        end
        cfg.ref.tipo  = 'lemniscata';
        cfg.planta.T  = 160;
        cfg.planta.mL = str2double(tok{1}{1})/1000;
        cfg.ad.theta_ini = [0.2; 0.5];      % sigma-modification prior: nominal theta0
        if strcmp(tok{1}{2}, 'fijo'), cfg.int.ley = 'sta_fijo'; else, cfg.int.ley = 'asta'; end
end
end