function T = barrido_ganancias(varargin)
% BARRIDO_GANANCIAS  Sensitivity to the controller gains (Reviewer 3, comment 4).
% Version 2026-10-16 (K1 and K2 groups with a gust).
%
%   barrido_ganancias()                                    % SimQuad1.exe, every group
%   barrido_ganancias('grupos', {'Kh'}, 'n_rep', 1)        % short test
%   barrido_ganancias('backend', 'sustituta', 'n_rep', 1)  % code test (not for the paper)
%   barrido_ganancias('salida', 'results/barrido_<fecha>', 'solo_figuras', true)
%
% ONE gain is varied at a time around the nominal configuration of cfg_paper
% (controlador_paso with washout, Kv = 0.4, phi_bl = 0.05, payload corrector v2);
% the rest stays fixed. Groups (values in grupos_def, at the end of the file):
%
%   mapa    Kh x Kv grid (1 repetition per point): stability-plane figure
%   Kh, Kv, Kdh, Ktraj    outer loop (E4, circle)
%   beta, Ksta, phi_bl    inner loop (E4). Ksta scales K1 and K2 together
%   K1, K2                dynamic compensator, each gain separately, under a wind
%                         gust (scenario 'raf': E4 circle with no initial error)
%   Gamma                 adaptation (E7 lemniscate, mismatched theta_ini)
%
% Why K1 and K2 are measured with a gust. K1 and K2 act on the inner-loop velocity
% error (v_tilde, s), not directly on the payload error, which the outer loop sets.
% Their effect shows up when there is a disturbance the super-twisting law has to
% reject: the 'raf' scenario therefore adds a gust 'rafaga' (by default 4 m/s in y
% between 20 and 32 s) and the metrics measure the peak of ||v_tilde||, the recovery
% time, and the payload error during the gust.
%
% Initial error. In E4 the UAV starts offset by cfg.exp.x0_offset = 'offset'
% ([1.5; -1.5; 0] m by default) from the nominal position, with a vertical cable.
% The error convergence is then seen from a known initial error, the same in every
% run. sim_controlador applies the offset only if the field exists.
%
% Repeated runs. Each distinct configuration is run once per repetition:
% the nominal point (Kh = 4, Kv = 0.4, ...) appears in several groups and is reused.
%
% Output in results/barrido_<fecha>/:
%   raw/<id>_rep<k>.mat     R (full series, R.cfg), M (metricas_payload), X (sweep metrics)
%   resumen_barrido.csv     one row per run
%   tabla_sensibilidad.tex  mean +- std per group and value (complete runs)
%   tabla_sensibilidad.txt  the same in text
%   figuras/conv_Kh_Kv      ||x_tilde||(t) for Kh and Kv, mean +- std, with the linear-model rate
%   figuras/mapa_Kh_Kv      (Kv, Kh) plane: Routh boundary, washout boundary, and runs
%   figuras/sensibilidad    RMSE, convergence time, and RMS du/dt versus each gain
%   figuras/sta_s           s(t) and u(t) for K1/K2 and phi_bl (limit cycle)
%   figuras/adaptacion      theta_hat(t) and |nu|(t) for Gamma
%   figuras/conv_K1_K2      ||v_tilde||(t), ||s||(t), and ||x_tilde||(t) around the gust, K1 and K2
%   figuras/sensibilidad_K1_K2   peak of ||v_tilde||, recovery, payload error, and RMS du/dt
%   tabla_compensador.tex   inner-loop metrics for K1 and K2
%
% Metrics (all from the simulator ground truth):
%   rmse_ss    RMSE of ||x_tilde|| for t >= t_ss (steady state)
%   t_conv     last instant at which ||x_tilde|| leaves the band max(1.2 max_ss, 5 cm)
%   sigma_med  slope of log(envelope - floor), fitted from t = 2 Ts; < 0 converges
%   sigma_lin  real part of the dominant pole of the linear model (pendulum + outer loop,
%              ideal velocity tracking, with washout); see modelo_lineal
%   clase      divergencia (run cut) | creciente (RMS of the last quarter > 1.3 times that
%              of the third) | oscilante (RMSE ss > 'e_osc', 0.15 m: sustained oscillation,
%              bounded by the saturation u_max) | estable
%   p99_vperp, rms_swing, rms_du, rms_nu, p99_s
%   Inner loop (every run): rms_vt = RMS of ||v_tilde|| for t >= t_ss.
%   Gust (scenario 'raf'), window [t_on, t_off + 8 s]:
%   pico_vt, pico_s       peaks of ||v_tilde|| and of max|s_i|
%   t_rec_on, t_rec_off   time from the start (end) of the gust until ||v_tilde||
%                         stays for 1 s inside max(1.5 RMS before the gust, 2 cm/s)
%   rms_vt_raf, rmse_ex_raf, pico_ex_raf
%
% Resume: as in comparar_omar_vs_asta. Each run is saved as soon as it finishes; calling
% again with the same arguments resumes the pending folder.
%
% Estimated time on SimQuad (default values): about 75 runs of 45 s, 27 of 50 s,
% and 12 of 120 s, around 2 h.

op = struct('backend', 'simquad', 'n_rep', 3, 'n_rep_mapa', 1, ...
    'grupos', {{'mapa', 'Kh', 'Kv', 'Kdh', 'Ktraj', 'beta', 'Ksta', 'phi_bl', 'K1', 'K2', 'Gamma'}}, ...
    'T', 45, 'T_adapt', 120, 'T_raf', 50, 't_ss', 25, 'offset', [1.5; -1.5; 0], 'e_osc', 0.15, ...
    'rafaga', [20 32 0 4 0], ...
    'salida', '', 'reanudar', true, 'reintentos', 2, 'solo_figuras', false, 'figuras', true);
for i = 1:2:numel(varargin)
    if ~isfield(op, varargin{i}), error('barrido_ganancias:op', 'Opcion desconocida: %s', varargin{i}); end
    op.(varargin{i}) = varargin{i+1};
end
op.offset = op.offset(:);
if ischar(op.grupos), op.grupos = {op.grupos}; end
validos = {'mapa', 'Kh', 'Kv', 'Kdh', 'Ktraj', 'beta', 'Ksta', 'phi_bl', 'K1', 'K2', 'Gamma'};
malos = op.grupos(~ismember(op.grupos, validos));
if ~isempty(malos)
    error('barrido_ganancias:grupos', 'Grupo desconocido: %s. Validos: %s', ...
        strjoin(malos, ', '), strjoin(validos, ', '));
end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
sim_dir = fullfile(aqui, '..', 'simulations');
if exist(sim_dir, 'dir'), addpath(sim_dir); end

%% Cases ----------------------------------------------------------------------
G = grupos_def(op);
[C, G] = armar_casos(G, op);
fprintf('%d grupos, %d configuraciones distintas, %d corridas.\n', numel(G), numel(C), sum([C.nrep]));

%% Folder ---------------------------------------------------------------------
fm = firma(op);
if isempty(op.salida) && op.reanudar && ~op.solo_figuras
    op.salida = buscar_pendiente(fullfile(aqui, 'results'), fm);
end
if isempty(op.salida)
    if op.solo_figuras, error('barrido_ganancias:salida', 'solo_figuras requiere ''salida''.'); end
    op.salida = fullfile(aqui, 'results', ['barrido_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
mkdir_p(fullfile(op.salida, 'raw'));
arch_prog = fullfile(op.salida, 'en_curso.mat');
if ~op.solo_figuras
    if exist(arch_prog, 'file')
        P0 = load(arch_prog);
        if ~strcmp(P0.firma, fm)
            error('barrido_ganancias:firma', '%s es de otra configuracion del barrido.', op.salida);
        end
        fprintf('Reanudando en %s\n', op.salida);
    end
    Pg.firma = fm; Pg.op = op;
    save(arch_prog, '-struct', 'Pg');
end

%% Runs -----------------------------------------------------------------------
pend = 0; tpend = 0;
for c = 1:numel(C)
    for k = 1:C(c).nrep
        if ~exist(arch_raw(op, C(c), k), 'file')
            pend = pend + 1; tpend = tpend + C(c).cfg.planta.T + 4;
        end
    end
end
if ~op.solo_figuras && pend > 0
    fprintf('Corridas pendientes: %d (unos %.0f min en tiempo real).\n', pend, tpend/60);
end

filas = {};
for c = 1:numel(C)
    C(c).X = {}; C(c).S = {};
    for k = 1:C(c).nrep
        a = arch_raw(op, C(c), k);
        [R, M, X, ya] = leer_raw(a);
        if ya                                          % recomputed with the current options
            nf = campo_o(X, 'reintentos', 0);           % (class thresholds, t_ss)
            if R.cfg.met.t_inicio ~= op.t_ss, R.cfg.met.t_inicio = op.t_ss; M = metricas_payload(R); end
            X = metricas_barrido(R, M, op); X.reintentos = nf;
        else
            if op.solo_figuras, continue; end
            cfg = C(c).cfg;
            semilla(cfg.exp.semilla + k);
            fprintf('[%s] %d/%d %s rep %d ... ', datestr(now, 'HH:MM:SS'), c, numel(C), C(c).id, k);
            [R, nfall] = sim_con_reintento(@controlador_paso, cfg, op.reintentos, a(1:end-4));
            R.rep = k; R.reintentos = nfall; R.barrido_id = C(c).id;
            M = metricas_payload(R);
            X = metricas_barrido(R, M, op);
            X.reintentos = nfall;
            fprintf('%s (%s), t = %.1f s, RMSE ss = %.3f m, sigma = %+.2f (lin %+.2f) 1/s\n', ...
                R.motivo, X.clase, R.t(end), X.rmse_ss, X.sigma_med, X.sigma_lin);
            guardar_raw(a, R, M, X);
        end
        C(c).X{end+1} = X;
        C(c).S{end+1} = serie_fig(R, M);
        filas{end+1} = fila(C(c), k, X); %#ok<AGROW>
    end
end
if isempty(filas), error('barrido_ganancias:vacio', 'No hay corridas en %s.', op.salida); end

%% Tables and figures ---------------------------------------------------------
escribir_csv(fullfile(op.salida, 'resumen_barrido.csv'), filas);
T = tabla(G, C);
escribir_tex(T, fullfile(op.salida, 'tabla_sensibilidad.tex'), op, claves_tabla('sensibilidad'), {'E4', 'lemn'});
escribir_tex(T, fullfile(op.salida, 'tabla_compensador.tex'), op, claves_tabla('compensador'), {'raf'});
escribir_txt(T, fullfile(op.salida, 'tabla_sensibilidad.txt'), op);
if op.figuras
    dfig = fullfile(op.salida, 'figuras');
    mkdir_p(dfig);
    intentar(@() fig_convergencia(G, C, op, dfig), 'conv_Kh_Kv');
    intentar(@() fig_mapa(G, C, op, dfig), 'mapa_Kh_Kv');
    intentar(@() fig_sensibilidad(G, T, find(~[G.mapa] & ~strcmp({G.esc}, 'raf')), ...
        {'rmse_ss', 'RMSE ||x_{tilde}|| [m]'; 't_conv', 't_{conv} [s]'; 'rms_du', 'RMS du_h/dt [m/s^2]'}, ...
        fullfile(dfig, 'sensibilidad')), 'sensibilidad');
    intentar(@() fig_sensibilidad(G, T, find(strcmp({G.esc}, 'raf')), ...
        {'pico_vt', 'peak ||v_{tilde}|| [m/s]'; 't_rec_on', 't_{rec}, gust onset [s]'; ...
         'rmse_ex_raf', 'RMSE ||x_{tilde}||, gust [m]'; 'rms_du', 'RMS du_h/dt [m/s^2]'}, ...
        fullfile(dfig, 'sensibilidad_K1_K2')), 'sensibilidad_K1_K2');
    intentar(@() fig_rafaga(G, C, op, dfig), 'conv_K1_K2');
    intentar(@() fig_sta(G, C, op, dfig), 'sta_s');
    intentar(@() fig_adaptacion(G, C, op, dfig), 'adaptacion');
end
if ~op.solo_figuras && exist(arch_prog, 'file'), delete(arch_prog); end
fprintf('\nResultados en %s\n', op.salida);
end

%% ===================================================================================
%% Sweep definition
%% ===================================================================================
function G = grupos_def(op)
% param: field that is modified (see aplicar). esc: 'E4' (circle with initial error),
% 'raf' (circle with a gust, no initial error), or 'lemn' (adaptation).
% K1 and K2 are scaled from the cfg_paper nominal (0.3 and 0.45); the rest stays fixed.
% The Kh values include points below the boundary Kh(1+Kv) = (1+Kdh)Ktraj
% (Kh = 1.67 with Kv = 0.4; 2.34 with Kv = 0), so the figure shows both sides.
D = {
 % name      param      values                 nominal  esc     label (tex)             LaTeX label
  'Kh',      'Kh',      [1 1.5 2 3 4 6],       4,       'E4',   'K_h',                  '$K_h$'
  'Kv',      'Kv',      [0 0.2 0.4 0.8],       0.4,     'E4',   'K_v',                  '$K_v$'
  'Kdh',     'Kdh',     [0 0.3 0.6],           0.3,     'E4',   'K_{dh}',               '$K_{dh}$'
  'Ktraj',   'Ktraj_s', [0.5 1 1.5],           1,       'E4',   'K_{traj} / K_{traj,0}', '$K_{traj}/K_{traj,0}$'
  'beta',    'beta',    [0.04 0.08 0.16 0.32], 0.08,    'E4',   '\beta',                '$\beta$'
  'Ksta',    'Ksta_s',  [0.5 1 2],             1,       'E4',   'K_1, K_2 / nominal',   '$(K_1,K_2)/(K_1,K_2)_0$'
  'phi_bl',  'phi_bl',  [0 0.05 0.1],          0.05,    'E4',   '\phi',                 '$\phi$'
  'K1',      'K1_s',    [0.25 0.5 1 2 4],      1,       'raf',  'K_1 / K_{1,0}',        '$K_1/K_{1,0}$'
  'K2',      'K2_s',    [0.25 0.5 1 2 4],      1,       'raf',  'K_2 / K_{2,0}',        '$K_2/K_{2,0}$'
  'Gamma',   'Gamma',   [0 0.03 0.1 0.3],      0.1,     'lemn', '\Gamma',               '$\Gamma$'
  };
G = struct('nombre', {}, 'param', {}, 'vals', {}, 'nom', {}, 'esc', {}, 'etq', {}, 'tex', {}, ...
    'mapa', {}, 'Kv_vals', {}, 'nrep', {});
for i = 1:size(D, 1)
    if ~any(strcmp(op.grupos, D{i,1})), continue; end
    G(end+1) = struct('nombre', D{i,1}, 'param', D{i,2}, 'vals', D{i,3}, 'nom', D{i,4}, ...
        'esc', D{i,5}, 'etq', D{i,6}, 'tex', D{i,7}, 'mapa', false, 'Kv_vals', [], 'nrep', op.n_rep); %#ok<AGROW>
end
if any(strcmp(op.grupos, 'mapa'))
    G(end+1) = struct('nombre', 'mapa', 'param', 'Kh', 'vals', [1 1.5 2 3 4 6], 'nom', 4, ...
        'esc', 'E4', 'etq', 'K_h', 'tex', '$K_h$', 'mapa', true, 'Kv_vals', [0 0.2 0.4 0.8], ...
        'nrep', op.n_rep_mapa);
end
end

function cfg = cfg_base(esc, op)
switch esc
    case 'E4'
        cfg = cfg_paper('E4_circ_asta');
        cfg.planta.T = op.T;
        cfg.exp.x0_offset = op.offset;
    case 'raf'
        cfg = cfg_paper('E4_circ_asta');
        cfg.planta.T = op.T_raf;
        cfg.viento.bloques = op.rafaga;            % [t_on t_off wx wy wz]
        cfg.exp.t_clase = op.rafaga(1, 2) + 8;     % the class is decided after the gust
    case 'lemn'
        cfg = cfg_paper('E7_masa_100_asta');      % lemniscate, theta_ini = [0.2; 0.5]
        cfg.planta.T = op.T_adapt;
    otherwise
        error('barrido_ganancias:esc', 'Escenario desconocido: %s', esc);
end
cfg.planta.backend = op.backend;
if strcmp(op.backend, 'sustituta'), cfg.planta.usar_dt_real = false; end
cfg.met.t_inicio = op.t_ss;                   % metricas_payload = steady state
end

function cfg = aplicar(cfg, param, v)
switch param
    case 'Kh',      cfg.ext.Kh = v*eye(2);
    case 'Kv',      cfg.ext.Kv = diag([v v cfg.ext.Kv(3,3)]);
    case 'Kdh',     cfg.ext.Kdh = v*eye(2);
    case 'Ktraj_s', cfg.ext.Ktraj(1:2,1:2) = v*cfg.ext.Ktraj(1:2,1:2);   % x,y only
    case 'beta',    cfg.int.beta = v;
    case 'Ksta_s',  cfg.int.K1 = v*cfg.int.K1; cfg.int.K2 = v*cfg.int.K2;
    case 'K1_s',    cfg.int.K1 = v*cfg.int.K1;
    case 'K2_s',    cfg.int.K2 = v*cfg.int.K2;
    case 'phi_bl',  cfg.int.phi_bl = v;
    case 'Gamma'
        cfg.ad.Gamma = v*eye(2);
        if v == 0, cfg.int.ley = 'sta_fijo'; end   % no adaptation: without the sigma-modification
                                                  % theta_hat would keep drifting toward the prior
    otherwise, error('barrido_ganancias:param', 'Parametro desconocido: %s', param);
end
end

function id = id_cfg(cfg, esc)
% Identifier with every gain that is swept: two groups that reach the
% same configuration share the runs.
id = sprintf('%s_Kh%.2f_Kv%.2f_Kdh%.2f_Kt%.2f_b%.3f_K1%.3f_K2%.3f_phi%.3f_G%.3f', esc, ...
    cfg.ext.Kh(1,1), cfg.ext.Kv(1,1), cfg.ext.Kdh(1,1), cfg.ext.Ktraj(1,1), cfg.int.beta, ...
    cfg.int.K1(1), cfg.int.K2(1), cfg.int.phi_bl, cfg.ad.Gamma(1,1));
id = strrep(id, '.', 'p');
end

function [C, G] = armar_casos(G, op)
C = struct('id', {}, 'cfg', {}, 'nrep', {}, 'esc', {});
for g = 1:numel(G)
    if G(g).mapa, kv = G(g).Kv_vals; else, kv = nan; end
    G(g).caso = zeros(numel(kv), numel(G(g).vals));   % row: Kv (map only); column: value
    for a = 1:numel(kv)
        for j = 1:numel(G(g).vals)
            cfg = cfg_base(G(g).esc, op);
            cfg = aplicar(cfg, G(g).param, G(g).vals(j));
            if G(g).mapa, cfg = aplicar(cfg, 'Kv', kv(a)); end
            id = id_cfg(cfg, G(g).esc);
            c = find(strcmp({C.id}, id));
            if isempty(c)
                cfg.barrido = struct('grupo', G(g).nombre, 'id', id);
                C(end+1) = struct('id', id, 'cfg', cfg, 'nrep', G(g).nrep, 'esc', G(g).esc); %#ok<AGROW>
                c = numel(C);
            else
                C(c).nrep = max(C(c).nrep, G(g).nrep);
            end
            G(g).caso(a, j) = c;
        end
    end
end
end

%% ===================================================================================
%% Sweep metrics
%% ===================================================================================
function X = metricas_barrido(R, M, op)
cfg = R.cfg;
t = R.t; T = t(end);
e = sqrt(sum((R.xd - R.xl).^2, 1));
ss = t >= op.t_ss;
X.completo = R.completo;
X.t_final = T;
X.e0 = e(1);
X.rmse_ss = M.rmse_ex;
X.rmse_total = sqrt(mean(e.^2));
X.max_ss = maxv(e(ss));
X.rms_swing = M.rms_swing;
X.max_alpha = M.max_alpha;
X.p99_vperp = p99(M.serie.vperp(ss));
X.rms_du = norm(M.rms_du);
X.rms_nu = norm(M.rms_nu);
X.rms_s = norm(M.rms_s);
X.p99_s = p99(max(abs(R.s(:, ss)), [], 1));
X.frac_sat = M.frac_sat;
X.theta_fin = M.theta_fin;
X.n_corr_xl = R.n_corr_xl;
[X.sigma_lin, X.w_lin, X.routh] = modelo_lineal(cfg);

% inner loop
vt = sqrt(sum(R.vt.^2, 1));
X.rms_vt = rms1(vt(ss));
X = metricas_rafaga(X, t, vt, e, R, cfg);

% run class (in 'raf', after the gust)
tc = op.t_ss; if isfield(cfg.exp, 't_clase'), tc = cfg.exp.t_clase; end
X.clase = 'estable';
if ~R.completo
    X.clase = 'divergencia';
elseif any(t >= tc)
    tm = (tc + T)/2;
    q3 = rms1(e(t >= tc & t < tm)); q4 = rms1(e(t >= tm));
    if q4 > 1.3*q3 && q4 > 0.10
        X.clase = 'creciente';
    elseif rms1(e(t >= tc)) > op.e_osc
        X.clase = 'oscilante';
    end
end

% convergence time: last instant outside the band
X.banda = max(1.2*X.max_ss, 0.05);
X.t_conv = nan;
if strcmp(X.clase, 'estable')
    i = find(e > X.banda, 1, 'last');
    if isempty(i), X.t_conv = 0; elseif i < numel(t), X.t_conv = t(i+1); end
end

% measured convergence rate, on the envelope of ||x_tilde||
dtg = 0.02; tg = 0:dtg:T;
[tu, iu] = unique(t);
eg = interp1(tu, e(iu), tg, 'linear', 'extrap');
W = max(1, round(2*pi/sqrt(cfg.planta.g/cfg.planta.l)/dtg));   % one pendulum period
env = eg;
for i = 1:numel(tg), env(i) = max(eg(i:min(i+W, end))); end
t0 = 2*cfg.ext.Ts;
X.sigma_med = nan;
nmin = 1.0/dtg;
switch X.clase
    case 'estable'
        piso = rms1(e(ss));
        sel = tg >= t0 & tg <= op.t_ss & env > 2*piso;
        if sum(sel) > nmin
            p = polyfit(tg(sel), log(env(sel) - piso), 1); X.sigma_med = p(1);
        end
    case {'creciente', 'divergencia'}
        sel = tg >= t0 & tg <= T - W*dtg & env > 0;
        if sum(sel) > nmin
            p = polyfit(tg(sel), log(env(sel)), 1); X.sigma_med = p(1);
        end
end

% dominant frequency of the cable oscillation about h* (x channel, inertial)
X.w_osc = nan;
sel = tg >= t0;
if sum(sel) > 4/dtg
    dh = interp1(tu, R.h(1, iu) - R.hs(1, iu), tg(sel), 'linear', 'extrap');
    dh = dh - mean(dh);
    n = numel(dh); F = abs(fft(dh .* hann_w(n)));
    w = 2*pi*(0:n-1)/(n*dtg);
    ok = w > 0.3 & w < 2*pi/(2*dtg);
    if any(ok), wf = w(ok); [~, j] = max(F(ok)); X.w_osc = wf(j); end
end
end

function X = metricas_rafaga(X, t, vt, e, R, cfg)
% Inner-loop response to the first gust in cfg.viento.bloques.
c = {'pico_vt', 'pico_s', 't_rec_on', 't_rec_off', 'rms_vt_raf', 'rmse_ex_raf', 'pico_ex_raf', 'banda_vt'};
for i = 1:numel(c), X.(c{i}) = nan; end
B = cfg.viento.bloques;
if isempty(B) || ~any(any(B(:, 3:5) ~= 0)), return; end
ton = B(1, 1); toff = B(1, 2);
if t(end) < toff, return; end                       % run cut before the end of the gust
pre = t >= ton - 8 & t < ton;
win = t >= ton & t < min(toff + 8, t(end));
X.banda_vt = max(1.5*rms1(vt(pre)), 0.02);
X.pico_vt = maxv(vt(win));
X.pico_s = maxv(max(abs(R.s(:, win)), [], 1));
X.rms_vt_raf = rms1(vt(win));
X.rmse_ex_raf = rms1(e(win));
X.pico_ex_raf = maxv(e(win));
X.t_rec_on = t_recuperacion(t, vt, ton, toff, X.banda_vt, 1.0);
X.t_rec_off = t_recuperacion(t, vt, toff, min(toff + 15, t(end)), X.banda_vt, 1.0);
end

function tr = t_recuperacion(t, y, ta, tb, banda, dur)
% Time from ta until the first instant at which y stays inside the band for
% 'dur' seconds in a row (before tb). NaN if it does not recover.
tr = nan;
idx = find(t >= ta & t <= tb - dur);
fuera = y > banda;
for k = idx
    if ~any(fuera(t >= t(k) & t <= t(k) + dur)), tr = t(k) - ta; return; end
end
end

function [sigma, wd, routh] = modelo_lineal(cfg)
% Characteristic polynomial per axis of the outer loop linearized about a tracking
% point: pendulum h_ddot = x_ddot - (g/l) h, ideal velocity tracking
% (x_dot = v_c, the inner loop is assumed fast) and zero reference:
%   v_c = -Ktraj x_l - Kv x_l_dot - Kh W(s) h - Kdh h_dot,  W(s) = tw s/(tw s + 1).
% Without washout it reduces to (1+Kdh) s^3 + Kh s^2 + (1+Kv) w2 s + Ktraj w2, whose
% Routh condition is Kh (1+Kv) > (1+Kdh) Ktraj. With washout:
%   s (s^2+w2)(tw s+1) + (Kv s + Ktraj) w2 (tw s+1) + Kh tw s^3 + Kdh s^3 (tw s+1).
w2 = cfg.planta.g/cfg.planta.l;
Kt = cfg.ext.Ktraj(1,1); Kh = cfg.ext.Kh(1,1); Kdh = cfg.ext.Kdh(1,1); Kv = cfg.ext.Kv(1,1);
tw = 0; if isfield(cfg.ext, 'tau_w'), tw = cfg.ext.tau_w; end
p = poly_lazo(Kt, Kh, Kdh, Kv, tw, w2);
r = roots(p);
[sigma, j] = max(real(r));
wd = abs(imag(r(j)));
routh = Kh*(1 + Kv) > (1 + Kdh)*Kt;
end

function p = poly_lazo(Kt, Kh, Kdh, Kv, tw, w2)
if tw > 0
    p = sumpoly(conv(conv([1 0], [1 0 w2]), [tw 1]), w2*conv([Kv Kt], [tw 1]), ...
        Kh*tw*[1 0 0 0], Kdh*conv([1 0 0 0], [tw 1]));
else
    p = [1 + Kdh, Kh, w2*(1 + Kv), w2*Kt];
end
end

function p = sumpoly(varargin)
n = max(cellfun(@numel, varargin));
p = zeros(1, n);
for i = 1:numel(varargin)
    q = varargin{i}; p(n-numel(q)+1:end) = p(n-numel(q)+1:end) + q;
end
end

%% ===================================================================================
%% Tables
%% ===================================================================================
function D = metricas_tabla()
% key, LaTeX, text, format
D = { 'sigma_lin', '$\sigma_{\mathrm{lin}}$ [s$^{-1}$]',            'sigma_lin [1/s]',    '%+.2f'
      'sigma_med', '$\sigma_{\mathrm{sim}}$ [s$^{-1}$]',            'sigma_sim [1/s]',    '%+.2f'
      't_conv',    '$t_{\mathrm{conv}}$ [s]',                        't_conv [s]',         '%.1f'
      'rmse_ss',   'RMSE $\|\tilde{\mathbf{x}}\|$ [m]',              'RMSE x_tilde [m]',   '%.3f'
      'rms_swing', 'RMS $\|\mathbf{h}_h-\mathbf{h}^\star_h\|$ [m]',  'RMS swing [m]',      '%.3f'
      'p99_vperp', 'P99 $\|\mathbf{v}_\perp\|$ [m/s]',               'P99 |v_perp| [m/s]', '%.3f'
      'rms_du',    'RMS $\|\dot{\mathbf{u}}_h\|$ [m/s$^2$]',         'RMS du/dt [m/s2]',   '%.2f'
      'rms_nu',    'RMS $\|\nu\|$ [m/s]',                            'RMS nu [m/s]',       '%.3f'
      'p99_s',     'P99 $|s|$ [m/s]',                                'P99 |s| [m/s]',      '%.3f'
      'rms_vt',    'RMS $\|\tilde{\mathbf{v}}\|$ [m/s]',            'RMS v_tilde [m/s]',  '%.3f'
      'pico_vt',   'Peak $\|\tilde{\mathbf{v}}\|$ [m/s]',           'pico v_tilde [m/s]', '%.3f'
      't_rec_on',  '$t_{\mathrm{rec}}$ onset [s]',                   't_rec ini [s]',      '%.2f'
      't_rec_off', '$t_{\mathrm{rec}}$ end [s]',                     't_rec fin [s]',      '%.2f'
      'pico_s',    'Peak $|s|$ [m/s]',                               'pico |s| [m/s]',     '%.3f'
      'rms_vt_raf','RMS $\|\tilde{\mathbf{v}}\|$, gust [m/s]',      'RMS v_tilde raf.',   '%.3f'
      'rmse_ex_raf','RMSE $\|\tilde{\mathbf{x}}\|$, gust [m]',      'RMSE x_tilde raf.',  '%.3f'
      'pico_ex_raf','Peak $\|\tilde{\mathbf{x}}\|$, gust [m]',      'pico x_tilde raf.',  '%.3f' };
end

function c = claves_tabla(cual)
switch cual
    case 'sensibilidad'
        c = {'sigma_lin', 'sigma_med', 't_conv', 'rmse_ss', 'rms_swing', 'p99_vperp', 'rms_du', 'rms_nu', 'p99_s'};
    case 'compensador'
        c = {'pico_vt', 't_rec_on', 't_rec_off', 'rms_vt_raf', 'pico_s', 'rmse_ex_raf', 'pico_ex_raf', 'rms_du'};
end
end

function T = tabla(G, C)
D = metricas_tabla();
T.D = D; T.filas = {};
for g = 1:numel(G)
    if G(g).mapa, nkv = numel(G(g).Kv_vals); else, nkv = 1; end
    for a = 1:nkv
        for j = 1:numel(G(g).vals)
            c = G(g).caso(a, j);
            Xs = C(c).X;
            f.grupo = G(g).nombre; f.tex = G(g).tex; f.esc = G(g).esc;
            f.valor = G(g).vals(j); f.nominal = abs(G(g).vals(j) - G(g).nom) < 1e-12;
            if G(g).mapa, f.Kv = G(g).Kv_vals(a); else, f.Kv = nan; end
            f.id = C(c).id;
            f.n = numel(Xs);
            f.nc = sum(cellfun(@(x) x.completo, Xs));
            f.clase = peor_clase(Xs);
            f.routh = nan; if ~isempty(Xs), f.routh = Xs{1}.routh; end
            for i = 1:size(D, 1)
                if any(strcmp(D{i,1}, {'sigma_med', 't_conv'}))
                    v = cellfun(@(x) x.(D{i,1}), Xs);                  % also from cut runs
                else
                    v = cellfun(@(x) x.(D{i,1}), Xs(cellfun(@(x) x.completo, Xs)));
                end
                v = v(isfinite(v));
                f.media(i) = mean_v(v); f.desv(i) = std_v(v);
            end
            T.filas{end+1} = f;
        end
    end
end
end

function k = peor_clase(Xs)
orden = {'estable', 'oscilante', 'creciente', 'divergencia'};
k = '--'; m = 0;
for i = 1:numel(Xs)
    j = find(strcmp(orden, Xs{i}.clase));
    if j > m, m = j; k = orden{j}; end
end
end

function escribir_tex(T, archivo, op, claves, escs)
% Only the columns 'claves' and the groups of the scenarios 'escs'.
F = T.filas(cellfun(@(f) any(strcmp(f.esc, escs)) && ~strcmp(f.grupo, 'mapa'), T.filas));
if isempty(F), return; end
[~, sel] = ismember(claves, T.D(:,1));
fid = fopen(archivo, 'w');
fprintf(fid, '%% Generado por barrido_ganancias.m (%s, backend %s).\n', datestr(now, 'yyyy-mm-dd HH:MM'), op.backend);
fprintf(fid, '%% Media $\\pm$ desvio sobre repeticiones completas; regimen permanente t >= %g s.\n', op.t_ss);
if any(strcmp(escs, 'raf'))
    fprintf(fid, '%% Rafaga [t_on t_off wx wy wz] = [%g %g %g %g %g]; ventana [t_on, t_off + 8 s].\n', op.rafaga(1,:));
else
    fprintf(fid, '%% Error inicial [%g %g %g] m en E4. sigma: < 0 converge.\n', op.offset);
end
D = T.D(sel, :); nd = size(D, 1);
fprintf(fid, '\\begin{tabular}{ll c%s}\n\\toprule\n', repmat('c', 1, nd));
fprintf(fid, 'Gain & Value & Runs');
for i = 1:nd, fprintf(fid, ' & %s', D{i,2}); end
fprintf(fid, ' \\\\\n\\midrule\n');
ant = '';
for r = 1:numel(F)
    f = F{r};
    if ~strcmp(f.grupo, ant)
        if ~isempty(ant), fprintf(fid, '\\midrule\n'); end
        g = f.tex; ant = f.grupo;
    else
        g = '';
    end
    v = sprintf('%g', f.valor); if f.nominal, v = [v '$^{\ast}$']; end
    fprintf(fid, '%s & %s & %d/%d', g, v, f.nc, f.n);
    for i = 1:nd, fprintf(fid, ' & %s', fmt_tex(f.media(sel(i)), f.desv(sel(i)), D{i,4})); end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fprintf(fid, '%% $^{\\ast}$ valor nominal.\n');
fclose(fid);
end

function escribir_txt(T, archivo, op)
fid = fopen(archivo, 'w');
D = T.D;
for dest = [1 fid]
    fprintf(dest, '\nBarrido de ganancias. Media +- desvio, t >= %g s. * = nominal.\n', op.t_ss);
    fprintf(dest, '%-7s %-8s %-6s %-6s %-11s', 'grupo', 'valor', 'Kv', 'corr.', 'clase');
    for i = 1:size(D, 1), fprintf(dest, ' %-18s', D{i,3}); end
    fprintf(dest, '\n');
    for r = 1:numel(T.filas)
        f = T.filas{r};
        v = sprintf('%g%s', f.valor, ifs(f.nominal, '*', ''));
        fprintf(dest, '%-7s %-8s %-6s %-6s %-11s', f.grupo, v, fmt_num(f.Kv, '%g'), ...
            sprintf('%d/%d', f.nc, f.n), f.clase);
        for i = 1:size(D, 1)
            fprintf(dest, ' %-18s', [fmt_num(f.media(i), D{i,4}) ' +- ' fmt_num(f.desv(i), D{i,4})]);
        end
        fprintf(dest, '\n');
    end
end
fclose(fid);
end

function f = fila(C, k, X)
cfg = C.cfg;
f = struct('id', C.id, 'grupo', cfg.barrido.grupo, 'rep', k, ...
    'Kh', cfg.ext.Kh(1,1), 'Kv', cfg.ext.Kv(1,1), 'Kdh', cfg.ext.Kdh(1,1), 'Ktraj', cfg.ext.Ktraj(1,1), ...
    'beta', cfg.int.beta, 'K1', cfg.int.K1(1), 'K2', cfg.int.K2(1), 'phi_bl', cfg.int.phi_bl, ...
    'Gamma', cfg.ad.Gamma(1,1), 'completo', double(X.completo), 'clase', X.clase, ...
    't_final', X.t_final, 'routh', double(X.routh), 'sigma_lin', X.sigma_lin, 'w_lin', X.w_lin, ...
    'sigma_med', X.sigma_med, 'w_osc', X.w_osc, 't_conv', X.t_conv, 'e0', X.e0, ...
    'rmse_ss', X.rmse_ss, 'max_ss', X.max_ss, 'rms_swing', X.rms_swing, 'max_alpha', X.max_alpha, ...
    'p99_vperp', X.p99_vperp, 'rms_du', X.rms_du, 'rms_nu', X.rms_nu, 'rms_s', X.rms_s, ...
    'p99_s', X.p99_s, 'frac_sat', X.frac_sat, 'th1_x', X.theta_fin(1), 'th2_x', X.theta_fin(2), ...
    'th1_y', X.theta_fin(3), 'th2_y', X.theta_fin(4), 'n_corr_xl', X.n_corr_xl, ...
    'rms_vt', X.rms_vt, 'pico_vt', X.pico_vt, 'pico_s', X.pico_s, 't_rec_on', X.t_rec_on, ...
    't_rec_off', X.t_rec_off, 'rms_vt_raf', X.rms_vt_raf, 'rmse_ex_raf', X.rmse_ex_raf, ...
    'pico_ex_raf', X.pico_ex_raf, 'reintentos', campo_o(X, 'reintentos', 0));
end

%% ===================================================================================
%% Figures
%% ===================================================================================
function s = serie_fig(R, M) %#ok<INUSD>
s.t = R.t;
s.e = sqrt(sum((R.xd - R.xl).^2, 1));
s.s = R.s;
s.vt = R.vt;
s.u = R.u(1:2, :);
s.nu = R.nu;
s.theta = R.theta;
s.completo = R.completo;
s.sust = [];
if strcmp(R.cfg.planta.backend, 'sustituta')
    s.sust = [R.cfg.sust.tau_xy(1); R.cfg.sust.kv_ku(1)];
end
end

function g = buscar_grupo(G, nombre)
g = find(strcmp({G.nombre}, nombre) & ~[G.mapa], 1);
end

function fig_convergencia(G, C, op, dfig)
grs = {'Kh', 'Kv'};
grs = grs(cellfun(@(n) ~isempty(buscar_grupo(G, n)), grs));
if isempty(grs), return; end
f = nueva_fig(18, 6.5*numel(grs));
for q = 1:numel(grs)
    g = buscar_grupo(G, grs{q});
    subplot(numel(grs), 1, q); hold on; box on; grid on;
    nv = numel(G(g).vals); col = paleta(nv);
    h = zeros(1, nv); leg = cell(1, nv); ymin = inf; ymax = 0; Tfin = 0;
    for j = 1:nv
        c = G(g).caso(1, j);
        [tg, ym, ys] = media_series(C(c).S, 'e', 0.05);
        if isempty(tg), continue; end
        Tfin = max(Tfin, tg(end));
        lo = max(ym - ys, 1e-3); hi = ym + ys;
        banda(tg, lo, hi, col(j,:));
        lw = 1.0 + 0.8*(abs(G(g).vals(j) - G(g).nom) < 1e-12);
        h(j) = plot(tg, ym, '-', 'Color', col(j,:), 'LineWidth', lw);
        % linear-model rate, anchored at t0 = 2 Ts
        X1 = C(c).X{1};
        t0 = 2*C(c).cfg.ext.Ts;
        [~, i0] = min(abs(tg - t0));
        tl = linspace(t0, op.t_ss, 50);
        plot(tl, ym(i0)*exp(X1.sigma_lin*(tl - t0)), ':', 'Color', col(j,:), 'LineWidth', 1.0);
        leg{j} = sprintf('%s = %g%s', G(g).etq, G(g).vals(j), ifs(lw > 1, ' (nom.)', ''));
        if ~X1.routh, leg{j} = [leg{j} ' (Routh fails)']; end
        ymin = min(ymin, min(ym(isfinite(ym) & ym > 0))); ymax = max(ymax, max(ym));
    end
    ok = h ~= 0;
    set(gca, 'YScale', 'log');
    if isfinite(ymin) && ymax > 0, ylim([max(ymin*0.8, 5e-3) min(ymax*1.5, 20)]); end
    if Tfin > 0, xlim([0 Tfin]); end
    ylabel('||x_{tilde}|| [m]');
    yl = ylim; plot(op.t_ss*[1 1], yl, 'k--', 'HandleVisibility', 'off');
    if q == numel(grs), xlabel('Time [s]'); end
    hd = plot(nan, nan, 'k:', 'LineWidth', 1.0);
    legend([h(ok) hd], [leg(ok) {'linear-model rate'}], 'Location', 'eastoutside', 'FontSize', 7);
    title(sprintf('||x_{tilde}||(t) for different %s, mean \\pm 1\\sigma', G(g).etq));
end
guardar_fig(f, fullfile(dfig, 'conv_Kh_Kv'));
end

function fig_mapa(G, C, op, dfig) %#ok<INUSL>
g = find([G.mapa], 1);
if isempty(g), return; end
cfg = C(G(g).caso(1,1)).cfg;
Kt = cfg.ext.Ktraj(1,1); Kdh = cfg.ext.Kdh(1,1); tw = cfg.ext.tau_w;
w2 = cfg.planta.g/cfg.planta.l;
kv = linspace(0, 1, 101); kh = linspace(0.5, 7, 131);
f = nueva_fig(14, 15); hold on; box on; grid on;
% Routh boundary (no washout) and boundary of the model with washout
h1 = plot(kv, (1 + Kdh)*Kt./(1 + kv), 'k-', 'LineWidth', 1.4);
S = zeros(numel(kh), numel(kv));
for a = 1:numel(kh)
    for b = 1:numel(kv)
        S(a, b) = max(real(roots(poly_lazo(Kt, kh(a), Kdh, kv(b), tw, w2))));
    end
end
contour(kv, kh, S, [0 0], 'LineColor', [0.2 0.2 0.2], 'LineStyle', '--', 'LineWidth', 1.2);
h2 = plot(nan, nan, '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);   % for the legend
% runs
cls = {'estable', 'oscilante', 'creciente', 'divergencia'};
cls_en = {'stable', 'sustained oscillation', 'growing oscillation', 'run aborted (diverged)'};
mk = {'o', 's', '^', 'x'};
cc = [0.165 0.471 0.839; 0.910 0.604 0.110; 0.831 0.294 0.235; 0.831 0.294 0.235];
hc = zeros(1, 4);
for a = 1:numel(G(g).Kv_vals)
    for j = 1:numel(G(g).vals)
        c = G(g).caso(a, j);
        if isempty(C(c).X), continue; end
        k = find(strcmp(cls, peor_clase(C(c).X)));
        x = G(g).Kv_vals(a); y = G(g).vals(j);
        hc(k) = plot(x, y, mk{k}, 'Color', cc(k,:), 'MarkerFaceColor', ifs(k <= 3, cc(k,:), 'none'), ...
            'MarkerSize', 8, 'LineWidth', 1.5);
        r = mean_v(cellfun(@(X) X.rmse_ss, C(c).X(cellfun(@(X) X.completo, C(c).X))));
        if isfinite(r), text(x + 0.025, y + 0.12, sprintf('%.2f', r), 'FontSize', 7); end
    end
end
xlabel('K_v'); ylabel('K_h'); xlim([-0.05 1]); ylim([0.5 7]);
hh = [h1 h2]; ll = {'K_h(1+K_v) = (1+K_{dh})K_{traj} (Routh, no washout)', 'Linear model with washout, max Re(s) = 0'};
for k = 1:4
    if hc(k) ~= 0, hh(end+1) = hc(k); ll{end+1} = cls_en{k}; end %#ok<AGROW>
end
legend(hh, ll, 'Location', 'southoutside', 'FontSize', 7);
title(sprintf('Labels: RMSE ||x_{tilde}|| for t \\geq %g s [m]', op.t_ss), 'FontWeight', 'normal');
guardar_fig(f, fullfile(dfig, 'mapa_Kh_Kv'));
end

function fig_sensibilidad(G, T, gs, filas, base)
% One column per group (gs) and one row per metric (filas: key, label).
if isempty(gs), return; end
D = T.D;
f = nueva_fig(max(3.8*numel(gs) + 2, 10), 4.3*size(filas, 1));
for q = 1:numel(gs)
    g = gs(q);
    F = T.filas(cellfun(@(x) strcmp(x.grupo, G(g).nombre), T.filas));
    for r = 1:size(filas, 1)
        i = find(strcmp(D(:,1), filas{r,1}));
        subplot(size(filas,1), numel(gs), (r-1)*numel(gs) + q); hold on; box on; grid on;
        m = cellfun(@(x) x.media(i), F); s = cellfun(@(x) x.desv(i), F); s(~isfinite(s)) = 0;
        n = numel(m);
        he = errorbar(1:n, m, s);
        set(he, 'Color', [0.165 0.471 0.839], 'Marker', 'o', 'MarkerFaceColor', [0.165 0.471 0.839], 'LineStyle', '-');
        bad = cellfun(@(x) ~strcmp(x.clase, 'estable'), F);
        if any(bad), plot(find(bad), m(bad), 'x', 'Color', [0.83 0.29 0.24], 'MarkerSize', 10, 'LineWidth', 1.5); end
        jn = find(abs(G(g).vals - G(g).nom) < 1e-12, 1);
        ytop = max(m + s); if ~isfinite(ytop) || ytop <= 0, ytop = 1; end
        ylim([0 1.15*ytop]);                                  % axis from 0: the real scale of the difference
        if ~isempty(jn), plot([jn jn], [0 1.15*ytop], 'k:'); end
        set(gca, 'XTick', 1:n, 'XTickLabel', arrayfun(@(v) sprintf('%g', v), G(g).vals, 'UniformOutput', false), ...
            'FontSize', 7);
        xlim([0.5 n + 0.5]);
        if q == 1, ylabel(filas{r,2}); end
        if r == size(filas,1), xlabel(G(g).etq); end
        if r == 1, title(ifs(strcmp(G(g).esc, 'lemn'), [G(g).etq ' (lemniscate)'], G(g).etq)); end
    end
end
guardar_fig(f, base);
end

function fig_rafaga(G, C, op, dfig)
% ||v_tilde||(t), ||s||(t), and ||x_tilde||(t) around the gust, one column per K1, K2 group.
gs = find(strcmp({G.esc}, 'raf'));
if isempty(gs), return; end
ton = op.rafaga(1,1); toff = op.rafaga(1,2);
ta = ton - 5; tb = min(toff + 12, op.T_raf);
var = {'vt', '||v_{tilde}|| [m/s]'; 's', '||s|| [m/s]'; 'e', '||x_{tilde}|| [m]'};
f = nueva_fig(8*numel(gs) + 4, 15);
for q = 1:numel(gs)
    g = gs(q); nv = numel(G(g).vals); col = paleta(nv);
    for r = 1:size(var, 1)
        subplot(size(var, 1), numel(gs), (r-1)*numel(gs) + q); hold on; box on; grid on;
        h = zeros(1, nv); leg = cell(1, nv); ymax = 0;
        % shaded gust, drawn first so it stays behind the curves
        patch([ton toff toff ton], [0 0 1e3 1e3], [0.92 0.92 0.92], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        for j = 1:nv
            c = G(g).caso(1, j);
            [tg, ym, ys] = media_series(C(c).S, var{r,1}, 0.02);
            if isempty(tg), continue; end
            sel = tg >= ta & tg <= tb;
            banda(tg(sel), max(ym(sel) - ys(sel), 0), ym(sel) + ys(sel), col(j,:));
            nom = abs(G(g).vals(j) - G(g).nom) < 1e-12;
            h(j) = plot(tg(sel), ym(sel), '-', 'Color', col(j,:), 'LineWidth', 1.0 + 0.8*nom);
            leg{j} = sprintf('\\times %g%s', G(g).vals(j), ifs(nom, ' (nom.)', ''));
            ymax = max(ymax, max(ym(sel) + ys(sel)));
        end
        if ymax > 0, ylim([0 1.1*ymax]); end
        xlim([ta tb]);
        if q == 1, ylabel(var{r,2}); end
        if r == size(var, 1), xlabel('Time [s]'); end
        if r == 1
            title(sprintf('%s (gust %g m/s, shaded)', ifs(G(g).nombre(2) == '1', 'K_1', 'K_2'), ...
                norm(op.rafaga(1, 3:5))));
            ok = h ~= 0;
            if any(ok), legend(h(ok), leg(ok), 'Location', 'northeast', 'FontSize', 7); end
        end
    end
end
guardar_fig(f, fullfile(dfig, 'conv_K1_K2'));
end

function fig_sta(G, C, op, dfig)
grs = {'Ksta', 'phi_bl'};
grs = grs(cellfun(@(n) ~isempty(buscar_grupo(G, n)), grs));
if isempty(grs), return; end
f = nueva_fig(16, 13);
for q = 1:numel(grs)
    g = buscar_grupo(G, grs{q});
    nv = numel(G(g).vals); col = paleta(nv);
    for r = 1:2
        subplot(2, numel(grs), (r-1)*numel(grs) + q); hold on; box on; grid on;
        h = zeros(1, nv); leg = cell(1, nv);
        for j = 1:nv
            c = G(g).caso(1, j);
            if isempty(C(c).S), continue; end
            s = C(c).S{1};
            sel = s.t >= op.t_ss & s.t <= op.t_ss + 5;
            if r == 1, y = s.s(1, sel); else, y = s.u(1, sel); end
            h(j) = plot(s.t(sel), y, '-', 'Color', col(j,:), 'LineWidth', 1.0);
            leg{j} = sprintf('%s = %g', G(g).etq, G(g).vals(j));
        end
        if r == 1, ylabel('s_x [m/s]'); title(G(g).etq);
        else, ylabel('u_x [m/s]'); xlabel('Time [s]'); end
        ok = h ~= 0; if any(ok) && r == 1, legend(h(ok), leg(ok), 'Location', 'southoutside', 'FontSize', 7); end
    end
end
guardar_fig(f, fullfile(dfig, 'sta_s'));
end

function fig_adaptacion(G, C, op, dfig) %#ok<INUSL>
g = buscar_grupo(G, 'Gamma');
if isempty(g), return; end
nv = numel(G(g).vals); col = paleta(nv);
f = nueva_fig(16, 14);
etq = {'\theta_{1,x} (= 1/\kappa_u)', '\theta_{2,x} (= \kappa_v/\kappa_u)', '||\nu|| (2 s moving RMS) [m/s]'};
for r = 1:3
    subplot(3, 1, r); hold on; box on; grid on;
    h = zeros(1, nv); leg = cell(1, nv); sv = [];
    for j = 1:nv
        c = G(g).caso(1, j);
        if isempty(C(c).S), continue; end
        if r < 3
            [tg, ym] = media_series(C(c).S, 'theta', 0.05, r);
        else
            [tg, ym] = media_series(C(c).S, 'nu', 0.05);
            W = round(2/0.05); y2 = ym.^2; ym2 = y2;
            for i = 1:numel(y2), ym2(i) = sqrt(mean(y2(max(1, i-W):i))); end
            ym = ym2;
        end
        h(j) = plot(tg, ym, '-', 'Color', col(j,:), 'LineWidth', 1.0 + 0.8*(abs(G(g).vals(j) - G(g).nom) < 1e-12));
        leg{j} = sprintf('\\Gamma = %g%s', G(g).vals(j), ifs(G(g).vals(j) == 0, ' (fixed)', ''));
        if isempty(sv), sv = C(c).S{1}.sust; end
    end
    if r < 3 && ~isempty(sv)
        xl = xlim; plot(xl, sv(r)*[1 1], 'k--'); % true value (surrogate plant only)
    end
    ylabel(etq{r});
    ok = h ~= 0; if any(ok) && r == 1, legend(h(ok), leg(ok), 'Location', 'eastoutside', 'FontSize', 7); end
    if r == 3, xlabel('Time [s]'); end
    if r == 1, title('Adaptation gain \\Gamma (lemniscate; \\Gamma = 0: fixed \\theta)'); end
end
guardar_fig(f, fullfile(dfig, 'adaptacion'));
end

function [tg, ym, ys] = media_series(S, campo, dtg, fila_c)
% mean and std across complete repetitions, on a common grid
if nargin < 4, fila_c = 0; end
S = S(cellfun(@(s) s.completo, S));
tg = []; ym = []; ys = [];
if isempty(S), return; end
Tfin = min(cellfun(@(s) s.t(end), S));
tg = 0:dtg:Tfin;
Y = nan(numel(S), numel(tg));
for k = 1:numel(S)
    s = S{k};
    y = s.(campo);
    if fila_c > 0, y = y(fila_c, :); elseif size(y, 1) > 1, y = sqrt(sum(y.^2, 1)); end
    [tu, iu] = unique(s.t);
    Y(k, :) = interp1(tu, y(iu), tg, 'linear', NaN);
end
ym = mean(Y, 1);
if numel(S) > 1, ys = std(Y, 0, 1); else, ys = zeros(size(ym)); end
end

function banda(t, lo, hi, c)
ok = isfinite(lo) & isfinite(hi);
if ~any(ok) || all(hi(ok) == lo(ok)), return; end
t = t(ok); lo = lo(ok); hi = hi(ok);
try
    patch([t fliplr(t)], [lo fliplr(hi)], c, 'EdgeColor', 'none', 'FaceAlpha', 0.18, 'HandleVisibility', 'off');
catch
    patch([t fliplr(t)], [lo fliplr(hi)], 0.82 + 0.18*c, 'EdgeColor', 'none');
end
end

function c = paleta(n)
% categorical, distinguishable in color and in grayscale (order = increasing value)
P = [0.831 0.294 0.235; 0.910 0.604 0.110; 0.227 0.620 0.361; 0.165 0.471 0.839; 0.106 0.184 0.431; 0.541 0.310 0.702];
c = P(1 + mod(0:n-1, size(P, 1)), :);
end

function f = nueva_fig(ancho_cm, alto_cm)
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', 'Position', [2 2 ancho_cm alto_cm]);
end

function guardar_fig(f, base)
set(f, 'PaperUnits', 'centimeters');
p = get(f, 'Position'); set(f, 'PaperSize', [p(3) p(4)]); set(f, 'PaperPosition', [0 0 p(3) p(4)]);
print(f, [base '.png'], '-dpng', '-r200');
try, print(f, [base '.pdf'], '-dpdf'); catch, end %#ok<NOCOM>
close(f);
end

function intentar(fh, nombre)
try
    fh();
catch err
    fprintf('  figura %s: %s\n', nombre, err.message);
end
end

%% ===================================================================================
%% Files and utilities
%% ===================================================================================
function a = arch_raw(op, C, k)
a = fullfile(op.salida, 'raw', sprintf('%s_rep%d.mat', C.id, k));
end

function [R, M, X, ok] = leer_raw(a)
R = []; M = []; X = []; ok = false;
if ~exist(a, 'file'), return; end
try
    S = load(a, 'R', 'M', 'X');
    R = S.R; M = S.M; X = S.X;
    ok = ~strcmp(R.motivo, 'simulador_detenido');
catch
    ok = false;
end
end

function guardar_raw(a, R, M, X) %#ok<INUSD>
tmp = [a '.tmp.mat'];
save(tmp, 'R', 'M', 'X', '-v7');
movefile(tmp, a, 'f');
end

function f = firma(op)
f = sprintf('%s|%d|%d|%s|%g|%g|%g|%g|%s|%s', op.backend, op.n_rep, op.n_rep_mapa, strjoin(op.grupos, ','), ...
    op.T, op.T_adapt, op.T_raf, op.t_ss, mat2str(op.offset(:).'), mat2str(op.rafaga));
end

function d = buscar_pendiente(base, fm)
d = '';
L = dir(fullfile(base, 'barrido_*'));
L = L([L.isdir]);
if isempty(L), return; end
[~, o] = sort({L.name});
for i = numel(o):-1:1
    c = fullfile(base, L(o(i)).name);
    a = fullfile(c, 'en_curso.mat');
    if exist(a, 'file')
        try
            P = load(a);
            if strcmp(P.firma, fm), d = c; return; end
        catch
        end
    end
end
end

function escribir_csv(archivo, filas)
fid = fopen(archivo, 'w');
c = fieldnames(filas{1});
fprintf(fid, '%s', c{1}); fprintf(fid, ',%s', c{2:end}); fprintf(fid, '\n');
for i = 1:numel(filas)
    for j = 1:numel(c)
        x = filas{i}.(c{j});
        if j > 1, fprintf(fid, ','); end
        if ischar(x), fprintf(fid, '%s', x); else, fprintf(fid, '%.6g', x); end
    end
    fprintf(fid, '\n');
end
fclose(fid);
end

function s = fmt_tex(m, d, f)
if ~isfinite(m), s = '--';
elseif ~isfinite(d) || d == 0, s = ['$' sprintf(f, m) '$'];
else, s = ['$' sprintf(f, m) ' \pm ' sprintf(strrep(f, '+', ''), d) '$']; end
end

function s = fmt_num(x, f)
if isfinite(x), s = sprintf(f, x); else, s = '--'; end
end

function y = p99(v)
v = sort(v(isfinite(v))); if isempty(v), y = nan; return; end
y = v(max(1, ceil(0.99*numel(v))));
end

function w = hann_w(n), w = 0.5 - 0.5*cos(2*pi*(0:n-1)/max(n-1, 1)); end
function r = rms1(x), if isempty(x), r = nan; else, r = sqrt(mean(x.^2)); end, end
function m = maxv(x), if isempty(x), m = nan; else, m = max(x); end, end
function m = mean_v(v), if isempty(v), m = nan; else, m = mean(v); end, end
function s = std_v(v), if numel(v) < 2, s = nan; else, s = std(v); end, end
function x = campo_o(S, c, def), if isfield(S, c), x = S.(c); else, x = def; end, end
function y = ifs(c, a, b), if c, y = a; else, y = b; end, end
function mkdir_p(d), if ~exist(d, 'dir'), mkdir(d); end, end

function semilla(s)
if exist('OCTAVE_VERSION', 'builtin'), rand('seed', s); randn('seed', s); else, rng(s); end %#ok<RAND>
end
