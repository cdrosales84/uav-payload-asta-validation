function comparar_lv_vs_asta(varargin)
% COMPARAR_LV_VS_ASTA  Misma planta y la misma referencia para dos controladores.
%
%   comparar_lv_vs_asta()
%   comparar_lv_vs_asta('backend','sustituta','T',20)   % prueba; no va al manuscrito
%   comparar_lv_vs_asta('escenarios',{'E4_circ_asta','E5_lemn_viento'},'n_rep',1)
%   comparar_lv_vs_asta('escenarios',{'E7_masa_300_asta'},'masa_modelo',0.1)
%
% Controladores:
%   asta  controlador_paso.m, con la cfg del escenario (lazo cinematico + A-STA)
%   lv    controlador_lv.m, cascada de Lv, Wu y Wang (IEEE TVT 2020)
%
% masa_modelo fija la masa que usa la ley de Lv. Vacio: usa la masa de la planta
% de ese escenario (masa conocida). En un barrido de masa, pasar 0.1 deja el
% modelo de Lv en el valor nominal mientras la planta cambia.
%
% Las metricas de carga salen de metricas_payload (velocidad de la carga por
% diferencias centrales de x_l). La tabla no usa las metricas del estimador ni
% las de theta: en Lv esos campos no son el lazo interno del manuscrito.
%
% Ejecutar con paper_revision como carpeta actual.

op = struct('backend', 'simquad', 'n_rep', 1, 'escenarios', {{'E4_circ_asta'}}, ...
    'salida', '', 'T', [], 'masa_modelo', []);
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'simulations'));

if isempty(op.salida)
    op.salida = fullfile(aqui, 'results', ['comp_lv_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~exist(op.salida, 'dir'), mkdir(op.salida); end

leyes = {'lv', 'asta'};
nombres = {'Lv TVT 2020', 'Propuesto'};
filas = {};

for e = 1:numel(op.escenarios)
    esc = op.escenarios{e};
    for k = 1:op.n_rep
        for j = 1:numel(leyes)
            cfg = cfg_paper(esc);
            cfg.planta.backend = op.backend;
            if strcmp(op.backend, 'sustituta'), cfg.planta.usar_dt_real = false; end
            if ~isempty(op.T), cfg.planta.T = op.T; end
            cfg.lv = parametros_lv(cfg, op.masa_modelo);
            if cfg.planta.T >= 20, cfg.met.t_inicio = 5; else, cfg.met.t_inicio = 0; end
            if exist('OCTAVE_VERSION', 'builtin')
                rand('seed', cfg.exp.semilla + k); randn('seed', cfg.exp.semilla + k); %#ok<RAND>
            else
                rng(cfg.exp.semilla + k);
            end
            fprintf('[%s] %s | %s | rep %d/%d\n', datestr(now,'HH:MM:SS'), esc, leyes{j}, k, op.n_rep);
            if strcmp(leyes{j}, 'asta')
                R = simular(@controlador_paso, cfg);
            else
                R = simular(@controlador_lv, cfg);
            end
            R.ley = leyes{j};
            R.escenario = esc;
            R.rep = k;
            M = metricas_payload(R);
            M.ley = leyes{j};
            M.escenario = esc;
            M.rep = k;
            M.completo = R.completo;
            M.t_final = R.t(end);
            save(fullfile(op.salida, sprintf('%s_%s_rep%d.mat', esc, leyes{j}, k)), 'R', 'M', '-v7.3');
            filas{end+1} = M; %#ok<AGROW>
            fprintf('  completo=%d  t=%.1f s  RMSE_x=%.4f  RMSE_v=%.4f  max_alpha=%.3f rad\n', ...
                R.completo, R.t(end), M.rmse_ex, M.rmse_evl, M.max_alpha);
        end
    end
end

imprimir_tabla(filas, leyes, nombres);
guardar_tabla(filas, leyes, nombres, op.salida);
fprintf('Resultados en %s\n', op.salida);
end

%% ------------------------------------------------------------------------
function P = parametros_lv(cfg, masa_modelo)
% Ganancias publicadas en Lv et al., IFAC 2020, Sec. 3.1 (misma cascada que TVT 2020):
%   k11 = k12 = 0.5, k13 = 3,  k7 = k8 = k9 = k10 = 6,  D_delta = 0.08.
P.mp = cfg.planta.mL;
if ~isempty(masa_modelo), P.mp = masa_modelo; end
P.Kv  = diag([0.5 0.5 3.0]);
% k7..k10 = 6 con el cable de 0.505 m de ese articulo. El comando de este
% simulador corrige con L*K*e (m/s). Se escala K con sqrt(0.505/L) al pasar
% a L = 2 m; si no, la ganancia 6 pide una correccion de velocidad de 12 m/s
% por radian.
L_paper = 0.505;
Ksw = 6 * sqrt(L_paper / cfg.planta.l);
P.Ks  = diag([Ksw Ksw]);
P.Kps = diag([Ksw Ksw]);
P.Dd  = 0.08 * eye(3);
P.Kp  = diag([1.0 1.0 0.4]);     % extension de posicion; el articulo no publica este PID
P.sigma_max = 0.40;              % [rad] tope de (30): por encima de esto el cable deja el regimen lineal
P.k_yaw = 1.0;                   % reemplaza el par de yaw de (24)
P.tau_med = 0.06;                % [s] filtro de v_l y de sigma_dot, que el simulador no mide
P.tau_ref = 0.50;                % [s] v_ld filtrada: el teorema supone aceleracion de referencia nula
P.tau_sigma = 0.30;              % [s] filtro de (alpha_d, beta_d)
P.u_max = cfg.int.u_max;
P.w_max = 2.0;
end

function R = simular(fh, cfg)
% Mismo lazo que sim_payload_asta, con el controlador elegido por fh.
planta = str2func(['planta_' cfg.planta.backend]);
es_rt = strcmp(cfg.planta.backend, 'simquad');
dt0 = cfg.planta.dt;
Nmax = ceil(1.2*cfg.planta.T/dt0) + 10;
r0 = referencia_carga(0, cfg.ref);
x0 = r0.p + [0; 0; cfg.planta.l];
planta('iniciar', cfg, x0, r0.psi);

Z = @(n) nan(n, Nmax);
L = struct('t',Z(1),'dt',Z(1),'x',Z(3),'v',Z(3),'psi',Z(1),'xl',Z(3), ...
    'xd',Z(3),'vd',Z(3),'ad',Z(3),'jd',Z(3),'psid',Z(1), ...
    'h',Z(3),'hp',Z(3),'hs',Z(2),'ex',Z(3),'epsi',Z(1), ...
    'vc',Z(3),'ukh',Z(4),'vh',Z(2),'vt',Z(2),'s',Z(2),'nu',Z(2),'w',Z(2), ...
    'phith',Z(2),'theta',Z(4),'ad_on',Z(2),'sat',Z(2),'u',Z(4),'viento',Z(3), ...
    'xl_crudo',Z(3),'corr_xl',Z(1));

st = []; t_prev = []; t0 = []; k = 0; motivo = 'fin'; w_ant = [0;0;0]; stc = [];
if es_rt, tic; end
while true
    if es_rt
        while toc < dt0, end
        tic;
    end
    m = planta('leer');
    if ~m.ok, motivo = 'simulador_detenido'; break; end
    if isempty(t0), t0 = m.t; end
    if cfg.planta.usar_dt_real, t = m.t - t0; else, t = k*dt0; end
    if isempty(t_prev), dtk = dt0; else, dtk = t - t_prev; end
    if dtk <= 0, dtk = dt0; end
    t_prev = t;
    if t > cfg.planta.T || k >= Nmax, break; end
    k = k + 1;

    xl_crudo = m.xl; corr = false;
    if es_rt, [m, stc, corr] = corregir_lectura_carga(m, stc, t); end
    rr = referencia_carga(t, cfg.ref);
    [u, st, lg] = fh(cfg, st, m, rr, t, dtk);
    w = viento_de(t, cfg.viento.bloques);
    if any(w ~= w_ant), planta('viento', w); w_ant = w; end
    planta('comandar', u);

    L.t(k) = t; L.dt(k) = dtk; L.x(:,k) = m.x; L.v(:,k) = m.v; L.psi(k) = m.psi; L.xl(:,k) = m.xl;
    L.xd(:,k) = rr.p; L.vd(:,k) = rr.v; L.ad(:,k) = rr.a; L.jd(:,k) = rr.j; L.psid(k) = rr.psi;
    L.h(:,k) = lg.h; L.hp(:,k) = lg.hp; L.hs(:,k) = lg.hs_h;
    L.ex(:,k) = lg.ex; L.epsi(k) = lg.epsi; L.vc(:,k) = lg.v_c; L.ukh(:,k) = lg.uk_h;
    L.vh(:,k) = lg.v_h; L.vt(:,k) = lg.vt; L.s(:,k) = lg.s; L.nu(:,k) = lg.nu; L.w(:,k) = lg.w;
    L.phith(:,k) = lg.phith; L.theta(:,k) = lg.theta(:); L.ad_on(:,k) = lg.ad_on;
    L.sat(:,k) = lg.sat; L.u(:,k) = lg.u; L.viento(:,k) = w;
    L.xl_crudo(:,k) = xl_crudo; L.corr_xl(k) = corr;

    alpha = atan2(norm(lg.h(1:2)), lg.h(3));
    if alpha > cfg.exp.alpha_abort || norm(rr.p - m.xl) > cfg.exp.ex_abort
        motivo = 'divergencia'; break;
    end
end
planta('cerrar');

f = fieldnames(L);
for i = 1:numel(f), L.(f{i}) = L.(f{i})(:, 1:k); end
R = L;
R.cfg = cfg;
R.motivo = motivo;
R.completo = strcmp(motivo, 'fin');
R.n_corr_xl = sum(R.corr_xl == 1);
R.fecha = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

function w = viento_de(t, B)
w = [0;0;0];
for i = 1:size(B,1)
    if t >= B(i,1) && t < B(i,2), w = B(i,3:5).'; end
end
end

function imprimir_tabla(filas, leyes, nombres)
fprintf('\n%-22s', 'Metrica');
for j = 1:numel(leyes), fprintf('  %-16s', nombres{j}); end
fprintf('  Delta [%%]\n');
fprintf('%s\n', repmat('-', 1, 22 + 18*numel(leyes) + 12));
claves = {'rmse_ex','rmse_evl','rms_alpha','max_alpha','rms_rho','max_rho','rms_vperp','max_vperp','rms_du','tv_u'};
etiq = {'RMSE x [m]','RMSE v [m/s]','RMS alpha [rad]','Max alpha [rad]', ...
    'RMS rho [m]','Max rho [m]','RMS v_T [m/s]','Max v_T [m/s]','RMS du [1/s]','TV u [-]'};
esc = unique(cellfun(@(f) f.escenario, filas, 'UniformOutput', false), 'stable');
for e = 1:numel(esc)
    fprintf('\n%s\n', esc{e});
    G = filas(cellfun(@(f) strcmp(f.escenario, esc{e}), filas));
    for i = 1:numel(claves)
        val = nan(1, numel(leyes));
        for j = 1:numel(leyes)
            Gj = G(cellfun(@(f) strcmp(f.ley, leyes{j}), G));
            if isempty(Gj), continue; end
            val(j) = media_clave(Gj, claves{i});
        end
        fprintf('%-22s', etiq{i});
        for j = 1:numel(leyes), fprintf('  %16.4f', val(j)); end
        if all(isfinite(val)) && abs(val(1)) > 1e-9
            fprintf('  %+8.1f', 100*(val(1) - val(2))/val(1));
        else
            fprintf('  %8s', 'n/a');
        end
        fprintf('\n');
    end
    for j = 1:numel(leyes)
        Gj = G(cellfun(@(f) strcmp(f.ley, leyes{j}), G));
        nc = mean(cellfun(@(f) double(f.completo), Gj));
        tf = mean(cellfun(@(f) f.t_final, Gj));
        fprintf('%-22s  completo %.0f %%   t_final %.1f s\n', nombres{j}, 100*nc, tf);
    end
end
fprintf('\nDelta positivo: el propuesto tiene la metrica mas baja.\n');
end

function v = media_clave(G, clave)
c = cellfun(@(f) f.(clave), G, 'UniformOutput', false);
if strcmp(clave, 'rms_du') || strcmp(clave, 'tv_u')
    M = cell2mat(cellfun(@(x) x(:), c, 'UniformOutput', false));
    v = mean(M(:));
else
    nums = cellfun(@(x) x(1), c);
    nums = nums(isfinite(nums));
    if isempty(nums), v = nan; else, v = mean(nums); end
end
end

function guardar_tabla(filas, leyes, nombres, carpeta)
fid = fopen(fullfile(carpeta, 'tabla_comparacion.txt'), 'w');
fprintf(fid, 'Comparacion Lv, Wu, Wang (IEEE TVT 2020) vs controlador_paso\n');
fprintf(fid, 'Delta positivo favorece al propuesto (metrica mas baja).\n');
claves = {'rmse_ex','rmse_evl','rms_alpha','max_alpha','rms_rho','max_rho','rms_vperp','max_vperp'};
esc = unique(cellfun(@(f) f.escenario, filas, 'UniformOutput', false), 'stable');
for e = 1:numel(esc)
    fprintf(fid, '\n%s\n', esc{e});
    fprintf(fid, '%-16s', 'metrica');
    for j = 1:numel(nombres), fprintf(fid, '\t%s', nombres{j}); end
    fprintf(fid, '\tdelta_pct\n');
    G = filas(cellfun(@(f) strcmp(f.escenario, esc{e}), filas));
    for i = 1:numel(claves)
        val = nan(1, numel(leyes));
        for j = 1:numel(leyes)
            Gj = G(cellfun(@(f) strcmp(f.ley, leyes{j}), G));
            val(j) = media_clave(Gj, claves{i});
        end
        if isfinite(val(1)) && abs(val(1)) > 1e-9
            dlt = 100*(val(1) - val(2))/val(1);
        else
            dlt = nan;
        end
        fprintf(fid, '%-16s\t%.6f\t%.6f\t%.2f\n', claves{i}, val(1), val(2), dlt);
    end
end
fclose(fid);
end
