function T = comparar_omar_vs_asta(varargin)
% COMPARAR_OMAR_VS_ASTA  External baseline of the benchmark section: Omar (2022) vs proposed.
%
%   comparar_omar_vs_asta()                                   % SimQuad1.exe, 5 repetitions
%   comparar_omar_vs_asta('escenarios',{'E4_circ_asta'},'n_rep',1)   % short test
%   comparar_omar_vs_asta('backend','sustituta','n_rep',1,'T',30)    % code test
%
% Controllers (all three run with sim_controlador, same test bench):
%   px4   PX4-type position tracker without anti-swing (common baseline reference)
%   omar  px4 + delayed-feedback anti-swing, Omar, Appl. Sci. 12:1706 (2022)
%   asta  proposed controller (controlador_paso.m, scenario cfg)
%
% Before the final run: ajustar_omar() on SimQuad (it leaves omar_ajuste.mat).
% Without that file, K = 0.6 (the paper) and tau = T_L/4 are used, and the table says so.
%
% Output in results/comp_omar_<fecha>/:
%   raw/<esc>_<ley>_rep<k>.mat    full series R, metrics M, cfg (in R.cfg)
%   resumen_omar.csv              one row per run
%   tabla_omar.tex                mean +- std, Delta %, and p (Welch) Omar vs proposed
%   tabla_omar.txt                the same in text
%   figuras/<esc>_*.png|pdf       alpha, ||x_tilde||, ||v_perp|| (mean of repetitions)
%   omar_parametros.txt           K, tau, T_L, Kp, and the origin of the tuning
%
% Resume. Each run is saved in raw/ as soon as it finishes. If MATLAB closes,
% call comparar_omar_vs_asta() again with the same arguments: the latest unfinished
% folder with that configuration is resumed, saved runs are read from raw/, and
% it continues from the next one. Only the run in progress is lost.
% To resume a specific folder: 'salida', 'results/comp_omar_<fecha>'.
% 'reanudar', false forces a new folder.
%
% SimQuad stopped. If simulation1 stops responding in the middle of a run,
% SimQuad1.exe is killed and that run is repeated (up to 'reintentos' times, 2 by
% default). Discarded attempts remain in raw/*_intento<i>.mat and their count
% appears in the table. A controller divergence is not repeated.
% Table metrics come from the complete runs; the "complete runs" row says how many.
%
% Every metric comes from the simulator ground truth (metricas_payload.m), with the
% window t >= t_inicio (5 s by default, after the sigma(t) ramp).

op = struct('backend', 'simquad', 'n_rep', 5, ...
    'escenarios', {{'E4_circ_asta', 'E5_lemn_viento'}}, ...
    'leyes', {{'px4', 'omar', 'asta'}}, 'salida', '', 'T', [], 't_inicio', 5, ...
    'figuras', true, 'reanudar', true, 'reintentos', 2);
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
sim_dir = fullfile(aqui, '..', 'simulations');
if exist(sim_dir, 'dir'), addpath(sim_dir); end
fm = firma(op);
if isempty(op.salida) && op.reanudar
    op.salida = buscar_pendiente(fullfile(aqui, 'results'), fm);
end
if isempty(op.salida)
    op.salida = fullfile(aqui, 'results', ['comp_omar_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
mkdir_p(op.salida);
arch_prog = fullfile(op.salida, 'en_curso.mat');
if exist(arch_prog, 'file')
    P0 = load(arch_prog);
    if ~strcmp(P0.firma, fm)
        error('comparar_omar_vs_asta:firma', ['%s es de otra configuracion (escenarios, ', ...
            'leyes, repeticiones, T o backend). Usar la misma o otra carpeta.'], op.salida);
    end
    fprintf('Reanudando en %s\n', op.salida);
end
Pg.firma = fm;
save(arch_prog, '-struct', 'Pg');
mkdir_p(fullfile(op.salida, 'raw'));
if op.figuras, mkdir_p(fullfile(op.salida, 'figuras')); end

NOM = struct('px4', 'PX4-type tracking (no ASC)', 'omar', 'Omar (2022) delayed-feedback ASC', ...
    'asta', 'Proposed (kinematic + A-STA)');

filas = {};
S = struct();                                         % series for figures
for e = 1:numel(op.escenarios)
    esc = op.escenarios{e};
    for k = 1:op.n_rep
        for j = 1:numel(op.leyes)
            ley = op.leyes{j};
            cfg = armar_cfg(op, esc, ley);
            semilla(cfg.exp.semilla + k);
            fprintf('[%s] %s | %-4s | rep %d/%d ... ', datestr(now, 'HH:MM:SS'), esc, ley, k, op.n_rep);
            arch_raw = fullfile(op.salida, 'raw', sprintf('%s_%s_rep%d.mat', esc, ley, k));
            [R, M, ya] = leer_raw(arch_raw);
            if ya
                fprintf('guardada, t = %.1f s, RMSE x = %.3f m, ISH = %.3f\n', R.t(end), M.rmse_ex, M.ish);
                filas{end+1} = fila(esc, ley, k, M); %#ok<AGROW>
                if op.figuras, S.(campo(esc)).(ley){k} = serie_fig(R, M); end
                continue;
            end
            if strcmp(ley, 'asta'), fh = @controlador_paso; else, fh = @controlador_omar; end
            [R, nfall] = sim_con_reintento(fh, cfg, op.reintentos, arch_raw(1:end-4));
            R.ley = ley; R.escenario = esc; R.rep = k; R.reintentos = nfall;
            M = metricas_payload(R);
            M.reintentos = nfall;
            [M.ish, M.ish_dev] = ish_omar(R, cfg.met.t_inicio);
            fprintf('%s, t = %.1f s, RMSE x = %.3f m, ISH = %.3f\n', R.motivo, R.t(end), M.rmse_ex, M.ish);
            guardar_raw(arch_raw, R, M);
            filas{end+1} = fila(esc, ley, k, M); %#ok<AGROW>
            if op.figuras
                S.(campo(esc)).(ley){k} = serie_fig(R, M);
            end
        end
    end
end

escribir_csv(fullfile(op.salida, 'resumen_omar.csv'), filas);
T = tabla(filas, op, NOM);
escribir_tex(T, fullfile(op.salida, 'tabla_omar.tex'), op);
escribir_txt(T, fullfile(op.salida, 'tabla_omar.txt'), op);
escribir_parametros(fullfile(op.salida, 'omar_parametros.txt'), armar_cfg(op, op.escenarios{1}, 'omar'));
if op.figuras
    for e = 1:numel(op.escenarios)
        figuras(S.(campo(op.escenarios{e})), op, op.escenarios{e}, NOM, fullfile(op.salida, 'figuras'));
    end
end
if exist(arch_prog, 'file'), delete(arch_prog); end   % run complete
fprintf('\nResultados en %s\n', op.salida);
end

%% ---- resume -------------------------------------------------------------
function f = firma(op)
f = sprintf('%s|%d|%s|%s|%g|%s', op.backend, op.n_rep, strjoin(op.escenarios, ','), ...
    strjoin(op.leyes, ','), op.t_inicio, mat2str(op.T));
end

function d = buscar_pendiente(base, fm)
% Latest comp_omar_* folder with en_curso.mat of the same configuration.
d = '';
L = dir(fullfile(base, 'comp_omar_*'));
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

function [R, M, ok] = leer_raw(a)
R = []; M = []; ok = false;
if ~exist(a, 'file'), return; end
try
    S = load(a, 'R', 'M');
    R = S.R; M = S.M;
    ok = isfield(M, 'ish') && ~strcmp(R.motivo, 'simulador_detenido');   % a run cut by SimQuad is repeated
catch
    ok = false;                                       % file cut by a close: it is repeated
end
end

function guardar_raw(a, R, M) %#ok<INUSD>
% Written to a temp file and renamed: a close mid-save does not leave a broken .mat.
tmp = [a '.tmp.mat'];
save(tmp, 'R', 'M', '-v7');
movefile(tmp, a, 'f');
end

%% ------------------------------------------------------------------------
function cfg = armar_cfg(op, esc, ley)
cfg = cfg_paper(esc);
cfg.planta.backend = op.backend;
if strcmp(op.backend, 'sustituta'), cfg.planta.usar_dt_real = false; end
if ~isempty(op.T), cfg.planta.T = op.T; end
cfg.met.t_inicio = op.t_inicio;
switch ley
    case 'asta'                                        % the scenario cfg, unchanged
    case 'omar', cfg.omar = parametros_omar(cfg);
    case 'px4',  cfg.omar = parametros_omar(cfg, 'activo', false);
    otherwise, error('comparar_omar_vs_asta:ley', 'Ley desconocida: %s', ley);
end
cfg.comparacion = struct('ley', ley, 'controlador', ley);
end

function f = fila(esc, ley, k, M)
f = struct('escenario', esc, 'ley', ley, 'rep', k, 'completo', double(M.completo), ...
    't_final', M.t_final, 'rmse_ex', M.rmse_ex, 'rmse_evl', M.rmse_evl, ...
    'rms_alpha', M.rms_alpha, 'max_alpha', M.max_alpha, 'rms_rho', M.rms_rho, 'max_rho', M.max_rho, ...
    'rms_vperp', M.rms_vperp, 'max_vperp', M.max_vperp, 'rms_swing', M.rms_swing, ...
    'max_swing', M.max_swing, 'ish', M.ish, 'ish_dev', M.ish_dev, ...
    'rms_du', norm(M.rms_du), 'tv_u', sum(M.tv_u), 'frac_sat', M.frac_sat, ...
    'dt_medio', M.dt_medio, 'dt_max', M.dt_max, 'reintentos', campo_o(M, 'reintentos', 0));
end

function D = metricas_tabla()
% key, LaTeX label, text label
D = { 'rmse_ex',   'RMSE $\tilde{\mathbf{x}}$ [m]',                    'RMSE pos. carga [m]';
      'rmse_evl',  'RMSE $\dot{\tilde{\mathbf{x}}}$ [m/s]',           'RMSE vel. carga [m/s]';
      'rms_alpha', 'RMS $\alpha$ [rad]',                               'RMS alpha [rad]';
      'max_alpha', 'Max $\alpha$ [rad]',                               'Max alpha [rad]';
      'rms_rho',   'RMS $\rho$ [m]',                                   'RMS rho [m]';
      'max_rho',   'Max $\rho$ [m]',                                   'Max rho [m]';
      'rms_vperp', 'RMS $\|\mathbf{v}_\perp\|$ [m/s]',                 'RMS |v_perp| [m/s]';
      'max_vperp', 'Max $\|\mathbf{v}_\perp\|$ [m/s]',                 'Max |v_perp| [m/s]';
      'rms_swing', 'RMS $\|\mathbf{h}_h-\mathbf{h}^\star_h\|$ [m]',     'RMS |h_h - h*_h| [m]';
      'ish',       'ISH, Eq.~(4) of Omar [rad$^2$s]',                   'ISH Omar Ec.(4) [rad2 s]';
      'ish_dev',   'ISH about $\mathbf{h}^\star_h$ [rad$^2$s]',         'ISH respecto de h* [rad2 s]';
      'rms_du',    'RMS $\|\dot{\mathbf{u}}_h\|$ [m/s$^2$]',            'RMS |du_h/dt| [m/s2]' };
end

function T = tabla(filas, op, NOM)
D = metricas_tabla();
T = struct('esc', {op.escenarios}, 'leyes', {op.leyes}, 'D', {D}, 'NOM', NOM);
for e = 1:numel(op.escenarios)
    G = filas(cellfun(@(f) strcmp(f.escenario, op.escenarios{e}), filas));
    for j = 1:numel(op.leyes)
        Gj = G(cellfun(@(f) strcmp(f.ley, op.leyes{j}), G));
        T.n(e, j) = numel(Gj);
        T.nc(e, j) = sum(cellfun(@(f) f.completo, Gj));
        T.nr(e, j) = sum(cellfun(@(f) f.reintentos, Gj));
        Gc = Gj(cellfun(@(f) f.completo == 1, Gj));     % metrics from complete runs only
        for i = 1:size(D, 1)
            v = cellfun(@(f) f.(D{i,1}), Gc);
            v = v(isfinite(v));
            T.val{e, j, i} = v;
            T.media(e, j, i) = mean_v(v);
            T.desv(e, j, i) = std_v(v);
        end
    end
    jo = find(strcmp(op.leyes, 'omar')); ja = find(strcmp(op.leyes, 'asta'));
    for i = 1:size(D, 1)
        if isempty(jo) || isempty(ja), T.delta(e,i) = nan; T.p(e,i) = nan; continue; end
        mo = T.media(e, jo, i); ma = T.media(e, ja, i);
        T.delta(e, i) = 100 * (mo - ma) / mo;          % > 0: the proposed controller has the lower metric
        T.p(e, i) = welch_p(T.val{e, jo, i}, T.val{e, ja, i});
    end
end
end

function p = welch_p(a, b)
% two-sided p of Welch's t-test (without the Statistics Toolbox).
na = numel(a); nb = numel(b);
if na < 2 || nb < 2, p = nan; return; end
va = var(a)/na; vb = var(b)/nb;
if va + vb == 0, p = nan; return; end              % identical runs: the test does not apply
t = (mean(a) - mean(b)) / sqrt(va + vb);
df = (va + vb)^2 / (va^2/(na-1) + vb^2/(nb-1));
p = betainc(df / (df + t^2), df/2, 0.5);
end

function escribir_tex(T, archivo, op)
fid = fopen(archivo, 'w');
fprintf(fid, '%% Generado por comparar_omar_vs_asta.m (%s, backend %s).\n', datestr(now, 'yyyy-mm-dd HH:MM'), op.backend);
fprintf(fid, '%% Media $\\pm$ desvio sobre repeticiones, t >= %g s. Delta = (Omar - propuesto)/Omar.\n', op.t_inicio);
fprintf(fid, '%% p: prueba t de Welch bilateral, Omar vs propuesto.\n');
nl = numel(T.leyes);
for e = 1:numel(T.esc)
    fprintf(fid, '\n%% ---- %s ----\n', T.esc{e});
    fprintf(fid, '\\begin{tabular}{l%scc}\n\\toprule\n', repmat('c', 1, nl));
    fprintf(fid, 'Metric');
    for j = 1:nl, fprintf(fid, ' & %s', T.NOM.(T.leyes{j})); end
    fprintf(fid, ' & $\\Delta$ [\\%%] & $p$ \\\\\n\\midrule\n');
    fprintf(fid, 'Completed runs');
    for j = 1:nl, fprintf(fid, ' & %d/%d', T.nc(e,j), T.n(e,j)); end
    fprintf(fid, ' & & \\\\\n');
    if any(T.nr(e,:) > 0)
        fprintf(fid, '%% Reinicios del simulador (corridas repetidas por SimQuad detenido):');
        fprintf(fid, ' %d', T.nr(e,:)); fprintf(fid, '\n');
    end
    for i = 1:size(T.D, 1)
        fprintf(fid, '%s', T.D{i,2});
        for j = 1:nl
            fprintf(fid, ' & %s', fmt_tex(T.media(e,j,i), T.desv(e,j,i)));
        end
        fprintf(fid, ' & %s & %s \\\\\n', fmt_num(T.delta(e,i), '%+.1f'), fmt_num(T.p(e,i), '%.3f'));
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end
fclose(fid);
end

function escribir_txt(T, archivo, op)
fid = fopen(archivo, 'w');
for dest = [1 fid]
    fprintf(dest, '\nOmar (2022) vs propuesto. Media +- desvio, t >= %g s. Delta > 0: el propuesto es menor.\n', op.t_inicio);
    for e = 1:numel(T.esc)
        fprintf(dest, '\n== %s ==\n%-34s', T.esc{e}, 'Metrica');
        for j = 1:numel(T.leyes), fprintf(dest, '  %-20s', T.leyes{j}); end
        fprintf(dest, '  %8s  %6s\n', 'Delta%', 'p');
        fprintf(dest, '%-34s', 'corridas completas');
        for j = 1:numel(T.leyes), fprintf(dest, '  %-20s', sprintf('%d/%d', T.nc(e,j), T.n(e,j))); end
        fprintf(dest, '\n%-34s', 'reinicios de SimQuad');
        for j = 1:numel(T.leyes), fprintf(dest, '  %-20d', T.nr(e,j)); end
        fprintf(dest, '\n');
        for i = 1:size(T.D, 1)
            fprintf(dest, '%-34s', T.D{i,3});
            for j = 1:numel(T.leyes)
                fprintf(dest, '  %-20s', sprintf('%.4f +- %.4f', T.media(e,j,i), T.desv(e,j,i)));
            end
            fprintf(dest, '  %8s  %6s\n', fmt_num(T.delta(e,i), '%+.1f'), fmt_num(T.p(e,i), '%.3f'));
        end
    end
end
fclose(fid);
end

function escribir_parametros(archivo, cfg)
P = cfg.omar;
fid = fopen(archivo, 'w');
fprintf(fid, 'Baseline Omar (2022), Appl. Sci. 12(3):1706\n');
fprintf(fid, 'Origen de K y tau: %s\n', P.origen);
fprintf(fid, 'K_sx = K_sy = %.3f\n', P.K(1));
fprintf(fid, 'T_L = 2*pi/sqrt(g/L) = %.4f s (L = %.2f m)\n', P.TL, cfg.planta.l);
fprintf(fid, 't_sx = t_sy = %.3f T_L = %.4f s\n', P.tau_TL, P.tau);
fprintf(fid, 'Lazo de posicion tipo PX4: Kp = diag(%.2f, %.2f, %.2f)\n', diag(P.Kp));
fprintf(fid, 'Signo de la correccion: %+d (convencion en controlador_omar.m)\n', P.signo);
fprintf(fid, 'Comunes con el propuesto: rampa sigma(t), Ts = %.1f s; ley de yaw; u_max = %.1f m/s\n', ...
    cfg.ext.Ts, cfg.int.u_max);
fclose(fid);
end

%% ---- figures ------------------------------------------------------------
function s = serie_fig(R, M)
s.t = R.t;
s.alpha = M.serie.alpha;
s.ex = sqrt(sum((R.xd - R.xl).^2, 1));
s.vperp = M.serie.vperp;
s.viento = R.viento;
s.completo = R.completo;
end

function figuras(SE, op, esc, NOM, carpeta)
C.px4  = struct('c', [0.45 0.45 0.45], 'ls', '--');
C.omar = struct('c', [0xb8 0x44 0x8f]/255, 'ls', '-');
C.asta = struct('c', [0x2a 0x78 0xd6]/255, 'ls', '-');
var = {'alpha', '$\alpha$ [rad]'; 'ex', '$\|\tilde{\mathbf{x}}\|$ [m]'; 'vperp', '$\|\mathbf{v}_\perp\|$ [m/s]'};
% complete runs only: a cut run shortens the axis and does not represent the controller
for j = 1:numel(op.leyes)
    l = op.leyes{j};
    if isfield(SE, l)
        ok = cellfun(@(s) ~isempty(s) && s.completo, SE.(l));
        SE.(l) = SE.(l)(ok);
        if isempty(SE.(l)), SE = rmfield(SE, l); end
    end
end
leyes = op.leyes(isfield_all(SE, op.leyes));
if isempty(leyes), return; end
Tfin = inf;
for j = 1:numel(leyes)
    for k = 1:numel(SE.(leyes{j})), Tfin = min(Tfin, SE.(leyes{j}){k}.t(end)); end
end
tg = linspace(0, Tfin, max(2, round(Tfin/0.02)));
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', 'Position', [2 2 16 13]);
for v = 1:size(var, 1)
    subplot(size(var,1), 1, v); hold on; box on; grid on;
    B = SE.(leyes{1}){1}.viento;
    if any(B(:))
        on = any(B ~= 0, 1); tt = SE.(leyes{1}){1}.t;
        d = diff([0 on 0]); i0 = find(d == 1); i1 = find(d == -1) - 1;
        for q = 1:numel(i0)
            patch([tt(i0(q)) tt(i1(q)) tt(i1(q)) tt(i0(q))], [0 0 1 1]*1e3, [0.93 0.93 0.93], ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end
    hh = gobjects_n(numel(leyes)); ymax = 0;
    for j = 1:numel(leyes)
        Y = [];
        for k = 1:numel(SE.(leyes{j}))
            s = SE.(leyes{j}){k};
            [tu, iu] = unique(s.t);
            Y(k, :) = interp1(tu, s.(var{v,1})(iu), tg, 'linear', NaN); %#ok<AGROW>
        end
        ym = mean(Y, 1, 'omitnan');
        hh(j) = plot(tg, ym, C.(leyes{j}).ls, 'Color', C.(leyes{j}).c, 'LineWidth', 1.0);
        yv = ym(tg >= op.t_inicio & isfinite(ym));
        if ~isempty(yv), ymax = max(ymax, prctile_99(yv)); end
    end
    if isfinite(ymax) && ymax > 0, ylim([0 1.1*ymax]); end
    xlim([0 Tfin]);
    ylabel(var{v,2}, 'Interpreter', 'latex');
    if v == size(var,1), xlabel('Time [s]'); end
    if v == 1
        legend(hh, cellfun(@(l) NOM.(l), leyes, 'UniformOutput', false), ...
            'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    end
end
base = fullfile(carpeta, [esc '_omar_vs_asta']);
print(f, [base '.png'], '-dpng', '-r200');
try, print(f, [base '.pdf'], '-dpdf'); catch, end %#ok<NOCOM>
close(f);
end

function y = prctile_99(v)
% 99th percentile without the Statistics Toolbox: an isolated spike does not set the axis
v = sort(v(:)); y = v(max(1, ceil(0.99*numel(v))));
end

function ok = isfield_all(S, c)
ok = cellfun(@(x) isfield(S, x), c);
end

function h = gobjects_n(n)
if exist('gobjects', 'builtin') || exist('gobjects', 'file'), h = gobjects(1, n); else, h = zeros(1, n); end
end

%% ---- utilities ----------------------------------------------------------
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

function s = fmt_tex(m, d)
if ~isfinite(m), s = '--'; elseif ~isfinite(d) || d == 0, s = sprintf('$%.4f$', m);
else, s = sprintf('$%.4f \\pm %.4f$', m, d); end
end

function s = fmt_num(x, f)
if isfinite(x), s = sprintf(f, x); else, s = '--'; end
end

function s = limpiar(s)
s = regexprep(s, '\$|\\mathbf|\\tilde|\\dot|\\|\{|\}|~|\^', '');
end

function x = campo_o(S, c, def)
if isfield(S, c), x = S.(c); else, x = def; end
end

function m = mean_v(v), if isempty(v), m = nan; else, m = mean(v); end, end
function s = std_v(v), if numel(v) < 2, s = nan; else, s = std(v); end, end
function c = campo(esc), c = regexprep(esc, '[^A-Za-z0-9_]', '_'); end

function semilla(s)
if exist('OCTAVE_VERSION', 'builtin'), rand('seed', s); randn('seed', s); else, rng(s); end %#ok<RAND>
end

function mkdir_p(d), if ~exist(d, 'dir'), mkdir(d); end, end
