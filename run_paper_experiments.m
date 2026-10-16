function run_paper_experiments(varargin)
% RUN_PAPER_EXPERIMENTS  Regenerates every run, table, and figure of the paper.
%
%   run_paper_experiments()                         % SimQuad1.exe, 5 repetitions
%   run_paper_experiments('backend','sustituta')    % test without the simulator (not for the paper)
%   run_paper_experiments('escenarios',{'E4_circ_asta'}, 'n_rep', 1)
%
% For each scenario and repetition it saves results/<fecha>/raw/<escenario>_rep<k>.mat with the
% full time series (R) and the configuration used (R.cfg). It then writes:
%   resumen.csv            every metric per run
%   tabla_metricas.tex     mean +- std per scenario (Table metrics, ablation, mass)
%   tabla_parametros.tex   parameters actually used (from cfg_paper)
%   figuras/               control figures of each scenario (repetition 1)
%   paper_figs/            manuscript figures (figuras_paper.m), EPS and PDF
%
% Run with paper_revision as the current folder; it adds ../simulations to the path.

op = struct('backend', 'simquad', 'n_rep', [], 'escenarios', {{}}, 'salida', '');
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'simulations'));

if isempty(op.escenarios)
    op.escenarios = {'E1_circ_sin_swing', 'E2_circ_cinematico', 'E3_circ_sta_fijo', ...
        'E4_circ_asta', 'E5_lemn_viento', 'E6_circ_smc'};
    for mg = [50 100 200 300]
        op.escenarios{end+1} = sprintf('E7_masa_%d_fijo', mg);
        op.escenarios{end+1} = sprintf('E7_masa_%d_asta', mg);
    end
end
if isempty(op.salida)
    op.salida = fullfile(aqui, 'results', datestr(now, 'yyyymmdd_HHMMSS'));
end
mkdir_p(fullfile(op.salida, 'raw'));
mkdir_p(fullfile(op.salida, 'figuras'));

test_referencias();

filas = {};
for e = 1:numel(op.escenarios)
    esc = op.escenarios{e};
    cfg = cfg_paper(esc);
    cfg.planta.backend = op.backend;
    if strcmp(op.backend, 'sustituta'), cfg.planta.usar_dt_real = false; end
    if ~isempty(op.n_rep), cfg.exp.n_rep = op.n_rep; end
    for k = 1:cfg.exp.n_rep
        semilla(cfg.exp.semilla + k);
        fprintf('[%s] %s, repeticion %d/%d ... ', datestr(now,'HH:MM:SS'), esc, k, cfg.exp.n_rep);
        R = sim_payload_asta(cfg);
        M = metricas_payload(R);
        fprintf('%s, t = %.1f s, RMSE x = %.3f m\n', R.motivo, M.t_final, M.rmse_ex);
        archivo = fullfile(op.salida, 'raw', sprintf('%s_rep%d.mat', esc, k));
        save(archivo, 'R', 'M', '-v7');
        filas{end+1} = fila_resumen(esc, k, M); %#ok<AGROW>
        if k == 1, figuras_escenario(R, M, fullfile(op.salida, 'figuras', esc)); end
    end
end

escribir_csv(fullfile(op.salida, 'resumen.csv'), filas);
tabla_metricas_latex(filas, fullfile(op.salida, 'tabla_metricas.tex'));
tabla_masa_latex(filas, fullfile(op.salida, 'tabla_masa.tex'));
tabla_parametros_latex(cfg_paper(), fullfile(op.salida, 'tabla_parametros.tex'));
figuras_paper(op.salida);          % manuscript figures in <salida>/paper_figs
fprintf('\nResultados en %s\n', op.salida);
end

%% ------------------------------------------------------------------------
function f = fila_resumen(esc, k, M)
f = struct('escenario', esc, 'rep', k, 'completo', double(M.completo), 't_final', M.t_final, ...
    'rmse_ex', M.rmse_ex, 'rmse_evl', M.rmse_evl, 'rms_alpha', M.rms_alpha, 'max_alpha', M.max_alpha, ...
    'rms_rho', M.rms_rho, 'max_rho', M.max_rho, 'rms_vperp', M.rms_vperp, 'max_vperp', M.max_vperp, ...
    'rms_swing', M.rms_swing, 'max_swing', M.max_swing, ...
    'est_rms_err', M.est_rms_err, 'est_retardo', M.est_retardo, 'est_ganancia', M.est_ganancia, ...
    'rms_vt_x', M.rms_vt(1), 'rms_vt_y', M.rms_vt(2), 'rms_nu_x', M.rms_nu(1), 'rms_nu_y', M.rms_nu(2), ...
    'rms_du_x', M.rms_du(1), 'rms_du_y', M.rms_du(2), 'tv_u_x', M.tv_u(1), 'tv_u_y', M.tv_u(2), ...
    'frac_sat', M.frac_sat, 'th1_x', M.theta_fin(1), 'th2_x', M.theta_fin(2), ...
    'th1_y', M.theta_fin(3), 'th2_y', M.theta_fin(4), 'dt_medio', M.dt_medio, 'dt_max', M.dt_max);
end

function escribir_csv(archivo, filas)
fid = fopen(archivo, 'w');
c = fieldnames(filas{1});
fprintf(fid, '%s', c{1}); fprintf(fid, ',%s', c{2:end}); fprintf(fid, '\n');
for i = 1:numel(filas)
    fprintf(fid, '%s', filas{i}.(c{1}));
    for j = 2:numel(c), fprintf(fid, ',%.6g', filas{i}.(c{j})); end
    fprintf(fid, '\n');
end
fclose(fid);
end

function tabla_metricas_latex(filas, archivo)
cols = {'rmse_ex','rmse_evl','rms_alpha','max_alpha','rms_rho','max_rho','rms_vperp','max_vperp','rms_swing'};
tit  = {'RMSE $\tilde{\mathbf{x}}$','RMSE $\dot{\tilde{\mathbf{x}}}$','RMS$_\alpha$','Max $\alpha$', ...
        'RMS$_\rho$','Max $\rho$','RMS$_{v_\perp}$','Max $v_\perp$','RMS $\|\tilde{\mathbf{h}}_h\|$'};
esc = unique(cellfun(@(f) f.escenario, filas, 'UniformOutput', false), 'stable');
fid = fopen(archivo, 'w');
fprintf(fid, '%% Generado por run_paper_experiments.m. Media $\\pm$ desvio sobre repeticiones; n = corridas completas/total.\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\nEscenario & n', repmat('c', 1, numel(cols)+1));
fprintf(fid, ' & %s', tit{:}); fprintf(fid, ' \\\\\n\\midrule\n');
for i = 1:numel(esc)
    F = filas(cellfun(@(f) strcmp(f.escenario, esc{i}), filas));
    nc = sum(cellfun(@(f) f.completo, F));
    fprintf(fid, '%s & %d/%d', strrep(esc{i}, '_', '\_'), nc, numel(F));
    for j = 1:numel(cols)
        v = cellfun(@(f) f.(cols{j}), F);
        if numel(v) > 1, fprintf(fid, ' & $%.4f \\pm %.4f$', mean(v), std(v));
        else, fprintf(fid, ' & $%.4f$', v); end
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);
end

function tabla_masa_latex(filas, archivo)
% Mass sweep (E7): final parameters and RMS of the integral state nu, fixed vs adaptive.
F = filas(cellfun(@(f) strncmp(f.escenario, 'E7_', 3), filas));
if isempty(F), return; end
esc = unique(cellfun(@(f) f.escenario, F, 'UniformOutput', false), 'stable');
cols = {'rmse_ex','rms_vt_x','rms_nu_x','rms_nu_y','th1_x','th2_x','th1_y','th2_y'};
tit  = {'RMSE $\tilde{\mathbf{x}}$','RMS $\tilde v_x$','RMS $\nu_x$','RMS $\nu_y$', ...
        '$\hat\theta_{1,x}$','$\hat\theta_{2,x}$','$\hat\theta_{1,y}$','$\hat\theta_{2,y}$'};
fid = fopen(archivo, 'w');
fprintf(fid, '%% Generado por run_paper_experiments.m. Valores finales de theta; media $\\pm$ desvio.\n');
fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\nEscenario', repmat('c', 1, numel(cols)));
fprintf(fid, ' & %s', tit{:}); fprintf(fid, ' \\\\\n\\midrule\n');
for i = 1:numel(esc)
    G = F(cellfun(@(f) strcmp(f.escenario, esc{i}), F));
    fprintf(fid, '%s', strrep(esc{i}, '_', '\_'));
    for j = 1:numel(cols)
        v = cellfun(@(f) f.(cols{j}), G);
        if numel(v) > 1, fprintf(fid, ' & $%.3f \\pm %.3f$', mean(v), std(v));
        else, fprintf(fid, ' & $%.3f$', v); end
    end
    fprintf(fid, ' \\\\\n');
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);
end

function semilla(s)
if exist('OCTAVE_VERSION', 'builtin'), rand('seed', s); randn('seed', s); else, rng(s); end %#ok<RAND>
end

function mkdir_p(d)
if ~exist(d, 'dir'), mkdir(d); end
end
