function A = ajustar_omar(varargin)
% AJUSTAR_OMAR  Tuning of the Omar (2022) baseline with its own criterion (minimum ISH).
%
%   ajustar_omar()                                  % SimQuad1.exe (~65 min in real time)
%   ajustar_omar('backend','sustituta')             % code test (not for the paper)
%   ajustar_omar('K',[0.3 0.6 1.0],'tau_TL',0:0.1:0.5,'signo',[1 -1])
%
% The paper (Sec. 3.2) chooses the gains and the delay of Eqs. (1)-(2) to
% minimize ISH = int(phi_L^2 + theta_L^2) dt (Eq. 4), and it only uses T_L (Eq. 3).
% It does not fix the sign of its angles and it does not give a readable delay.
% That is why all three are searched: sign, K, and tau/T_L.
%
% Step 1 (diagnostic, hover). UAV at rest, a 2 s gust in x, ISH after the
%   gust, K = 0.6, sweep of tau for both signs and the case without ASC. It shows
%   the damping region. It does not choose anything.
% Step 2 (tuning). Grid sign x K x tau on the paper's "travel flight" maneuver
%   (Sec. 5.2): a 3 m step in x, then 3 m in y, and back to the start. The winner
%   is the minimum ISH among complete runs. On SimQuad the result is saved in
%   omar_ajuste.mat, which parametros_omar loads.
%   It is not tuned on the circle: there the ISH of Eq. (4) is dominated by the
%   quasi-static cable tilt (centripetal acceleration), not by the oscillation, and
%   the minimum falls on gains that destabilize the pendulum in hover (test on the
%   surrogate plant). Tuning on the paper's own maneuver avoids that bias.
% Step 3 (verification). The optimum and the tracker without ASC on the E4 circle:
%   absolute ISH and ISH relative to h*_h. Reported only.
%
% The chosen parameters are used unchanged in every scenario, just as the gains
% of the proposed controller are.
%
% Resume. After each run the result is saved in omar_ajuste_parcial.mat. If MATLAB
% closes, call ajustar_omar() again with the same arguments: finished runs are read
% from that file and the search continues from the next one. Only the run that was
% in progress is lost. When it finishes, the partial file is deleted and
% omar_ajuste.mat remains. ajustar_omar('reanudar', false) discards the partial
% file and starts from scratch.

op = struct('backend', 'simquad', 'escenario', 'E4_circ_asta', 'T', 60, ...
    'signo', [1 -1], 'K', [0.3 0.6 0.9 1.2 1.5], 'tau_TL', [0 0.05 0.1 0.15 0.2 0.3], ...
    't_inicio', 5, 'hover', true, 'verificar', true, 'guardar', true, 'reanudar', true);
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
sim_dir = fullfile(aqui, '..', 'simulations');
if exist(sim_dir, 'dir'), addpath(sim_dir); end
arch = fullfile(aqui, 'omar_ajuste.mat');
CK.archivo = fullfile(aqui, 'omar_ajuste_parcial.mat');
CK.firma = firma(op);
CK.cache = struct();
if op.reanudar && exist(CK.archivo, 'file')
    try, S = load(CK.archivo); catch, S = struct(); end
    if isfield(S, 'firma') && strcmp(S.firma, CK.firma)
        CK.cache = S.cache;
        fprintf('Reanudando: %d corridas ya hechas en %s\n', numel(fieldnames(CK.cache)), CK.archivo);
    else
        fprintf('El parcial existente es de otra grilla u otro backend: se empieza de cero.\n');
    end
end
A = struct();

%% Step 1: hover diagnostic
if op.hover
    cfg = base_cfg(op, 'E4_circ_asta');
    cfg.ref.circ.R = 0; cfg.ref.circ.vz = 0;              % fixed point
    cfg.planta.T = 30;
    cfg.viento.bloques = [5 7 4 0 0];                     % 4 m/s gust in x, 2 s
    fprintf('\nPaso 1: hover, rafaga de 2 s, ISH para t >= 7 s, K = 0.6\n');
    cfg.omar = parametros_omar(cfg, 'activo', false, 'archivo', 'ninguno');
    [r, CK] = correr(CK, 'hover_sin_asc', cfg, 7);
    H.sin_asc = r.ish;
    fprintf('  sin ASC                 ISH = %.4f\n', H.sin_asc);
    H.ish = nan(numel(op.signo), numel(op.tau_TL));
    for a = 1:numel(op.signo)
        for b = 1:numel(op.tau_TL)
            cfg.omar = parametros_omar(cfg, 'K', 0.6, 'tau_TL', op.tau_TL(b), ...
                'signo', op.signo(a), 'archivo', 'ninguno');
            [r, CK] = correr(CK, clave('hover', op.signo(a), 0.6, op.tau_TL(b)), cfg, 7);
            H.ish(a, b) = r.ish;
            fprintf('  signo %+d  tau = %.2f T_L  ISH = %.4f  (%+.0f %% vs sin ASC)\n', ...
                op.signo(a), op.tau_TL(b), H.ish(a,b), 100*(H.ish(a,b)/H.sin_asc - 1));
        end
    end
    H.signo = op.signo; H.tau_TL = op.tau_TL;
    A.hover = H;
end

%% Step 2: sign x K x tau grid by minimum ISH (Eq. 4), step maneuver
ns = numel(op.signo); nK = numel(op.K); nT = numel(op.tau_TL);
ISH = nan(ns, nK, nT); ISHd = ISH; RMSE = ISH; OK = false(ns, nK, nT);
fprintf('\nPaso 2: escalones de 3 m en x y en y (Omar, Secc. 5.2), 55 s, ISH de toda la corrida\n');
for a = 1:ns
    for i = 1:nK
        for j = 1:nT
            cfg = cfg_pasos(op);
            cfg.omar = parametros_omar(cfg, 'K', op.K(i), 'tau_TL', op.tau_TL(j), ...
                'signo', op.signo(a), 'archivo', 'ninguno');
            [r, CK] = correr(CK, clave('grilla', op.signo(a), op.K(i), op.tau_TL(j)), cfg, 0);
            ISH(a,i,j) = r.ish; ISHd(a,i,j) = r.ish_dev;
            OK(a,i,j) = r.completo; RMSE(a,i,j) = r.rmse_x;
            fprintf('  signo %+d  K = %.2f  tau = %.2f T_L  ISH = %8.4f  RMSE x = %.3f  %s%s\n', ...
                op.signo(a), op.K(i), op.tau_TL(j), ISH(a,i,j), RMSE(a,i,j), r.motivo, r.nota);
        end
    end
end
J = ISH; J(~OK) = inf;
[~, idx] = min(J(:));
[ia, ib, jb] = ind2sub(size(J), idx);

% reference: tracker without ASC on the same maneuver
cfg = cfg_pasos(op);
cfg.omar = parametros_omar(cfg, 'activo', false, 'archivo', 'ninguno');
[r, CK] = correr(CK, 'grilla_sin_asc', cfg, 0);
ish0 = r.ish;

A.grilla = struct('signo', op.signo, 'K', op.K, 'tau_TL', op.tau_TL, 'ISH', ISH, ...
    'RMSE_x', RMSE, 'completo', OK, 'ISH_sin_asc', ish0, ...
    'maniobra', 'escalones 3 m x, 3 m y, vuelta (Omar 2022, Secc. 5.2)', 'backend', op.backend);
A.signo = op.signo(ia); A.K = op.K(ib); A.tau_TL = op.tau_TL(jb);
A.fecha = datestr(now, 'yyyy-mm-dd HH:MM');
TL = 2*pi/sqrt(cfg.planta.g/cfg.planta.l);
fprintf('\nOptimo: signo %+d, K = %.2f, tau = %.2f T_L (%.3f s). ISH = %.4f (sin ASC: %.4f, %+.0f %%)\n', ...
    A.signo, A.K, A.tau_TL, A.tau_TL*TL, ISH(ia,ib,jb), ish0, 100*(ISH(ia,ib,jb)/ish0 - 1));
borde_K = nK > 1 && (ib == 1 || ib == nK);
borde_T = nT > 1 && (jb == nT || (jb == 1 && op.tau_TL(1) > 0));
if borde_K || borde_T
    fprintf('Aviso: el optimo esta en el borde de la grilla. Ampliarla y repetir.\n');
elseif nT > 1 && op.tau_TL(jb) == 0
    fprintf(['Nota: el optimo es tau = 0, el limite fisico (no hay retardo negativo). ', ...
        'El desfase lo aporta el lazo de posicion del UAV.\n']);
end

%% Step 3: verification on the circle
if op.verificar
    fprintf('\nPaso 3: %s, T = %g s, t >= %g s\n', op.escenario, op.T, op.t_inicio);
    V = struct();
    nom = {'sin_asc', 'optimo'};
    for c = 1:2
        cfg = base_cfg(op, op.escenario);
        if c == 1
            cfg.omar = parametros_omar(cfg, 'activo', false, 'archivo', 'ninguno');
        else
            cfg.omar = parametros_omar(cfg, 'K', A.K, 'tau_TL', A.tau_TL, 'signo', A.signo, 'archivo', 'ninguno');
        end
        if c == 1, cl = 'verif_sin_asc'; else, cl = clave('verif', A.signo, A.K, A.tau_TL); end
        [r, CK] = correr(CK, cl, cfg, op.t_inicio);
        V.(nom{c}) = rmfield(r, 'nota');
        fprintf('  %-8s ISH = %.4f  ISH_dev = %.4f  RMSE x = %.3f  %s%s\n', nom{c}, ...
            r.ish, r.ish_dev, r.rmse_x, r.motivo, r.nota);
    end
    A.verificacion = V;
end

if op.guardar && strcmp(op.backend, 'simquad')
    if exist(arch, 'file')                              % do not overwrite a previous tuning
        S0 = load(arch);
        bak = fullfile(aqui, ['omar_ajuste_' regexprep(S0.ajuste.fecha, '[^0-9]', '') '.mat']);
        movefile(arch, bak, 'f');
        fprintf('Ajuste anterior guardado como %s\n', bak);
    end
    ajuste = A; %#ok<NASGU>
    save(arch, 'ajuste');
    fprintf('Guardado en %s\n', arch);
elseif op.guardar
    fprintf('Backend %s: no se guarda omar_ajuste.mat (el ajuste del paper sale de SimQuad).\n', op.backend);
end
if exist(CK.archivo, 'file'), delete(CK.archivo); end   % tuning finished: the partial file is no longer needed
end

%% ------------------------------------------------------------------------
function [r, CK] = correr(CK, cl, cfg, t_ish)
% One run, or its saved result if it already finished before MATLAB closed.
if isfield(CK.cache, cl)
    r = CK.cache.(cl); r.nota = '  (guardada)';
    return;
end
[R, r.reintentos] = sim_con_reintento(@controlador_omar, cfg, 2, '');
[r.ish, r.ish_dev] = ish_omar(R, t_ish);
sel = R.t >= t_ish;
r.rmse_x = sqrt(mean(sum((R.xd(:,sel) - R.xl(:,sel)).^2, 1)));
r.completo = R.completo;
r.motivo = R.motivo;
r.t_final = R.t(end);
CK.cache.(cl) = r;
cache = CK.cache; firma = CK.firma; %#ok<NASGU>
tmp = [CK.archivo '.tmp.mat'];                          % checkpoint after each run;
save(tmp, 'cache', 'firma');                            % temp file + rename: a close mid-save
movefile(tmp, CK.archivo, 'f');                         % does not leave a broken partial file
r.nota = '';
end

function c = clave(paso, sg, K, tau)
if sg > 0, s = 'p'; else, s = 'm'; end
c = sprintf('%s_s%s_K%03d_t%03d', paso, s, round(100*K), round(100*tau));
end

function f = firma(op)
% Identifies the grid: a partial file is reused only with the same arguments.
f = sprintf('%s|%s|%g|%s|%s|%s|%g|%d|%d', op.backend, op.escenario, op.T, ...
    mat2str(op.signo), mat2str(op.K), mat2str(op.tau_TL), op.t_inicio, op.hover, op.verificar);
end

%% ------------------------------------------------------------------------
function cfg = cfg_pasos(op)
cfg = base_cfg(op, 'E4_circ_asta');
cfg.planta.T = 55;
cfg.ref.tipo = 'pasos';
cfg.ref.pasos = struct('t', [0 10 25 40], 'p', [0 0 6; 3 0 6; 3 3 6; 0 0 6].');
end

function cfg = base_cfg(op, esc)
cfg = cfg_paper(esc);
cfg.planta.backend = op.backend;
if strcmp(op.backend, 'sustituta'), cfg.planta.usar_dt_real = false; end
cfg.planta.T = op.T;
end
