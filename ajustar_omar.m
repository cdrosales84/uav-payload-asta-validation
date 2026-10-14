function A = ajustar_omar(varargin)
% AJUSTAR_OMAR  Ajuste del baseline de Omar (2022) con su propio criterio (minimo ISH).
%
%   ajustar_omar()                                  % SimQuad1.exe (~65 min en tiempo real)
%   ajustar_omar('backend','sustituta')             % prueba del codigo (no para el paper)
%   ajustar_omar('K',[0.3 0.6 1.0],'tau_TL',0:0.1:0.5,'signo',[1 -1])
%
% El articulo (Secc. 3.2) elige las ganancias y el retardo de las Ecs. (1)-(2) para
% minimizar ISH = int(phi_L^2 + theta_L^2) dt (Ec. 4), y solo usa T_L (Ec. 3). No
% fija el signo de sus angulos ni da un valor legible del retardo. Por eso se buscan
% los tres: signo, K y tau/T_L.
%
% Paso 1 (diagnostico, hover). UAV quieto, rafaga de 2 s en x, ISH despues de la
%   rafaga, K = 0.6, barrido de tau para los dos signos y caso sin ASC. Muestra la
%   region de amortiguamiento. No elige nada.
% Paso 2 (ajuste). Grilla signo x K x tau en la maniobra de "travel flight" del
%   articulo (Secc. 5.2): escalon de 3 m en x, despues 3 m en y, y vuelta al punto
%   de partida. Gana el minimo ISH entre las corridas completas. En SimQuad el
%   resultado se guarda en omar_ajuste.mat, que parametros_omar carga.
%   No se ajusta en el circulo: ahi el ISH de la Ec. (4) lo domina la inclinacion
%   cuasiestatica del cable (aceleracion centripeta), no la oscilacion, y el minimo
%   cae en ganancias que en hover desestabilizan el pendulo (prueba con la planta
%   sustituta). Ajustar en la maniobra del propio articulo evita ese sesgo.
% Paso 3 (verificacion). El optimo y el seguidor sin ASC en el circulo de E4:
%   ISH absoluto y respecto de h*_h. Solo se informa.
%
% Los parametros elegidos se usan sin cambios en todos los escenarios, igual que
% las ganancias del controlador propuesto.

op = struct('backend', 'simquad', 'escenario', 'E4_circ_asta', 'T', 60, ...
    'signo', [1 -1], 'K', [0.3 0.6 0.9 1.2 1.5], 'tau_TL', [0 0.05 0.1 0.15 0.2 0.3], ...
    't_inicio', 5, 'hover', true, 'verificar', true, 'guardar', true);
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
sim_dir = fullfile(aqui, '..', 'simulations');
if exist(sim_dir, 'dir'), addpath(sim_dir); end
arch = fullfile(aqui, 'omar_ajuste.mat');
A = struct();

%% Paso 1: diagnostico en hover
if op.hover
    cfg = base_cfg(op, 'E4_circ_asta');
    cfg.ref.circ.R = 0; cfg.ref.circ.vz = 0;              % punto fijo
    cfg.planta.T = 30;
    cfg.viento.bloques = [5 7 4 0 0];                     % rafaga de 4 m/s en x, 2 s
    fprintf('\nPaso 1: hover, rafaga de 2 s, ISH para t >= 7 s, K = 0.6\n');
    cfg.omar = parametros_omar(cfg, 'activo', false, 'archivo', 'ninguno');
    R = sim_controlador(@controlador_omar, cfg);
    H.sin_asc = ish_omar(R, 7);
    fprintf('  sin ASC                 ISH = %.4f\n', H.sin_asc);
    H.ish = nan(numel(op.signo), numel(op.tau_TL));
    for a = 1:numel(op.signo)
        for b = 1:numel(op.tau_TL)
            cfg.omar = parametros_omar(cfg, 'K', 0.6, 'tau_TL', op.tau_TL(b), ...
                'signo', op.signo(a), 'archivo', 'ninguno');
            R = sim_controlador(@controlador_omar, cfg);
            H.ish(a, b) = ish_omar(R, 7);
            fprintf('  signo %+d  tau = %.2f T_L  ISH = %.4f  (%+.0f %% vs sin ASC)\n', ...
                op.signo(a), op.tau_TL(b), H.ish(a,b), 100*(H.ish(a,b)/H.sin_asc - 1));
        end
    end
    H.signo = op.signo; H.tau_TL = op.tau_TL;
    A.hover = H;
end

%% Paso 2: grilla signo x K x tau por minimo ISH (Ec. 4), maniobra de escalones
ns = numel(op.signo); nK = numel(op.K); nT = numel(op.tau_TL);
ISH = nan(ns, nK, nT); ISHd = ISH; RMSE = ISH; OK = false(ns, nK, nT);
fprintf('\nPaso 2: escalones de 3 m en x y en y (Omar, Secc. 5.2), 55 s, ISH de toda la corrida\n');
for a = 1:ns
    for i = 1:nK
        for j = 1:nT
            cfg = cfg_pasos(op);
            cfg.omar = parametros_omar(cfg, 'K', op.K(i), 'tau_TL', op.tau_TL(j), ...
                'signo', op.signo(a), 'archivo', 'ninguno');
            R = sim_controlador(@controlador_omar, cfg);
            [ISH(a,i,j), ISHd(a,i,j)] = ish_omar(R, 0);
            OK(a,i,j) = R.completo;
            RMSE(a,i,j) = sqrt(mean(sum((R.xd - R.xl).^2, 1)));
            fprintf('  signo %+d  K = %.2f  tau = %.2f T_L  ISH = %8.4f  RMSE x = %.3f  %s\n', ...
                op.signo(a), op.K(i), op.tau_TL(j), ISH(a,i,j), RMSE(a,i,j), R.motivo);
        end
    end
end
J = ISH; J(~OK) = inf;
[~, idx] = min(J(:));
[ia, ib, jb] = ind2sub(size(J), idx);

% referencia: seguidor sin ASC en la misma maniobra
cfg = cfg_pasos(op);
cfg.omar = parametros_omar(cfg, 'activo', false, 'archivo', 'ninguno');
R = sim_controlador(@controlador_omar, cfg);
ish0 = ish_omar(R, 0);

A.grilla = struct('signo', op.signo, 'K', op.K, 'tau_TL', op.tau_TL, 'ISH', ISH, ...
    'RMSE_x', RMSE, 'completo', OK, 'ISH_sin_asc', ish0, ...
    'maniobra', 'escalones 3 m x, 3 m y, vuelta (Omar 2022, Secc. 5.2)', 'backend', op.backend);
A.signo = op.signo(ia); A.K = op.K(ib); A.tau_TL = op.tau_TL(jb);
A.fecha = datestr(now, 'yyyy-mm-dd HH:MM');
TL = 2*pi/sqrt(cfg.planta.g/cfg.planta.l);
fprintf('\nOptimo: signo %+d, K = %.2f, tau = %.2f T_L (%.3f s). ISH = %.4f (sin ASC: %.4f, %+.0f %%)\n', ...
    A.signo, A.K, A.tau_TL, A.tau_TL*TL, ISH(ia,ib,jb), ish0, 100*(ISH(ia,ib,jb)/ish0 - 1));
if (nK > 1 && (ib == 1 || ib == nK)) || (nT > 1 && (jb == 1 || jb == nT))
    fprintf('Aviso: el optimo esta en el borde de la grilla. Ampliarla y repetir.\n');
end

%% Paso 3: verificacion en el circulo
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
        R = sim_controlador(@controlador_omar, cfg);
        [V.(nom{c}).ish, V.(nom{c}).ish_dev] = ish_omar(R, op.t_inicio);
        sel = R.t >= op.t_inicio;
        V.(nom{c}).rmse_x = sqrt(mean(sum((R.xd(:,sel) - R.xl(:,sel)).^2, 1)));
        V.(nom{c}).completo = R.completo;
        fprintf('  %-8s ISH = %.4f  ISH_dev = %.4f  RMSE x = %.3f  %s\n', nom{c}, ...
            V.(nom{c}).ish, V.(nom{c}).ish_dev, V.(nom{c}).rmse_x, R.motivo);
    end
    A.verificacion = V;
end

if op.guardar && strcmp(op.backend, 'simquad')
    ajuste = A; %#ok<NASGU>
    save(arch, 'ajuste');
    fprintf('Guardado en %s\n', arch);
elseif op.guardar
    fprintf('Backend %s: no se guarda omar_ajuste.mat (el ajuste del paper sale de SimQuad).\n', op.backend);
end
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
