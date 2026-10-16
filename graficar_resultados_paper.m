%% graficar_resultados_paper.m
% Regenerates the paper figures from the experiment .mat files
% (structures R = time series, M = metrics).
% Walks every .mat file in the results folder.
clear; close all; clc;

%% ================= Configuration =================
%CarpetaResultados  = 'C:\Repositorios\uav-payload-asta-validation\paper_revision\results\20261010_011019\raw';
CarpetaResultados = 'C:\Users\claud\GitHub\paper_revision\results\E6_circ_smc\raw';
CarpetaFiguras     = fullfile(CarpetaResultados, 'figuras');
ExportarFiguras    = true;   % saves .fig and .png of each figure
ExportarPDF        = false;  % also a vector .pdf (recommended for the paper)
Resolucion         = 300;    % dpi of the .png
CerrarTrasExportar = true;   % closes each case's figures after exporting them

% Format of the paper figures
F.LinAncho = 2.0;
F.FntSize  = 14;
F.colors   = lines(4);

%% ================= Process files =================
archivos = dir(fullfile(CarpetaResultados, '*.mat'));
if isempty(archivos)
    error('No se encontraron archivos .mat en %s', CarpetaResultados);
end
if ExportarFiguras && ~exist(CarpetaFiguras,'dir'), mkdir(CarpetaFiguras); end

for k = 1:numel(archivos)
    archivo = fullfile(archivos(k).folder, archivos(k).name);
    [~, experimento] = fileparts(archivo);
    fprintf('\n[%d/%d] %s\n', k, numel(archivos), archivos(k).name);

    S = load(archivo);
    if ~isfield(S, 'R') || ~isfield(S.R, 't')
        fprintf('  Omitido: no contiene la estructura R con R.t\n');
        continue
    end
    if ~isfield(S, 'M'), S.M = struct(); end
    if isfield(S.R, 'completo') && ~S.R.completo
        fprintf('  Aviso: simulación incompleta (%s)\n', getf(S.R, 'motivo', ''));
    end

    D = calcularDerivadas(S.R, S.M);
    [figs, nombres] = graficarCaso(D, F, experimento);
    imprimirMetricas(D, S.M);

    if ExportarFiguras
        for i = 1:numel(figs)
            base = fullfile(CarpetaFiguras, sprintf('%s_%s', experimento, nombres{i}));
            saveas(figs(i), [base '.fig']);
            exportgraphics(figs(i), [base '.png'], 'Resolution', Resolucion);
            if ExportarPDF
                exportgraphics(figs(i), [base '.pdf'], 'ContentType', 'vector');
            end
        end
        fprintf('  %d figuras exportadas en %s\n', numel(figs), CarpetaFiguras);
        if CerrarTrasExportar && numel(archivos) > 1, close(figs); end
    end
end

%% =====================================================================
function D = calcularDerivadas(R, M)
    % Copies R and adds the signals the paper plots but that are not stored
    D = R;
    t = R.t;

    % True payload velocity. Code convention: h = x - x_l,
    % so v_l = v - hdot. M.serie.vl (derivative of R.xl) is used if it exists.
    serie0 = getf(M, 'serie', struct());
    if isfield(serie0, 'vl') && size(serie0.vl, 2) == numel(t)
        D.vl = serie0.vl;
    else
        D.vl = [diff(R.xl, 1, 2) ./ diff(t), zeros(3,1)];
    end

    % Swing signals: taken from M.serie if present, otherwise computed
    % from the cable vector h = x - x_l
    serie = getf(M, 'serie', struct());
    h  = R.h;   hp = R.hp;
    L2 = sum(h.^2, 1);
    D.alpha = atan2(vecnorm(h(1:2,:)), abs(h(3,:)));            % angle from the vertical
    D.rho   = vecnorm(h(1:2,:));                                % horizontal displacement
    D.vperp = vecnorm(hp - (sum(hp.*h,1)./L2).*h);              % relative velocity perpendicular to the cable
    D.fuente_osc = 'calculada de R.h';
    camposSerie = {'alpha','rho','vperp'};
    for c = camposSerie
        nombre = buscarCampo(serie, c{1});
        if ~isempty(nombre) && numel(serie.(nombre)) == numel(t)
            D.(c{1}) = serie.(nombre)(:).';
            D.fuente_osc = 'M.serie';
        end
    end

    % Yaw rate for the velocity error
    D.psip = gradient(unwrap(R.psi), t);
end

function [figs, nombres] = graficarCaso(D, F, experimento)
    LinAncho = F.LinAncho; FntSize = F.FntSize; colors = F.colors;
    t = D.t;
    Bv = bloques_viento(D);
    figs = gobjects(0); nombres = {};
    lx = @(s) xlabel(s,'Interpreter','Latex','FontSize',FntSize);
    ly = @(s) ylabel(s,'Interpreter','Latex','FontSize',FntSize);
    lg = @(varargin) legend(varargin{:},'Interpreter','Latex','FontSize',FntSize);
    tituloFig = @(f) set(f, 'Name', experimento, 'NumberTitle', 'off');

    %% 1) Payload position errors
    f = nuevaFig(); tituloFig(f);
    etq   = {'$\tilde{X}$ [m]','$\tilde{Y}$ [m]','$\tilde{Z}$ [m]','$\tilde{\psi}$ [rad]'};
    datos = {D.ex(1,:), D.ex(2,:), D.ex(3,:), D.epsi};
    for j = 1:4
        subplot(4,1,j); hold on; grid on;
        plot(t, datos{j}, 'LineWidth', LinAncho, 'Color', colors(1,:));
        ly(etq{j});
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'error_posicion');

    %% 2) Velocity error.
    % R.ev is the SimQuad script series: [v; psi_dot] - u, with measured psi_dot.
    % If it is absent, v_d - v_l and the yaw derivative are used (paper_revision runs).
    if isfield(D, 'ev') && size(D.ev, 1) >= 4 && size(D.ev, 2) == numel(t)
        TE = D.ev(1:4, :);
    else
        TE = [D.vd - D.vl; gradient(unwrap(D.psid), t) - D.psip];
    end
    f = nuevaFig(); tituloFig(f);
    etq = {'$\dot{\tilde{X}}$ [m/s]','$\dot{\tilde{Y}}$ [m/s]', ...
           '$\dot{\tilde{Z}}$ [m/s]','$\dot{\tilde{\psi}}$ [rad/s]'};
    for j = 1:4
        subplot(4,1,j); hold on; grid on;
        plot(t, TE(j,:), 'LineWidth', LinAncho, 'Color', colors(1,:));
        ly(etq{j});
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'error_velocidad');

    %% 3) Swing angle, rho, and tangential velocity
    f = nuevaFig(); tituloFig(f);
    subplot(3,1,1); hold on; grid on;
    plot(t, D.alpha, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$\alpha$ [rad]'); lg('$\alpha_\ell$');
    subplot(3,1,2); hold on; grid on;
    plot(t, D.rho, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$\rho$ [m]'); lg('$\rho(t)$');
    subplot(3,1,3); hold on; grid on;
    plot(t, D.vperp, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$||v_\perp||$ [m/s]'); lx('Time [s]'); lg('$||v_\perp(t)||$');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'oscilacion');

%     f = nuevaFig(); tituloFig(f);
%     subplot(3,1,1); hold on; grid on;
%     plot(t, D.alpha, 'LineWidth', LinAncho, 'Color', colors(1,:));
%     ly('$\alpha$ [rad]'); lg('$\alpha_\ell$');
%     subplot(3,1,2); hold on; grid on;
%     plot(t, D.rho, 'LineWidth', LinAncho, 'Color', colors(1,:));
%     ly('$\rho$ [m]'); lg('$\rho(t)$');
%     subplot(3,1,3); hold on; grid on;
%     [vt, malo, ev] = marcar_lecturas_atrasadas(D, D.vt);    
%     plot(D.t, vt, 'LineWidth', LinAncho, 'Color', colors(1,:));
% %     plot(t, D.vperp, 'LineWidth', LinAncho, 'Color', colors(1,:));
%     ly('$||v_\perp||$ [m/s]'); lx('Time [s]'); lg('$||v_\perp(t)||$');
%     [figs, nombres] = agregar(Bv, figs, nombres, f, 'oscilacion');

    %% 4) XY plane: desired vs payload
    f = nuevaFig(); tituloFig(f);
    comp = {'x','y'};
    for j = 1:2
        subplot(2,1,j); hold on; grid on;
        plot(t, D.xd(j,:), '--', 'LineWidth', LinAncho, 'Color', colors(2,:));
        plot(t, D.xl(j,:), '-',  'LineWidth', LinAncho, 'Color', colors(1,:));
        ly(sprintf('$%s_\\ell$ [m]', comp{j}));
        lg(sprintf('$%s_d$', comp{j}), sprintf('$%s_\\ell$', comp{j}));
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'seguimiento_xy');

    %% 5) Payload X, Y, Z positions and yaw
    f = nuevaFig(); tituloFig(f);
    legd = {'$x_d$','$y_d$','$z_d$','$\psi_d$'};
    leg  = {'$x_\ell$','$y_\ell$','$z_\ell$','$\psi_\ell$'};
    etq  = {'x [m]','y [m]','z [m]','$\psi$ [rad]'};
    for j = 1:4
        subplot(4,1,j); hold on; grid on;
        if j < 4
            plot(t, D.xd(j,:), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
            plot(t, D.xl(j,:), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
        else
            plot(t, D.psid, '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
            plot(t, D.psi,  '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
        end
        lg(legd{j}, leg{j}); ly(etq{j});
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'posiciones');

    %% 6) Adaptive parameters theta
    f = nuevaFig(); tituloFig(f); hold on; grid on;
    n = size(D.theta,1);
    for c = 1:n
        plot(t, D.theta(c,:), 'LineWidth', LinAncho, 'Color', colors(c,:));
    end
    etqT = arrayfun(@(c) sprintf('$\\hat{\\theta}_{%d}$', c), 1:n, 'UniformOutput', false);
    lg(etqT{:}); ly('$\hat{\theta}$'); lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'parametros_theta');

    %% 7) Sliding surface
    f = nuevaFig(); tituloFig(f); hold on; grid on;
    etqS = {'$s_x$','$s_y$','$s_z$','$s_{\psi}$'};
    n = size(D.s,1);
    for c = 1:n
        plot(t, D.s(c,:), 'LineWidth', LinAncho, 'Color', colors(c,:));
    end
    lg(etqS{1:n}); lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'superficie');

    %% 8) Reference vs drone vs payload (positions)
    f = nuevaFig(); tituloFig(f);
    etq = {'X [m]','Y [m]','Z [m]'};
    for j = 1:3
        subplot(3,1,j); hold on; grid on;
        plot(t, D.xd(j,:), 'r');
        plot(t, D.x(j,:),  'k');
        plot(t, D.xl(j,:));
        ly(etq{j});
        if j == 1, lg('$x_d$','$x$','$x_\ell$'); end
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'ref_dron_carga_pos');

    %% 9) Reference vs drone vs payload (velocities)
    f = nuevaFig(); tituloFig(f);
    etq = {'$\dot{x}$ [m/s]','$\dot{y}$ [m/s]','$\dot{z}$ [m/s]'};
    for j = 1:3
        subplot(3,1,j); hold on; grid on;
        plot(t, D.vd(j,:), 'r');
        plot(t, D.v(j,:),  'k');
        plot(t, D.vl(j,:));
        ly(etq{j});
        if j == 1, lg('$\dot{x}_d$','$\dot{x}$','$\dot{x}_\ell$'); end
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'ref_dron_carga_vel');

    %% 10) Control signals
    f = nuevaFig(); tituloFig(f);
    etq = {'$u_1$','$u_2$','$u_3$','$u_4$'};
    for j = 1:size(D.u,1)
        subplot(size(D.u,1),1,j); hold on; grid on;
        plot(t, D.u(j,:), 'LineWidth', LinAncho, 'Color', colors(j,:));
        ly(etq{j});
    end
    lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'control');
end

function imprimirMetricas(D, M)
    fprintf('  Oscilación: %s\n', D.fuente_osc);
    % Check: the computed values should match the stored metrics
    chk = {'rms_alpha', rms(D.alpha); 'max_alpha', max(abs(D.alpha));
           'rms_rho',   rms(D.rho);   'max_rho',   max(D.rho);
           'rms_vperp', rms(D.vperp); 'max_vperp', max(D.vperp)};
    fprintf('  %-12s %12s %12s\n', 'métrica', 'en M', 'calculada');
    for i = 1:size(chk,1)
        if isfield(M, chk{i,1})
            fprintf('  %-12s %12.4g %12.4g\n', chk{i,1}, M.(chk{i,1}), chk{i,2});
        end
    end
    for m = {'rmse_ex','rmse_ex_xyz','rmse_evl','rmse_epsi','frac_sat'}
        if isfield(M, m{1}), fprintf('  %-12s = %s\n', m{1}, mat2str(M.(m{1}), 4)); end
    end
end

%% ================= Utilities =================
function f = nuevaFig()
    f = figure('Position', [50 50 1200 600], 'Color', 'w');
end

function v = getf(D, nombre, defecto)
    if isstruct(D) && isfield(D, nombre), v = D.(nombre); else, v = defecto; end
end

function nombre = buscarCampo(S, clave)
    % Returns the first field of S whose name contains 'clave'
    nombre = '';
    if ~isstruct(S), return; end
    f = fieldnames(S);
    i = find(contains(lower(f), lower(clave)), 1);
    if ~isempty(i), nombre = f{i}; end
end

function [figs, nombres] = agregar(Bv, figs, nombres, f, nombre)
    marcar_viento(f, Bv);
    figs(end+1) = f; nombres{end+1} = nombre;
end

function B = bloques_viento(D)
% Rows [t_ini t_fin wx wy wz]. Empty if the case has no wind.
B = zeros(0, 5);
if isfield(D, 'cfg') && isfield(D.cfg, 'viento') && isfield(D.cfg.viento, 'bloques')
    B = D.cfg.viento.bloques;
end
if isempty(B) && isfield(D, 'viento') && isfield(D, 't') && ~isempty(D.viento)
    on = any(abs(D.viento) > 1e-6, 1);
    d = diff([false, on, false]);
    i0 = find(d == 1);
    i1 = find(d == -1) - 1;
    B = zeros(numel(i0), 5);
    for i = 1:numel(i0)
        B(i, 1:2) = [D.t(i0(i)), D.t(i1(i))];
        B(i, 3:5) = D.viento(:, i0(i)).';
    end
end
if ~isempty(B)
    B = B(any(abs(B(:, 3:5)) > 1e-6, 2), :);
end
end

function marcar_viento(f, B)
% Semi-transparent bands over the wind intervals, behind the curves.
% The first block (lateral) is green and the second (oblique) is blue.
if isempty(B), return; end
relleno = [0.75 0.90 0.75; 0.75 0.84 0.95];
borde0  = [0.15 0.55 0.20; 0.20 0.40 0.85];
borde1  = [0.90 0.45 0.10; 0.80 0.15 0.55];
ax = findobj(f, 'Type', 'axes');
for a = 1:numel(ax)
    if strcmp(get(ax(a), 'Tag'), 'legend'), continue; end
    if ~isequal(round(get(ax(a), 'View')), [0 90]), continue; end
    yl = ylim(ax(a));
    hold(ax(a), 'on');
    for i = 1:size(B, 1)
        k = mod(i-1, size(relleno, 1)) + 1;
        hp = patch(ax(a), [B(i,1) B(i,2) B(i,2) B(i,1)], yl([1 1 2 2]), ...
            relleno(k,:), 'FaceAlpha', 0.28, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(ax(a), [B(i,1) B(i,1)], yl, '--', 'Color', borde0(k,:), ...
            'LineWidth', 1.25, 'HandleVisibility', 'off');
        plot(ax(a), [B(i,2) B(i,2)], yl, '--', 'Color', borde1(k,:), ...
            'LineWidth', 1.25, 'HandleVisibility', 'off');
        uistack(hp, 'bottom');
    end
    ylim(ax(a), yl);
    set(ax(a), 'Layer', 'top');
end
end