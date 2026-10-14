%% graficar_resultados_paper.m
% Regenera las figuras del artículo a partir de los .mat de experimentos
% (estructuras R = series temporales, M = métricas).
% Recorre todos los .mat de la carpeta de resultados.
clear; close all; clc;

%% ================= Configuración =================
%CarpetaResultados  = 'C:\Repositorios\uav-payload-asta-validation\paper_revision\results\20261010_011019\raw';
CarpetaResultados = 'C:\Repositorios\uav-payload-asta-validation\results\E5_lemn_viento\raw';
% CarpetaResultados  = 'C:\Repositorios\uav-payload-asta-validation\paper_revision\results\20261013_163051\raw';
CarpetaFiguras     = fullfile(CarpetaResultados, 'figuras');
ExportarFiguras    = true;   % guarda .fig y .png de cada figura
ExportarPDF        = false;  % además .pdf vectorial (recomendado para el paper)
Resolucion         = 300;    % dpi del .png
CerrarTrasExportar = true;   % cierra las figuras de cada caso tras exportarlas

% Formato de las figuras del artículo
F.LinAncho = 2.0;
F.FntSize  = 14;
F.colors   = lines(4);

%% ================= Procesar archivos =================
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
    % Copia R y añade las señales que el artículo grafica pero no se guardan
    D = R;
    t = R.t;

    % Velocidad verdadera de la carga. Convencion del codigo: h = x - x_l,
    % por lo que v_l = v - hdot. Se usa M.serie.vl (derivada de R.xl) si existe.
    serie0 = getf(M, 'serie', struct());
    if isfield(serie0, 'vl') && size(serie0.vl, 2) == numel(t)
        D.vl = serie0.vl;
    else
        D.vl = [diff(R.xl, 1, 2) ./ diff(t), zeros(3,1)];
    end

    % Señales de oscilación: se toman de M.serie si existen, si no se
    % calculan del vector cuerda h = x - x_l
    serie = getf(M, 'serie', struct());
    h  = R.h;   hp = R.hp;
    L2 = sum(h.^2, 1);
    D.alpha = atan2(vecnorm(h(1:2,:)), abs(h(3,:)));            % ángulo respecto a la vertical
    D.rho   = vecnorm(h(1:2,:));                                % desplazamiento horizontal
    D.vperp = vecnorm(hp - (sum(hp.*h,1)./L2).*h);              % velocidad relativa ⟂ a la cuerda
    D.fuente_osc = 'calculada de R.h';
    camposSerie = {'alpha','rho','vperp'};
    for c = camposSerie
        nombre = buscarCampo(serie, c{1});
        if ~isempty(nombre) && numel(serie.(nombre)) == numel(t)
            D.(c{1}) = serie.(nombre)(:).';
            D.fuente_osc = 'M.serie';
        end
    end

    % Velocidad de yaw para el error de velocidad
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

    %% 1) Errores de posición de la carga
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

    %% 2) Error de velocidad.
    % R.ev es la serie del script de SimQuad: [v; psi_dot] - u, con psi_dot medido.
    % Si no está, se usa v_d - v_l y la derivada del yaw (corridas de paper_revision).
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

    %% 3) Ángulo de oscilación, rho y velocidad tangencial
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

    f = nuevaFig(); tituloFig(f);
    subplot(3,1,1); hold on; grid on;
    plot(t, D.alpha, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$\alpha$ [rad]'); lg('$\alpha_\ell$');
    subplot(3,1,2); hold on; grid on;
    plot(t, D.rho, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$\rho$ [m]'); lg('$\rho(t)$');
    subplot(3,1,3); hold on; grid on;
    [vt, malo, ev] = marcar_lecturas_atrasadas(D, D.vt);    
    plot(D.t, vt, 'LineWidth', LinAncho, 'Color', colors(1,:));
%     plot(t, D.vperp, 'LineWidth', LinAncho, 'Color', colors(1,:));
    ly('$||v_\perp||$ [m/s]'); lx('Time [s]'); lg('$||v_\perp(t)||$');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'oscilacion');

    %% 4) Plano XY: deseada vs carga
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

    %% 5) Posiciones X, Y, Z y yaw de la carga
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

    %% 6) Parámetros adaptativos theta
    f = nuevaFig(); tituloFig(f); hold on; grid on;
    n = size(D.theta,1);
    for c = 1:n
        plot(t, D.theta(c,:), 'LineWidth', LinAncho, 'Color', colors(c,:));
    end
    etqT = arrayfun(@(c) sprintf('$\\hat{\\theta}_{%d}$', c), 1:n, 'UniformOutput', false);
    lg(etqT{:}); ly('$\hat{\theta}$'); lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'parametros_theta');

    %% 7) Superficie deslizante
    f = nuevaFig(); tituloFig(f); hold on; grid on;
    etqS = {'$s_x$','$s_y$','$s_z$','$s_{\psi}$'};
    n = size(D.s,1);
    for c = 1:n
        plot(t, D.s(c,:), 'LineWidth', LinAncho, 'Color', colors(c,:));
    end
    lg(etqS{1:n}); lx('Time [s]');
    [figs, nombres] = agregar(Bv, figs, nombres, f, 'superficie');

    %% 8) Referencia vs dron vs carga (posiciones)
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

    %% 9) Referencia vs dron vs carga (velocidades)
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

    %% 10) Señales de control
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
    % Comprobación: lo calculado debe coincidir con las métricas guardadas
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

%% ================= Utilidades =================
function f = nuevaFig()
    f = figure('Position', [50 50 1200 600], 'Color', 'w');
end

function v = getf(D, nombre, defecto)
    if isstruct(D) && isfield(D, nombre), v = D.(nombre); else, v = defecto; end
end

function nombre = buscarCampo(S, clave)
    % Devuelve el primer campo de S cuyo nombre contenga 'clave'
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
% Filas [t_ini t_fin wx wy wz]. Vacío si el caso no tiene viento.
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
% Bandas semitransparentes sobre los intervalos de viento, detrás de las curvas.
% El primer bloque (lateral) va en verde y el segundo (oblicuo) en azul.
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