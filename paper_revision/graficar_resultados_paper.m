%% graficar_resultados_paper.m
% Regenera las figuras del artículo a partir de los .mat exportados por la
% simulación (bloque "Exportar figuras y datos para el paper").
% Recorre todos los archivos *_datos.mat de la carpeta de resultados.
clear; close all; clc;

%% ================= Configuración =================
CarpetaResultados = 'C:\Repositorios\uav-payload-asta-validation\paper_revision\results\E2_circ_cinematico\raw';
CarpetaFiguras    = fullfile(CarpetaResultados, 'figuras');
ExportarFiguras   = true;    % guarda .fig y .png de cada figura
Resolucion        = 300;     % dpi del .png (150 en el script original)
CerrarTrasExportar = true;   % cierra figuras de cada caso tras exportarlas

% Formato por defecto (solo se usa si el .mat no trae estas variables)
Formato.LinAncho = 1.5;
Formato.FntSize  = 14;
Formato.colors   = lines(4);

%% ================= Procesar archivos =================
archivos = dir(fullfile(CarpetaResultados, '*.mat'));
if isempty(archivos)
    error('No se encontraron archivos *_datos.mat en %s', CarpetaResultados);
end
if ExportarFiguras && ~exist(CarpetaFiguras,'dir'), mkdir(CarpetaFiguras); end

for k = 1:numel(archivos)
    archivo = fullfile(archivos(k).folder, archivos(k).name);
    fprintf('\n[%d/%d] %s\n', k, numel(archivos), archivos(k).name);
    D = load(archivo);
    [~, prefijo] = fileparts(archivo);
    prefijo = erase(prefijo, '_datos');

    [figs, nombres] = graficarCaso(D, Formato);

    if ExportarFiguras
        for i = 1:numel(figs)
            base = fullfile(CarpetaFiguras, sprintf('%s_%s', prefijo, nombres{i}));
            saveas(figs(i), [base '.fig']);
            exportgraphics(figs(i), [base '.png'], 'Resolution', Resolucion);
            % Para el paper en vectorial:
            % exportgraphics(figs(i), [base '.pdf'], 'ContentType', 'vector');
        end
        fprintf('  %d figuras exportadas en %s\n', numel(figs), CarpetaFiguras);
        if CerrarTrasExportar && numel(archivos) > 1, close(figs); end
    end
end

%% =====================================================================
function [figs, nombres] = graficarCaso(D, F)
    % Formato: usa el del .mat si existe, si no el por defecto
    LinAncho = getf(D, 'LinAncho', F.LinAncho);
    FntSize  = getf(D, 'FntSize',  F.FntSize);
    colors   = getf(D, 'colors',   F.colors);
    ControlCarga = getf(D, 'ControlCarga', true);

    idx = 1:D.yTFind;
    t   = D.tt(idx);
    figs = gobjects(0); nombres = {};
    lx = @(s) xlabel(s,'Interpreter','Latex','FontSize',FntSize);
    ly = @(s) ylabel(s,'Interpreter','Latex','FontSize',FntSize);
    lg = @(varargin) legend(varargin{:},'Interpreter','Latex','FontSize',FntSize);

    %% 1) Errores de posición de la carga
    if tiene(D, {'yExil','yEpsi'})
        f = nuevaFig();
        etq = {'$\tilde{X}$ [m]','$\tilde{Y}$ [m]','$\tilde{Z}$ [m]','$\tilde{\psi}$ [rad]'};
        datos = {D.yExil(1,idx), D.yExil(2,idx), D.yExil(3,idx), D.yEpsi(idx)};
        for j = 1:4
            subplot(4,1,j); hold on; grid on;
            plot(t, datos{j}, 'LineWidth', LinAncho, 'Color', colors(1,:));
            ly(etq{j});
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'error_posicion');
    end

    %% 2) Error de seguimiento de velocidad
    if tiene(D, {'yXip','yPsip','yU'})
        TE = [D.yXip; D.yPsip] - D.yU;
        f = nuevaFig();
        etq = {'$\dot{\tilde{X}}$ [m/s]','$\dot{\tilde{Y}}$ [m/s]', ...
               '$\dot{\tilde{Z}}$ [m/s]','$\dot{\tilde{\psi}}$ [rad/s]'};
        for j = 1:4
            subplot(4,1,j); hold on; grid on;
            plot(t, TE(j,idx), 'LineWidth', LinAncho, 'Color', colors(1,:));
            ly(etq{j});
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'error_velocidad');
    else
        avisar('error de velocidad', {'yXip','yPsip'});
    end

    %% 3) Ángulo de oscilación, rho y velocidad tangencial
    if tiene(D, {'alpha_l','rho','vperp_norm'})
        f = nuevaFig();
        subplot(3,1,1); hold on; grid on;
        plot(t, D.alpha_l(idx), 'LineWidth', LinAncho, 'Color', colors(1,:));
        ly('$\alpha$ [rad]'); lg('$\alpha_\ell$');

        subplot(3,1,2); hold on; grid on;
        plot(t, D.rho(idx), 'LineWidth', LinAncho, 'Color', colors(1,:));
        ly('$\rho$ [m]'); lg('$\rho(t)$');

        subplot(3,1,3); hold on; grid on;
        plot(t, D.vperp_norm(idx), 'LineWidth', LinAncho, 'Color', colors(1,:));
        ly('$||v_\perp||$ [m/s]'); lx('Time [s]'); lg('$||v_\perp(t)||$');
        [figs, nombres] = agregar(figs, nombres, f, 'oscilacion');
    else
        avisar('oscilación', {'alpha_l','rho','vperp_norm'});
    end

    %% 4) Plano XY: deseada vs carga
    if tiene(D, {'yXid','yXil'})
        f = nuevaFig();
        comp = {'x','y'};
        for j = 1:2
            subplot(2,1,j); hold on; grid on;
            plot(t, D.yXid(j,idx), '--', 'LineWidth', LinAncho, 'Color', colors(2,:));
            plot(t, D.yXil(j,idx), '-',  'LineWidth', LinAncho, 'Color', colors(1,:));
            ly(sprintf('$%s_\\ell$ [m]', comp{j}));
            lg(sprintf('$%s_d$', comp{j}), sprintf('$%s_\\ell$', comp{j}));
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'seguimiento_xy');
    end

    %% 5) Posiciones X, Y, Z y yaw (carga o dron)
    if tiene(D, {'yXid','yPsid','yQuad'})
        if ControlCarga && isfield(D,'yXil')
            yX = D.yXil;
            leg = {'$x_\ell$','$y_\ell$','$z_\ell$','$\psi_\ell$'};
        else
            yX = D.yQuad(1:3,:);
            leg = {'$x$','$y$','$z$','$\psi$'};
        end
        legd = {'$x_d$','$y_d$','$z_d$','$\psi_d$'};
        etq  = {'x [m]','y [m]','z [m]','$\psi$ [rad]'};
        f = nuevaFig();
        for j = 1:4
            subplot(4,1,j); hold on; grid on;
            if j < 4
                plot(t, D.yXid(j,idx), '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
                plot(t, yX(j,idx),     '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
            else
                plot(t, D.yPsid(1,idx),  '-', 'LineWidth', LinAncho, 'Color', colors(1,:));
                plot(t, D.yQuad(15,idx), '-', 'LineWidth', LinAncho, 'Color', colors(2,:));
            end
            lg(legd{j}, leg{j}); ly(etq{j});
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'posiciones');
    end

    %% 6) Ganancias adaptativas Ku y Kv
    if tiene(D, {'Gan_U','Gan_V'})
        f = nuevaFig();
        G = {D.Gan_U, D.Gan_V}; pre = {'Ku','Kv'};
        for j = 1:2
            subplot(2,1,j); hold on; grid on;
            for c = 1:4
                plot(t, G{j}(c,idx), 'LineWidth', LinAncho, 'Color', colors(c,:));
            end
            etqK = arrayfun(@(c) sprintf('$%s_{%d}$', pre{j}, c), 1:4, 'UniformOutput', false);
            lg(etqK{:});
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'ganancias');
    end

    %% 7) Superficie deslizante
    if tiene(D, {'s'})
        f = nuevaFig(); hold on; grid on;
        for c = 1:4
            plot(t, D.s(c,idx), 'LineWidth', LinAncho, 'Color', colors(c,:));
        end
        lg('$s_x$','$s_y$','$s_z$','$s_{\psi}$'); lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'superficie');
    end

    %% 8) Referencia vs dron vs carga (posiciones)
    if tiene(D, {'yXid','yQuad','yXil'})
        f = nuevaFig();
        etq = {'X [m]','Y [m]','Z [m]'};
        for j = 1:3
            subplot(3,1,j); hold on; grid on;
            plot(t, D.yXid(j,idx), 'r');
            plot(t, D.yQuad(j,idx), 'k');
            plot(t, D.yXil(j,idx));
            ly(etq{j});
            if j == 1, lg('$x_d$','$x$','$x_\ell$'); end
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'ref_dron_carga_pos');
    end

    %% 9) Referencia vs dron vs carga (velocidades)
    if tiene(D, {'yXidp','yQuad','yHp'})
        f = nuevaFig();
        etq = {'$\dot{x}$ [m/s]','$\dot{y}$ [m/s]','$\dot{z}$ [m/s]'};
        for j = 1:3
            subplot(3,1,j); hold on; grid on;
            plot(t, D.yXidp(j,idx), 'r');
            plot(t, D.yQuad(3+j,idx), 'k');
            plot(t, D.yHp(j,idx) + D.yQuad(3+j,idx));
            ly(etq{j});
            if j == 1, lg('$\dot{x}_d$','$\dot{x}$','$\dot{x}_\ell$'); end
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'ref_dron_carga_vel');
    end

    %% 10) Error de velocidad en el marco local
    if tiene(D, {'TrackingErrorDyn'})
        f = nuevaFig();
        etq = {'$e_{vx}$ [m/s]','$e_{vy}$ [m/s]','$e_{vz}$ [m/s]','$e_{v\psi}$ [rad/s]'};
        for j = 1:4
            subplot(4,1,j); hold on; grid on;
            plot(t, D.TrackingErrorDyn(j,idx), 'LineWidth', LinAncho, 'Color', colors(j,:));
            ly(etq{j});
        end
        lx('Time [s]');
        [figs, nombres] = agregar(figs, nombres, f, 'error_vel_local');
    else
        avisar('error de velocidad local', {'TrackingErrorDyn'});
    end

    %% Resumen de métricas guardadas
    met = {'RMSE_pos','RMSE_vel_payload','RMSE_alpha','max_alpha','rho_rms','max_rho','vperp_rms','vperp_peak'};
    for m = met
        if isfield(D, m{1}), fprintf('  %-17s = %s\n', m{1}, mat2str(D.(m{1}), 4)); end
    end
end

%% ================= Utilidades =================
function f = nuevaFig()
    f = figure('Position', [50 50 1200 600], 'Color', 'w');
end

function v = getf(D, nombre, defecto)
    if isfield(D, nombre), v = D.(nombre); else, v = defecto; end
end

function ok = tiene(D, vars)
    ok = all(isfield(D, vars));
end

function avisar(fig, vars)
    fprintf('  Aviso: figura de %s omitida (faltan en el .mat: %s)\n', fig, strjoin(vars, ', '));
end

function [figs, nombres] = agregar(figs, nombres, f, nombre)
    figs(end+1) = f; nombres{end+1} = nombre;
end