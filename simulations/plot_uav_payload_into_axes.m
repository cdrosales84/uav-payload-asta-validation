function plot_uav_payload_into_axes(ax, k_frame, xd, yd, zd, yQuad, yXil, yRope, ...
    payload, tam_carga, frameRate, varargin)

% Dibuja un frame de la simulacion UAV-carga-cuerda en un axes existente.
%
% Opciones (Name-Value):
%   'WindHist'      — matriz 3xN de viento (igual que plot_uav_payload_frame)
%   'AxesMode'      — 'global' (trayectoria completa) o 'local' (zoom alrededor UAV+carga)
%   'LocalRadius'   — semilado del cubo de vista en modo local [m], default 1.2
%   'DrawWind'      — logical, default true en global y false en local
%   'TitleStr'      — titulo del axes (default segun t y modo)
%   'ShowTimeTitle' — si false, no pone title (default true)

p = inputParser;
addParameter(p, 'WindHist', []);
addParameter(p, 'AxesMode', 'global', @(s) any(strcmpi(s, {'global','local'})));
addParameter(p, 'LocalRadius', 1.2, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'DrawWind', []);
addParameter(p, 'TitleStr', '');
addParameter(p, 'ShowTimeTitle', true, @(x) islogical(x) && isscalar(x));
parse(p, varargin{:});

wind_hist = p.Results.WindHist;
axesMode = lower(p.Results.AxesMode);
localR = p.Results.LocalRadius;
titleStr = p.Results.TitleStr;
showTimeTitle = p.Results.ShowTimeTitle;

drawWind = p.Results.DrawWind;
if isempty(drawWind)
    drawWind = strcmp(axesMode, 'global');
end

N = size(yQuad, 2);
k_frame = max(1, min(k_frame, N));

hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
axes(ax);

xlabel(ax, 'x [m]', 'FontSize', 11, 'Interpreter', 'latex');
ylabel(ax, 'y [m]', 'FontSize', 11, 'Interpreter', 'latex');
zlabel(ax, 'z [m]', 'FontSize', 11, 'Interpreter', 'latex');

view(ax, 35, 23);
axis(ax, 'equal');
axis(ax, 'vis3d');

margin = 1.5;
x_all = [xd(:); yQuad(1, :).'];
y_all = [yd(:); yQuad(2, :).'];
z_all = [zd(:); yQuad(3, :).'];

if ~isempty(yXil)
    x_all = [x_all; yXil(1, :).'];
    y_all = [y_all; yXil(2, :).'];
    z_all = [z_all; yXil(3, :).'];
end

if strcmp(axesMode, 'global')
    xmin = min(x_all) - margin;
    xmax = max(x_all) + margin;
    xmin = -8;
    ymin = -8;
    ymax = 12;
    zmin = max(0, min([z_all; 0]) - 0.2);
    zmax = max(z_all) + margin;
else
    pq = [yQuad(1, k_frame); yQuad(2, k_frame); yQuad(3, k_frame)];
    if payload >= 0.001 && ~isempty(yXil)
        pl = [yXil(1, k_frame); yXil(2, k_frame); yXil(3, k_frame)];
        center = 0.5 * (pq + pl);
    else
        center = pq;
    end
    r = localR;
    xmin = center(1) - r;
    xmax = center(1) + r;
    ymin = center(2) - r;
    ymax = center(2) + r;
    zmin = max(0, center(3) - r - 0.05);
    zmax = center(3) + r;
end

xlim(ax, [xmin xmax]);
ylim(ax, [ymin ymax]);
zlim(ax, [zmin zmax]);

[Xg, Yg] = meshgrid(linspace(xmin, xmax, 2), linspace(ymin, ymax, 2));
Zg = zeros(size(Xg));
surf(ax, Xg, Yg, Zg, ...
    'FaceColor', [0.96 0.96 0.96], ...
    'EdgeColor', [0.85 0.85 0.85], ...
    'FaceAlpha', 0.35);

lighting(ax, 'gouraud');
camlight(ax, 'headlight');

color_matlab = get(groot, 'defaultAxesColorOrder');
colorquad = color_matlab(1, :);
colorFrenteQuad = [1 0 1];
colorBox = [0.38 0.1 0.035];
colorCuerda = [0.5430 0.2695 0.0742];
colorWind = [0.12 0.62 0.22];

scale = 0.573;
Nquadrotor_parametros(1, scale, colorFrenteQuad, colorquad);

hRef = plot3(ax, xd, yd, zd, '--r', 'LineWidth', 1.2);

plot3(ax, yQuad(1, 1:k_frame), yQuad(2, 1:k_frame), yQuad(3, 1:k_frame), ...
    '-', 'Color', colorquad, 'LineWidth', 1.8);

hPay = gobjects(0);
if payload >= 0.001 && ~isempty(yXil)
    hPay = plot3(ax, yXil(1, 1:k_frame), yXil(2, 1:k_frame), yXil(3, 1:k_frame), ...
        '-', 'Color', colorBox, 'LineWidth', 1.8);
end

plot3(ax, xd(k_frame), yd(k_frame), zd(k_frame), ...
    's', 'Color', [1 0 0], 'MarkerFaceColor', [1 0 0], 'MarkerSize', 6);

x = [yQuad(1, k_frame), yQuad(2, k_frame), yQuad(3, k_frame), ...
     yQuad(13, k_frame), yQuad(14, k_frame), yQuad(15, k_frame)];

Nquadrotor(1, [x(1), x(2), x(3), x(4), x(5), x(6)]);

if payload >= 0.001
    r = yRope(1:3:40, :, k_frame);

    plot3(ax, r(:, 1), r(:, 2), r(:, 3) + 0.0025, ...
        'Color', colorCuerda, 'LineWidth', 1.6);

    plot3(ax, [r(1, 1) x(1)], [r(1, 2) x(2)], [r(1, 3) x(3)], ...
        'Color', colorquad, 'LineWidth', 2.0);

    dirBoxT = squeeze(r(end-1, :) - r(end, :));
    dirBoxT = dirBoxT / max(norm(dirBoxT), 1e-12);
    moverBox = dirBoxT * 0;

    if yXil(3, k_frame) < tam_carga / 2
        Hbox = cube_plot([r(end, 1) - moverBox(1), ...
                          r(end, 2) - moverBox(2), ...
                          tam_carga / 2], ...
                          tam_carga, tam_carga, tam_carga, colorBox);
    else
        Hbox = cube_plot([r(end, 1) - moverBox(1), ...
                          r(end, 2) - moverBox(2), ...
                          r(end, 3) - moverBox(3) - tam_carga / 2], ...
                          tam_carga, tam_carga, tam_carga, colorBox);
    end

    rotate(Hbox, [1 0 0], rad2deg(-asin(dirBoxT(2))), ...
        [r(end, 1) - moverBox(1), r(end, 2) - moverBox(2), r(end, 3) - moverBox(3)]);
    rotate(Hbox, [0 1 0], rad2deg(asin(dirBoxT(1))), ...
        [r(end, 1) - moverBox(1), r(end, 2) - moverBox(2), r(end, 3) - moverBox(3)]);
end

if drawWind
    if ~isempty(wind_hist)
        w_now = wind_hist(:, k_frame);
    else
        w_now = [0; 0; 0];
    end

    windTextPos  = [xmin - 0 * 0.18 * (xmax - xmin), ...
                    0 * ymax - 0.08 * (ymax - ymin), ...
                    zmax - 0.05 * (zmax - zmin)];

    if norm(w_now) > 0
        uvw = w_now(:).' / max(norm(w_now), 1e-12);
        Lq  = min(1.2, 0.35 + 0.08 * norm(w_now));

        xs = linspace(xmin + 0.15 * (xmax - xmin), xmax - 0.15 * (xmax - xmin), 4);
        ys = linspace(ymin + 0.15 * (ymax - ymin), ymax - 0.15 * (ymax - ymin), 5);
        zs = linspace(zmin + 0.12 * (zmax - zmin), zmax - 0.12 * (zmax - zmin), 2);

        [Xw, Yw, Zw] = ndgrid(xs, ys, zs);
        Xw = Xw(:); Yw = Yw(:); Zw = Zw(:);
        Uw = uvw(1) * Lq * ones(size(Xw));
        Vw = uvw(2) * Lq * ones(size(Xw));
        Ww = uvw(3) * Lq * ones(size(Xw));

        quiver3(ax, Xw, Yw, Zw, Uw, Vw, Ww, ...
            'Color', colorWind, ...
            'LineWidth', 1.0, ...
            'AutoScale', 'off', ...
            'MaxHeadSize', 0.85);
        text(ax, windTextPos(1), windTextPos(2), windTextPos(3), ...
            sprintf('Wind = [%.2f, %.2f, %.2f] m/s\n|w| = %.2f m/s', ...
            w_now(1), w_now(2), w_now(3), norm(w_now)), ...
            'Color', colorWind, ...
            'FontSize', 9, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle', ...
            'Interpreter', 'none');

        set(ax, 'Color', [0.95 0.985 0.95]);
    else
        text(ax, windTextPos(1), windTextPos(2), windTextPos(3), ...
            'Wind OFF', ...
            'Color', [0.4 0.4 0.4], ...
            'FontSize', 9, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'middle', ...
            'Interpreter', 'none');
    end
end

if showTimeTitle
    if isempty(titleStr)
        if strcmp(axesMode, 'local')
            titleStr = sprintf('Detalle (t = %.2f s)', (k_frame - 1) / frameRate);
        else
            titleStr = sprintf('Vista global (t = %.2f s)', (k_frame - 1) / frameRate);
        end
    end
    title(ax, titleStr, 'FontSize', 12, 'FontWeight', 'bold');
end

lighting(ax, 'gouraud');
camlight(ax, 'headlight');

% Legend al final + AutoUpdate off: evita que UAV/cuerda/caja entren como dataN
if ~isempty(hPay)
    legend(ax, [hRef, hPay], {'$\mathbf{x}_d$', '$\mathbf{x}_\ell$'}, ...
        'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast', ...
        'AutoUpdate', 'off');
else
    legend(ax, hRef, {'$\mathbf{x}_d$'}, ...
        'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast', ...
        'AutoUpdate', 'off');
end
end
