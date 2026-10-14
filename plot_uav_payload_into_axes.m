function plot_uav_payload_into_axes(ax, varargin)
% PLOT_UAV_PAYLOAD_INTO_AXES  Dibuja un frame de la simulacion UAV-cuerda-carga en un axes existente.
%
% Formato nuevo (resultados de sim_payload_asta / run_paper_experiments):
%   plot_uav_payload_into_axes(ax, R, k)
%   plot_uav_payload_into_axes(ax, R, k, 'AxesMode', 'local', ...)
%       R : estructura R del .mat (tambien acepta S = load(archivo), con S.R)
%       k : indice de muestra (para un tiempo t: [~, k] = min(abs(R.t - t)))
%
% Formato antiguo (se mantiene por compatibilidad con los scripts de ../simulations):
%   plot_uav_payload_into_axes(ax, k, xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate, ...)
%
% Opciones (Name-Value):
%   'WindHist'       matriz 3xN de viento. Formato nuevo: por defecto R.viento
%   'AxesMode'       'global' (trayectoria completa) o 'local' (zoom alrededor de UAV + carga)
%   'LocalRadius'    semilado del cubo de vista en modo local [m], default 1.2
%   'DrawWind'       logical, default true en global y false en local
%   'TitleStr'       titulo del axes (default segun t y modo)
%   'ShowTimeTitle'  si false, no pone titulo (default true)
%   'Limites'        [xmin xmax ymin ymax zmin zmax] en modo global (default: segun los datos)
%   'Margen'         margen [m] de los limites automaticos en modo global, default 1.5
%   'EstimarActitud' formato nuevo: inclina el UAV segun su aceleracion (default true).
%                    R no guarda roll/pitch; se aproximan suponiendo que el empuje apunta
%                    en la direccion de a + g*e_z (solo para visualizacion).
%
% Correspondencia con el formato antiguo:
%   xd, yd, zd          -> R.xd(1,:), R.xd(2,:), R.xd(3,:)
%   yQuad(1:3,:)        -> R.x          yQuad(15,:) -> R.psi    yQuad(13:14,:) -> no se guarda
%   yXil                -> R.xl
%   yRope               -> no se guarda: la cuerda se dibuja recta entre R.x y R.xl (esta tensa)
%   payload, tam_carga  -> R.cfg.planta.mL, R.cfg.planta.tam_carga
%   frameRate           -> no hace falta: el tiempo es R.t(k)
%
% Requiere Nquadrotor, Nquadrotor_parametros y cube_plot (carpeta ../simulations en el path).

if isempty(varargin)
    error('plot_uav_payload_into_axes:args', 'Faltan argumentos.');
end

if isstruct(varargin{1})
    % ---------------- formato nuevo: (ax, R, k, opciones) ----------------
    if numel(varargin) < 2
        error('plot_uav_payload_into_axes:args', 'Uso: plot_uav_payload_into_axes(ax, R, k, ...)');
    end
    R = varargin{1};
    if isfield(R, 'R'), R = R.R; end            % se paso S = load(archivo)
    k_frame = varargin{2};
    opciones = varargin(3:end);
    formatoNuevo = true;
else
    % ---------------- formato antiguo: posicionales ----------------
    if numel(varargin) < 10
        error('plot_uav_payload_into_axes:args', ...
            'Uso antiguo: (ax, k, xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate, ...)');
    end
    [k_frame, xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate] = varargin{1:10};
    opciones = varargin(11:end);
    formatoNuevo = false;
end

p = inputParser;
addParameter(p, 'WindHist', []);
addParameter(p, 'AxesMode', 'global', @(s) any(strcmpi(s, {'global','local'})));
addParameter(p, 'LocalRadius', 1.2, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'DrawWind', []);
addParameter(p, 'TitleStr', '');
addParameter(p, 'ShowTimeTitle', true, @(x) islogical(x) && isscalar(x));
addParameter(p, 'Limites', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 6));
addParameter(p, 'Margen', 1.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'EstimarActitud', true, @(x) islogical(x) && isscalar(x));
parse(p, opciones{:});
op = p.Results;

%% ---------------- datos en una representacion comun ----------------
if formatoNuevo
    D = datos_desde_R(R, op.EstimarActitud);
    if isempty(op.WindHist) && isfield(R, 'viento'), op.WindHist = R.viento; end
else
    D = datos_formato_antiguo(xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate);
end

N = numel(D.t);
k_frame = max(1, min(round(k_frame), N));
axesMode = lower(op.AxesMode);
drawWind = op.DrawWind;
if isempty(drawWind), drawWind = strcmp(axesMode, 'global'); end
hayCarga = D.mL >= 0.001 && ~isempty(D.xl);

pq  = D.xq(:, k_frame);                         % posicion del UAV
att = D.att(:, k_frame);                        % [roll; pitch; yaw]

%% ---------------- ejes ----------------
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
axes(ax);                                       % Nquadrotor y cube_plot dibujan en gca

xlabel(ax, 'x [m]', 'FontSize', 11, 'Interpreter', 'latex');
ylabel(ax, 'y [m]', 'FontSize', 11, 'Interpreter', 'latex');
zlabel(ax, 'z [m]', 'FontSize', 11, 'Interpreter', 'latex');

view(ax, 35, 23);
axis(ax, 'equal');
axis(ax, 'vis3d');

if strcmp(axesMode, 'global')
    if ~isempty(op.Limites)
        L = op.Limites(:).';
    else
        P = [D.xd, D.xq];
        if hayCarga, P = [P, D.xl]; end
        P = P(:, all(isfinite(P), 1));
        L = [min(P(1,:)) max(P(1,:)) min(P(2,:)) max(P(2,:)) min(P(3,:)) max(P(3,:))] ...
            + op.Margen * [-1 1 -1 1 -1 1];
        L(5) = max(0, L(5));                    % no por debajo del suelo
    end
else
    if hayCarga
        centro = 0.5 * (pq + D.xl(:, k_frame));
    else
        centro = pq;
    end
    r = op.LocalRadius;
    L = [centro(1)-r, centro(1)+r, centro(2)-r, centro(2)+r, max(0, centro(3)-r-0.05), centro(3)+r];
end
xmin = L(1); xmax = L(2); ymin = L(3); ymax = L(4); zmin = L(5); zmax = L(6);

xlim(ax, [xmin xmax]);
ylim(ax, [ymin ymax]);
zlim(ax, [0 10]);

% plano de referencia (suelo, o el piso de la vista si el suelo queda fuera)
[Xg, Yg] = meshgrid(linspace(xmin, xmax, 2), linspace(ymin, ymax, 2));
Zg = 0 * ones(size(Xg));
surf(ax, Xg, Yg, Zg, ...
    'FaceColor', [0.96 0.96 0.96], ...
    'EdgeColor', [0.85 0.85 0.85], ...
    'FaceAlpha', 0.35);

lighting(ax, 'gouraud');
camlight(ax, 'headlight');

%% ---------------- colores ----------------
color_matlab = get(groot, 'defaultAxesColorOrder');
colorquad = color_matlab(1, :);
colorFrenteQuad = [1 0 1];
colorBox = [0.38 0.1 0.035];
colorCuerda = [0.5430 0.2695 0.0742];
colorWind = [0.12 0.62 0.22];

%% ---------------- trayectorias ----------------
scale = 0.573;
Nquadrotor_parametros(1, scale, colorFrenteQuad, colorquad);

hRef = plot3(ax, D.xd(1,:), D.xd(2,:), D.xd(3,:), '--r', 'LineWidth', 1.2);

plot3(ax, D.xq(1, 1:k_frame), D.xq(2, 1:k_frame), D.xq(3, 1:k_frame), ...
    '-', 'Color', colorquad, 'LineWidth', 1.8);

hPay = gobjects(0);
if hayCarga
    hPay = plot3(ax, D.xl(1, 1:k_frame), D.xl(2, 1:k_frame), D.xl(3, 1:k_frame), ...
        '-', 'Color', colorBox, 'LineWidth', 1.8);
end

plot3(ax, D.xd(1, k_frame), D.xd(2, k_frame), D.xd(3, k_frame), ...
    's', 'Color', [1 0 0], 'MarkerFaceColor', [1 0 0], 'MarkerSize', 6);

%% ---------------- UAV ----------------
Nquadrotor(1, [pq(1), pq(2), pq(3), att(1), att(2), att(3)]);

%% ---------------- cuerda y carga ----------------
if hayCarga
    r = D.cuerda(k_frame);                      % n x 3, primera fila junto al UAV, ultima = carga

    plot3(ax, r(:, 1), r(:, 2), r(:, 3) + 0.0025, ...
        'Color', colorCuerda, 'LineWidth', 1.6);

    plot3(ax, [r(1, 1) pq(1)], [r(1, 2) pq(2)], [r(1, 3) pq(3)], ...
        'Color', colorquad, 'LineWidth', 2.0);

    dirBoxT = r(end-1, :) - r(end, :);
    dirBoxT = dirBoxT / max(norm(dirBoxT), 1e-12);
    tc = D.tam_carga;
    pc = r(end, :);                             % punto de enganche de la carga

    if D.xl(3, k_frame) < tc / 2
        Hbox = cube_plot([pc(1), pc(2), tc / 2], tc, tc, tc, colorBox);
    else
        Hbox = cube_plot([pc(1), pc(2), pc(3) - tc / 2], tc, tc, tc, colorBox);
    end

    rotate(Hbox, [1 0 0], rad2deg(-asin(dirBoxT(2))), pc);
    rotate(Hbox, [0 1 0], rad2deg(asin(dirBoxT(1))), pc);
end

%% ---------------- viento ----------------
if drawWind
    if ~isempty(op.WindHist) && size(op.WindHist, 2) >= k_frame
        w_now = op.WindHist(:, k_frame);
    else
        w_now = [0; 0; 0];
    end

    windTextPos = [xmin, -0.08 * (ymax - ymin), zmax - 0.05 * (zmax - zmin)];

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

        quiver3(ax, Xw, Yw, Zw, Uw, Vw, Ww, 0, ...
            'Color', colorWind, ...
            'LineWidth', 1.0, ...
            'MaxHeadSize', 0.85);
        text(ax, windTextPos(1), windTextPos(2), windTextPos(3), ...
            sprintf('Wind = [%.2f, %.2f, %.2f] m/s\n|w| = %.2f m/s', ...
            w_now(1), w_now(2), w_now(3), norm(w_now)), ...
            'Color', colorWind, 'FontSize', 9, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
            'Interpreter', 'none');

        set(ax, 'Color', [0.95 0.985 0.95]);
    else
        text(ax, windTextPos(1), windTextPos(2), windTextPos(3), 'Wind OFF', ...
            'Color', [0.4 0.4 0.4], 'FontSize', 9, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
            'Interpreter', 'none');
    end
end

%% ---------------- titulo y leyenda ----------------
if op.ShowTimeTitle
    titleStr = op.TitleStr;
    if isempty(titleStr)
        if strcmp(axesMode, 'local')
            titleStr = sprintf('Detalle (t = %.2f s)', D.t(k_frame));
        else
            titleStr = sprintf('Vista global (t = %.2f s)', D.t(k_frame));
        end
    end
    title(ax, titleStr, 'FontSize', 12, 'FontWeight', 'bold');
end

lighting(ax, 'gouraud');
camlight(ax, 'headlight');

% Leyenda al final + AutoUpdate off: evita que UAV/cuerda/caja entren como dataN
if ~isempty(hPay)
    hLeg = [hRef, hPay]; txtLeg = {'$\mathbf{x}_d$', '$\mathbf{x}_\ell$'};
else
    hLeg = hRef; txtLeg = {'$\mathbf{x}_d$'};
end
try
    legend(ax, hLeg, txtLeg, 'Interpreter', 'latex', 'FontSize', 12, ...
        'Location', 'northwest', 'AutoUpdate', 'off');
catch                                           % versiones sin 'AutoUpdate'
    legend(ax, hLeg, txtLeg, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
end
end

%% ======================================================================
function D = datos_desde_R(R, estimarActitud)
% Representacion comun a partir de la estructura R de sim_payload_asta.
D.t   = R.t(:).';
D.xd  = R.xd;
D.xq  = R.x;
D.xl  = R.xl;
D.mL  = 0.1;  D.tam_carga = 0.1;
if isfield(R, 'cfg') && isfield(R.cfg, 'planta')
    if isfield(R.cfg.planta, 'mL'),        D.mL = R.cfg.planta.mL; end
    if isfield(R.cfg.planta, 'tam_carga'), D.tam_carga = R.cfg.planta.tam_carga; end
end

N = numel(D.t);
D.att = zeros(3, N);
D.att(3, :) = R.psi;
if estimarActitud && isfield(R, 'v')
    D.att(1:2, :) = actitud_cuasiestatica(R.v, R.psi, D.t);
end

% La cuerda no se guarda en R: con el cable tenso es el segmento UAV -> carga
% (mismo numero de puntos que yRope(1:3:40,:,k) del formato antiguo).
npts = 14;
s = linspace(0, 1, npts).';
D.cuerda = @(k) (1 - s) * R.x(:, k).' + s * R.xl(:, k).';
end

function D = datos_formato_antiguo(xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate)
N = size(yQuad, 2);
D.t   = (0:N-1) / frameRate;
D.xd  = [xd(:).'; yd(:).'; zd(:).'];
D.xq  = yQuad(1:3, :);
D.xl  = yXil;
D.mL  = payload;
D.tam_carga = tam_carga;
D.att = yQuad(13:15, :);
D.cuerda = @(k) yRope(1:3:40, :, k);
end

function rp = actitud_cuasiestatica(v, psi, t)
% Roll y pitch aproximados: el empuje apunta segun a + g*e_z, con a = dv/dt suavizada.
% Convencion de Nquadrotor: Rot = Rz(psi)*Ry(theta)*Rx(phi), z hacia arriba.
g = 9.81;
N = numel(t);
dtm = median(diff(t));
if ~(dtm > 0), dtm = 0.02; end
w = max(1, round(0.1 / dtm));                   % semiventana ~0.1 s
i0 = max(1, (1:N) - w);
i1 = min(N, (1:N) + w);
dT = t(i1) - t(i0);
dT(dT <= 0) = NaN;
a = (v(:, i1) - v(:, i0)) ./ dT;
a(:, ~isfinite(dT)) = 0;
a(~isfinite(a)) = 0;

f = a + [0; 0; g];
c = cos(psi); s = sin(psi);
fb1 =  c .* f(1,:) + s .* f(2,:);               % f en el marco de yaw: Rz(-psi)*f
fb2 = -s .* f(1,:) + c .* f(2,:);
fb3 = f(3,:);
theta = atan2(fb1, fb3);
phi   = atan2(-fb2, sqrt(fb1.^2 + fb3.^2));
rp = [phi; theta];
end
