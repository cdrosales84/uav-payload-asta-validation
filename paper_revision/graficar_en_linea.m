function graficar_en_linea(op, varargin)
% GRAFICAR_EN_LINEA  Vista 3D del UAV, la cuerda y la carga dentro del lazo.
%
%   graficar_en_linea('iniciar', cfg)
%   graficar_en_linea('paso', m, p_d, t)
%
% Misma escena que el bloque "Graficar en Linea" de
% simulations/CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m:
% trayectoria de referencia, Nquadrotor con roll/pitch/yaw, cuerda, cubo de la
% carga y estelas. Se activa con cfg.planta.grafico_en_linea = true.
% cfg.planta.grafico_cada indica cada cuantas muestras se redibuja.

persistent V
switch op
    case 'iniciar'
        V = crear(varargin{1});
    case 'paso'
        if isempty(V) || ~ishandle(V.fig), return; end
        V.n = V.n + 1;
        if mod(V.n - 1, V.cada) ~= 0, return; end
        V = dibujar(V, varargin{1}, varargin{2}(:), varargin{3});
    otherwise
        error('graficar_en_linea:op', 'Operacion desconocida: %s', op);
end
end

function V = crear(cfg)
tt = 0:cfg.planta.dt:cfg.planta.T;
rd = referencia_carga(tt, cfg.ref);
r0 = referencia_carga(0, cfg.ref);

V.cada = 5;
if isfield(cfg.planta, 'grafico_cada') && cfg.planta.grafico_cada >= 1
    V.cada = max(1, round(cfg.planta.grafico_cada));
end
V.n = 0;
V.tam = cfg.planta.tam_carga;
V.hayCarga = cfg.planta.mL >= 0.001;
V.colorFrente = [1 0 1];
V.colorquad = get(groot, 'defaultAxesColorOrder');
V.colorquad = V.colorquad(1, :);
V.colorBox = [0.38 0.1 0.035];
V.colorCuerda = [0.5430 0.2695 0.0742];
V.H1 = gobjects(0);
V.H2 = gobjects(0);
V.H3 = gobjects(0);
V.H4 = gobjects(0);

V.fig = figure('Name', 'Evolucion UAV-carga', 'Color', 'w', 'NumberTitle', 'off');
s = get(0, 'ScreenSize');
V.fig.Position = [s(3)*0.125 s(4)*0.125 s(3)*0.75 s(4)*0.75];
V.ax = axes('Parent', V.fig);
hold(V.ax, 'on');
grid(V.ax, 'on');
box(V.ax, 'on');
xlabel(V.ax, 'x [m]');
ylabel(V.ax, 'y [m]');
zlabel(V.ax, 'z [m]');
plot3(V.ax, rd.p(1,:), rd.p(2,:), rd.p(3,:), '--r');
view(V.ax, 45, 20);

margen = 1.5;
l = cfg.planta.l;
xlim(V.ax, [min(rd.p(1,:))-margen, max(rd.p(1,:))+margen]);
ylim(V.ax, [min(rd.p(2,:))-margen, max(rd.p(2,:))+margen]);
zlim(V.ax, [max(0, min(rd.p(3,:))-margen), max(rd.p(3,:))+l+margen]);
daspect(V.ax, [1 1 1]);
axis(V.ax, 'manual');

Nquadrotor_parametros(1, 0.573, V.colorFrente, V.colorquad);
vent = max(2, round(6/(cfg.planta.dt*V.cada)));   % 6 s de estela
if V.hayCarga
    V.H6 = animatedline(V.ax, 'MaximumNumPoints', vent, 'Color', V.colorBox, 'LineWidth', 1.5);
end
V.H5 = animatedline(V.ax, 'MaximumNumPoints', vent, 'Color', V.colorquad, 'LineWidth', 1.5);
V.H7 = animatedline(V.ax, 'MaximumNumPoints', 1, 'LineWidth', 2, 'Color', [1 0 0], ...
    'Marker', 's', 'MarkerSize', 5);

m0.x = r0.p + [0; 0; l];
m0.xl = r0.p;
m0.psi = r0.psi;
m0.att = [0; 0; r0.psi];
m0.rope = [];
V = dibujar(V, m0, r0.p, 0);
end

function V = dibujar(V, m, pd, t)
if ~ishandle(V.fig) || ~ishandle(V.ax), return; end
set(0, 'CurrentFigure', V.fig);
set(V.fig, 'CurrentAxes', V.ax);

borrar(V.H1);
x = m.x(:);
att = [0; 0; m.psi];
if isfield(m, 'att') && numel(m.att) >= 3, att = m.att(:); end
V.H1 = Nquadrotor(1, [x(1), x(2), x(3), att(1), att(2), att(3)]);

r = cuerda_de(m, x);
if V.hayCarga && size(r, 1) >= 2
    borrar(V.H2); borrar(V.H3); borrar(V.H4);
    V.H2 = line(V.ax, r(:,1), r(:,2), r(:,3)+0.0025, ...
        'Color', V.colorCuerda, 'LineWidth', 1.5);
    V.H3 = line(V.ax, [r(1,1) x(1)], [r(1,2) x(2)], [r(1,3) x(3)], ...
        'Color', V.colorquad, 'LineWidth', 2);

    dirBox = r(end-1,:) - r(end,:);
    dirBox = dirBox / max(norm(dirBox), 1e-12);
    tc = V.tam;
    pc = r(end,:);
    if r(end,3) < tc/2
        origen = [pc(1), pc(2), tc/2];
    else
        origen = [pc(1), pc(2), pc(3) - tc/2];
    end
    V.H4 = cube_plot(origen, tc, tc, tc, V.colorBox);
    rotate(V.H4, [1 0 0], rad2deg(-asin(max(min(dirBox(2), 1), -1))), pc);
    rotate(V.H4, [0 1 0], rad2deg(asin(max(min(dirBox(1), 1), -1))), pc);
    addpoints(V.H6, r(end,1), r(end,2), r(end,3));
end
addpoints(V.H5, x(1), x(2), x(3));
addpoints(V.H7, pd(1), pd(2), pd(3));
title(V.ax, sprintf('t = %.2f s', t));
drawnow limitrate;
end

function r = cuerda_de(m, x)
if isfield(m, 'rope') && size(m.rope, 1) >= 2 && size(m.rope, 2) == 3
    n = size(m.rope, 1);
    idx = 1:3:n;                              % igual que yRope(1:3:40,:,k)
    if idx(end) ~= n, idx = [idx, n]; end
    r = m.rope(idx, :);
else
    s = linspace(0, 1, 8).';
    r = (1 - s) * x.' + s * m.xl(:).';
end
end

function borrar(h)
if ~isempty(h), delete(h(isgraphics(h))); end
end
