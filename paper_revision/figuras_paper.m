function figuras_paper(dir_res, varargin)
% FIGURAS_PAPER  Figuras del manuscrito a partir de los resultados de run_paper_experiments.
%
%   figuras_paper('results/20261010_120000')
%   figuras_paper(dir_res, 'formato', {'eps','pdf'}, 'ancho_mm', 160, 'banda', true)
%
% Lee results/<fecha>/raw/<escenario>_rep<k>.mat y escribe en results/<fecha>/paper_figs/
% un archivo por figura, con el MISMO nombre que usa \includegraphics en SN_Payload.tex,
% de modo que basta copiar la carpeta a Paper/Figures.
%
% Cada serie temporal se dibuja como la media de las repeticiones (interpoladas a una grilla
% comun, porque cada corrida tiene su propio tiempo de simulador) y, si 'banda' = true,
% con una banda minimo-maximo entre repeticiones.
%
% Opciones:
%   'formato'   {'eps','pdf'} (default). Vectorial. 'png' tambien disponible.
%   'ancho_mm'  ancho de las figuras de ancho completo. Conviene igualarlo al \textwidth del
%               manuscrito (en LaTeX: \the\textwidth, 1 pt = 0.3515 mm) y luego incluir con
%               \includegraphics[width=\linewidth]{...}: asi la letra queda en su tamano real.
%   'fuente_pt' tamano de letra (default 8).
%   'banda'     dibujar la banda min-max entre repeticiones (default true).
%   'salida'    carpeta de salida (default <dir_res>/paper_figs).

op = struct('formato', {{'eps','pdf'}}, 'ancho_mm', 160, 'fuente_pt', 8, 'banda', true, 'salida', '');
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end
if ischar(op.formato), op.formato = {op.formato}; end
if isempty(op.salida), op.salida = fullfile(dir_res, 'paper_figs'); end
if ~exist(op.salida, 'dir'), mkdir(op.salida); end
global FP; FP = op;  %#ok<GVMIS> estilo compartido por las funciones locales

% Colores fijos por controlador (paleta categorica validada; el estilo de linea es la
% codificacion secundaria para impresion en grises y para daltonismo).
C.ref  = struct('c', [0.10 0.10 0.10], 'ls', '--', 'nom', 'Reference');
C.kin  = struct('c', hex('#eb6834'), 'ls', '-',  'nom', 'Kinematic controller');
C.prop = struct('c', hex('#2a78d6'), 'ls', '-',  'nom', 'Proposed (kinematic + A-STA)');
C.smc  = struct('c', hex('#1baf7a'), 'ls', '-',  'nom', 'Boundary-layer SMC (ablation)');  % linea continua: el punteado se pierde sobre el chattering
C.fijo = struct('c', hex('#eda100'), 'ls', ':',  'nom', 'STA, fixed $\hat{\theta}$');
C.uav  = struct('c', [0.45 0.45 0.45], 'ls', '-', 'nom', 'UAV');
masas  = [50 100 200 300];
C.masa = [hex('#86b6ef'); hex('#3987e5'); hex('#1c5cab'); hex('#0d366b')];  % rampa ordinal

E1 = cargar(dir_res, 'E1_circ_sin_swing');
E2 = cargar(dir_res, 'E2_circ_cinematico');
E4 = cargar(dir_res, 'E4_circ_asta');
E5 = cargar(dir_res, 'E5_lemn_viento');
E6 = cargar(dir_res, 'E6_circ_smc');

W = op.ancho_mm;

%% Fig. 6: vistas 3D, ley sin compensacion (a) y controlador cinematico (b)
if ~isempty(E1), vista3d(E1(1), [], 10, C, 'a', 0.48*W, 0.42*W, '3D_Vista_Base'); end
if ~isempty(E2), vista3d(E2(1), [], 10, C, 'b', 0.48*W, 0.42*W, '3D_Vista_CinematicoII'); end

%% Figs. 7-8: controlador cinematico solo
if ~isempty(E2)
    fig_posicion({E2}, {C.kin}, C, W, 'CinematicoII');
    fig_oscilacion({E2}, {C.kin}, W, 'CinematicoII_Angles', []);
end

%% Figs. 9-12: cascada completa, comparada con el cinematico solo
if ~isempty(E4)
    fig_posicion({E4}, {C.prop}, C, W, 'Compensador');
    if isempty(E2), S = {E4}; K = {C.prop}; else, S = {E2, E4}; K = {C.kin, C.prop}; end
    fig_oscilacion(S, K, W, 'Compensador_Angles', []);
    fig_errores(S, K, W, 'Compensador_ErroresControl', []);
    fig_errvel(S, K, W, 'Compensador_ErroresVelocidad', []);
    fig_estimador(E4, C.prop, W, 'Estimador_Validacion');
end

%% Figs. 13-16: lemniscata con viento
if ~isempty(E5)
    B = E5(1).cfg.viento.bloques;
    tv = [10, mean(B(1,1:2)), mean(B(min(2,end),1:2))];
    vista3d(E5(1), tv(1), [], C, '', 0.6*W, 0.42*W, '3D_Vista_10Sec');
    vista3d(E5(1), tv(2), [], C, '', 0.6*W, 0.42*W, '3D_Vista_VientoLateral');
    vista3d(E5(1), tv(3), [], C, '', 0.6*W, 0.42*W, '3D_Vista_Viento45');
    fig_posicion({E5}, {C.prop}, C, W, 'Viento_ErroresControl', B);
    fig_oscilacion({E5}, {C.prop}, W, 'Viento_Angles', B);
    fig_errvel({E5}, {C.prop}, W, 'Viento_ErroresVelocidad', B);
end

%% Figs. 17-18: ablacion del lazo interno
if ~isempty(E4) && ~isempty(E6)
    fig_oscilacion({E6, E4}, {C.smc, C.prop}, W, 'Comp_Baseline_Oscilaciones', []);
    fig_correccion({E6, E4}, {C.smc, C.prop}, W, 'Comp_Baseline_Chattering', []);
    fig_correccion({E6, E4}, {C.smc, C.prop}, W, 'Comp_Baseline_Chattering_zoom', [40 50]);
end

%% Figura nueva: barrido de masa (adaptacion)
EA = {}; EF = {};
for i = 1:numel(masas)
    EA{i} = cargar(dir_res, sprintf('E7_masa_%d_asta', masas(i))); %#ok<AGROW>
    EF{i} = cargar(dir_res, sprintf('E7_masa_%d_fijo', masas(i))); %#ok<AGROW>
end
if all(~cellfun(@isempty, EA)), fig_masa(EA, EF, masas, C, W, 'Masa_Adaptacion'); end

fprintf('Figuras en %s\n', op.salida);
clear global FP
end

%% ======================================================================
%% Figuras
%% ======================================================================
function fig_posicion(S, K, C, W, nombre, B)
if nargin < 6, B = []; end
[f, ax] = panel(4, W, 0.62*W);
lab = {'$x_\ell$ [m]', '$y_\ell$ [m]', '$z_\ell$ [m]', '$\psi$ [rad]'};
G = grilla(S{1});
for i = 1:4
    axes(ax(i)); hold on; sombrear_viento(B);
    if i < 4
        h(1) = plot(G.t, interp_rep(S{1}(1), 'xd', i, G.t), C.ref.ls, 'Color', C.ref.c, 'LineWidth', 1.0);
        for k = 1:numel(S), h(k+1) = serie(S{k}, 'xl', i, G.t, K{k}); end
    else
        h(1) = plot(G.t, sin_saltos(interp_rep(S{1}(1), 'psid', 1, G.t)), C.ref.ls, 'Color', C.ref.c, 'LineWidth', 1.0);
        for k = 1:numel(S), h(k+1) = serie(S{k}, 'psi', 1, G.t, K{k}, @envolver); end
    end
    etiqueta_y(lab{i}); acabado(ax(i), i == 4, G.t);
end
leyenda(ax(1), h, [{C.ref.nom}, cellfun(@(k) k.nom, K, 'UniformOutput', false)]);
exportar(f, nombre); %#ok<NASGU>
end

function fig_oscilacion(S, K, W, nombre, B)
[f, ax] = panel(3, W, 0.5*W);
G = grilla(S{1});
campos = {'alpha', 'rho', 'vperp'};
lab = {'$\alpha$ [rad]', '$\rho$ [m]', '$\|\mathbf{v}_\perp\|$ [m/s]'};
for i = 1:3
    axes(ax(i)); hold on; sombrear_viento(B);
    for k = 1:numel(S), h(k) = serie(S{k}, campos{i}, 1, G.t, K{k}); end %#ok<AGROW>
    etiqueta_y(lab{i}); acabado(ax(i), i == 3, G.t);
end
if numel(S) > 1, leyenda(ax(1), h, cellfun(@(k) k.nom, K, 'UniformOutput', false)); end
exportar(f, nombre);
end

function fig_errores(S, K, W, nombre, B)
[f, ax] = panel(4, W, 0.62*W);
G = grilla(S{1});
lab = {'$\tilde{x}$ [m]', '$\tilde{y}$ [m]', '$\tilde{z}$ [m]', '$e_\psi$ [rad]'};
for i = 1:4
    axes(ax(i)); hold on; sombrear_viento(B);
    for k = 1:numel(S)
        if i < 4, h(k) = serie(S{k}, 'ex_verdad', i, G.t, K{k});  %#ok<AGROW>
        else,     h(k) = serie(S{k}, 'epsi', 1, G.t, K{k}); end    %#ok<AGROW>
    end
    linea_cero(G.t); etiqueta_y(lab{i}); acabado(ax(i), i == 4, G.t);
end
if numel(S) > 1, leyenda(ax(1), h, cellfun(@(k) k.nom, K, 'UniformOutput', false)); end
exportar(f, nombre);
end

function fig_errvel(S, K, W, nombre, B)
[f, ax] = panel(2, W, 0.36*W);
G = grilla(S{1});
lab = {'$\tilde{v}_x$ [m/s]', '$\tilde{v}_y$ [m/s]'};
for i = 1:2
    axes(ax(i)); hold on; sombrear_viento(B);
    for k = 1:numel(S), h(k) = serie(S{k}, 'vt', i, G.t, K{k}); end %#ok<AGROW>
    linea_cero(G.t); etiqueta_y(lab{i}); acabado(ax(i), i == 2, G.t);
end
if numel(S) > 1, leyenda(ax(1), h, cellfun(@(k) k.nom, K, 'UniformOutput', false)); end
exportar(f, nombre);
end

function fig_correccion(S, K, W, nombre, ventana)
% correccion del lazo interno: comando enviado menos comando cinematico, canales x,y
[f, ax] = panel(2, W, 0.36*W);
G = grilla(S{1});
lab = {'$u_x - v_{d,x}$ [m/s]', '$u_y - v_{d,y}$ [m/s]'};
for i = 1:2
    axes(ax(i)); hold on;
    for k = 1:numel(S), h(k) = serie(S{k}, 'correccion', i, G.t, K{k}); end %#ok<AGROW>
    etiqueta_y(lab{i}); acabado(ax(i), i == 2, G.t);
    if ~isempty(ventana), xlim(ventana); end
end
leyenda(ax(1), h, cellfun(@(k) k.nom, K, 'UniformOutput', false));
exportar(f, nombre);
end

function fig_estimador(S, K, W, nombre)
% validacion del estimador de hdot contra la verdad del simulador (repeticion 1, ventana de 10 s)
r = S(1); t = r.t; v = [20 30];
[f, ax] = panel(2, W, 0.36*W);
lab = {'$\dot h_x$ [m/s]', '$\dot h_y$ [m/s]'};
for i = 1:2
    axes(ax(i)); hold on;
    h(1) = plot(t, r.hdot(i,:), '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.0); %#ok<AGROW>
    h(2) = plot(t, r.hp(i,:), '--', 'Color', K.c, 'LineWidth', 1.0); %#ok<AGROW>
    etiqueta_y(lab{i}); acabado(ax(i), i == 2, t); xlim(v);
end
leyenda(ax(1), h, {'Simulator ground truth', 'Estimate $\hat{\mathbf{h}}_p$'});
exportar(f, nombre);
end

function fig_masa(EA, EF, masas, C, W, nombre)
[f, ax] = panel(2, W, 0.42*W);
lab = {'$\hat\theta_{1,x}=1/\hat\kappa_{u,x}$', '$\hat\theta_{2,x}=\hat\kappa_{v,x}/\hat\kappa_{u,x}$'};
fil = [1 2];   % filas de R.theta: [th1_x th2_x th1_y th2_y]
for i = 1:2
    axes(ax(i)); hold on;
    for m = 1:numel(masas)
        G = grilla(EA{m});
        kk = struct('c', C.masa(m,:), 'ls', '-', 'nom', sprintf('$m_L = %d$ g', masas(m)));
        h(m) = serie(EA{m}, 'theta', fil(i), G.t, kk); %#ok<AGROW>
    end
    etiqueta_y(lab{i}); acabado(ax(i), i == 2, G.t);
end
leyenda(ax(1), h, arrayfun(@(m) sprintf('$m_L = %d$ g', m), masas, 'UniformOutput', false));
exportar(f, nombre);
% texto para el pie de figura: RMS de nu_x fijo vs adaptativo
for m = 1:numel(masas)
    if isempty(EF{m}), continue; end
    a = mean(arrayfun(@(r) rms1(r.nu(1, r.t >= r.cfg.met.t_inicio)), EA{m}));
    b = mean(arrayfun(@(r) rms1(r.nu(1, r.t >= r.cfg.met.t_inicio)), EF{m}));
    fprintf('  m_L = %3d g: RMS nu_x fijo %.3f -> adaptativo %.3f\n', masas(m), b, a);
end
end

function vista3d(r, t_inst, cada_s, C, letra, w, h, nombre)
% Trayectoria 3D: referencia, carga y UAV, con el cable dibujado cada 'cada_s' segundos
% o en el instante t_inst (cubo de 5 m centrado en el cable, +-2 s de trayectoria y flecha de viento).
f = nueva_fig(w, h);
ax = axes(f, 'Units', 'centimeters', 'Position', [1.2 1.0 w/10-1.8 h/10-2.0]); hold(ax, 'on');
t = r.t;
if isempty(t_inst)
    sel = true(size(t)); idx = unique([1, find(diff(floor(t/cada_s)) > 0) + 1]);
else
    sel = abs(t - t_inst) <= 2; [~, idx] = min(abs(t - t_inst));
end
h1 = plot3(r.xd(1,sel), r.xd(2,sel), r.xd(3,sel), C.ref.ls, 'Color', C.ref.c, 'LineWidth', 0.9);
h2 = plot3(r.xl(1,sel), r.xl(2,sel), r.xl(3,sel), '-', 'Color', C.prop.c, 'LineWidth', 1.1);
h3 = plot3(r.x(1,sel),  r.x(2,sel),  r.x(3,sel),  '-', 'Color', C.uav.c,  'LineWidth', 0.8);
for k = idx
    plot3([r.x(1,k) r.xl(1,k)], [r.x(2,k) r.xl(2,k)], [r.x(3,k) r.xl(3,k)], '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 0.6);
    plot3(r.xl(1,k), r.xl(2,k), r.xl(3,k), 's', 'MarkerSize', 4, 'MarkerFaceColor', C.prop.c, 'MarkerEdgeColor', 'none');
    plot3(r.x(1,k),  r.x(2,k),  r.x(3,k),  'o', 'MarkerSize', 4, 'MarkerFaceColor', C.uav.c,  'MarkerEdgeColor', 'none');
end
if ~isempty(t_inst) && norm(r.viento(:,idx)) > 0
    w = r.viento(:,idx); p0 = r.x(:,idx) + [0;0;0.8]; e = 1.2*w/norm(w);
    plot3(p0(1)+[-e(1) 0], p0(2)+[-e(2) 0], p0(3)*[1 1], 'k-', 'LineWidth', 1.4);      % flecha de viento
    plot3(p0(1), p0(2), p0(3), 'k>', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
end
grid(ax, 'on'); view(ax, 45, 20);
if isempty(t_inst)
    P = [r.xd(:,sel), r.xl(:,sel), r.x(:,sel)]; pad = 0.3;    % limites ajustados a lo dibujado
    lim = [min(P,[],2) - pad, max(P,[],2) + pad].';
else
    c = (r.x(:,idx) + r.xl(:,idx))/2;                         % cubo de +-2.5 m centrado en el cable
    lim = [c - 2.5, c + 2.5].';
end
set(ax, 'XLim', lim(:,1), 'YLim', lim(:,2), 'ZLim', lim(:,3), 'PlotBoxAspectRatio', diff(lim));  % misma escala en x, y, z
xlabel(ax, txt('$x$ [m]')); ylabel(ax, txt('$y$ [m]')); zlabel(ax, txt('$z$ [m]'));
estilo_ejes(ax);
leyenda(ax, [h1 h2 h3], {'Reference', 'Payload', 'UAV'});
exportar(f, nombre);
end

%% ======================================================================
%% Datos
%% ======================================================================
function S = cargar(dir_res, esc)
% Carga todas las repeticiones de un escenario y agrega las senales derivadas.
d = dir(fullfile(dir_res, 'raw', [esc '_rep*.mat']));
S = [];
for i = 1:numel(d)
    D = load(fullfile(dir_res, 'raw', d(i).name));
    r = D.R; M = D.M;
    r.alpha = M.serie.alpha; r.rho = M.serie.rho; r.vperp = M.serie.vperp;
    r.hdot = M.serie.hdot; r.ex_verdad = r.xd - r.xl;
    r.correccion = r.u(1:2,:) - r.ukh(1:2,:);
    if isempty(S), S = r; else, S(end+1) = r; end %#ok<AGROW>
end
if isempty(S), fprintf('  (sin resultados de %s)\n', esc); end
end

function G = grilla(S)
% grilla comun para todas las repeticiones (cada una tiene su propio reloj de simulador)
tf = min(arrayfun(@(r) r.t(end), S));
G.t = 0:S(1).cfg.planta.dt:tf;
end

function Y = interp_rep(S, campo, fila, tg, fn)
% matriz (repeticiones x tiempo) de la senal interpolada a la grilla tg
if nargin < 5, fn = []; end
Y = nan(numel(S), numel(tg));
for k = 1:numel(S)
    y = S(k).(campo)(fila, :);
    if ~isempty(fn), y = fn(y); end
    Y(k,:) = interp1(S(k).t, y, tg, 'linear', NaN);
end
if size(Y,1) == 1, return; end
end

function h = serie(S, campo, fila, tg, K, fn)
% media de las repeticiones y banda min-max (color aclarado, sin transparencia: compatible con EPS)
global FP %#ok<GVMIS>
if nargin < 6, fn = []; end
Y = interp_rep(S, campo, fila, tg, fn);
m = mean(Y, 1);
if FP.banda && size(Y,1) > 1
    lo = min(Y, [], 1); hi = max(Y, [], 1); ok = ~isnan(lo);
    cb = 0.25*K.c + 0.75*[1 1 1];
    fill([tg(ok) fliplr(tg(ok))], [lo(ok) fliplr(hi(ok))], cb, 'EdgeColor', 'none');
end
if ~isempty(fn), m = sin_saltos(m); end
h = plot(tg, m, K.ls, 'Color', K.c, 'LineWidth', 1.0);
end

%% ======================================================================
%% Estilo
%% ======================================================================
function f = nueva_fig(w_mm, h_mm)
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2 2 w_mm/10 h_mm/10], 'PaperUnits', 'centimeters', ...
    'PaperSize', [w_mm/10 h_mm/10], 'PaperPosition', [0 0 w_mm/10 h_mm/10]);
end

function [f, ax] = panel(n, w_mm, h_mm)
% n ejes apilados que comparten el eje de tiempo, con margenes fijos en mm
f = nueva_fig(w_mm, h_mm);
izq = 16; der = 4; arr = 9; aba = 10; sep = 2.5;           % mm
alto = (h_mm - arr - aba - (n-1)*sep)/n;
for i = 1:n
    y0 = h_mm - arr - i*alto - (i-1)*sep;
    ax(i) = axes(f, 'Units', 'centimeters', 'Position', [izq y0 (w_mm-izq-der) alto]/10); %#ok<AGROW>
end
end

function acabado(ax, ultimo, t)
estilo_ejes(ax);
xlim(ax, [t(1) t(end)]);
if ultimo, xlabel(ax, txt('Time [s]')); else, set(ax, 'XTickLabel', []); end
end

function estilo_ejes(ax)
global FP %#ok<GVMIS>
set(ax, 'FontSize', FP.fuente_pt, 'LineWidth', 0.5, 'Box', 'off', 'TickDir', 'out', ...
    'XColor', [0.25 0.25 0.25], 'YColor', [0.25 0.25 0.25], 'GridColor', [0.85 0.85 0.85], ...
    'GridAlpha', 1, 'Layer', 'top');
grid(ax, 'on');
if ~es_octave, set(ax, 'TickLabelInterpreter', 'latex'); end
end

function etiqueta_y(s), ylabel(txt(s)); end

function leyenda(ax, h, nombres)
% Leyenda en una franja propia en el borde superior de la figura, para que no mueva ni
% achique los ejes (asi todos los paneles quedan alineados).
global FP %#ok<GVMIS>
nombres = cellfun(@txt_simple, nombres, 'UniformOutput', false);
f = ancestor(ax, 'figure'); fp = get(f, 'Position');          % cm
la = axes(f, 'Units', 'centimeters', 'Position', [0 fp(4)-0.75 fp(3) 0.7], 'Visible', 'off');
hold(la, 'on');
for i = 1:numel(h)
    d(i) = plot(la, NaN, NaN, 'LineStyle', get(h(i), 'LineStyle'), 'Color', get(h(i), 'Color'), ...
        'LineWidth', get(h(i), 'LineWidth'), 'Marker', get(h(i), 'Marker')); %#ok<AGROW>
end
lg = legend(la, d, nombres, 'Location', 'north', 'Orientation', 'horizontal');
set(lg, 'FontSize', FP.fuente_pt, 'Box', 'off');
if ~es_octave, set(lg, 'Interpreter', 'latex'); end
axes(ax);
end

function linea_cero(t), plot([t(1) t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5); end

function sombrear_viento(B)
% intervalos de viento como bandas grises de fondo
if isempty(B), return; end
yl = [-1e3 1e3];
for i = 1:size(B,1)
    fill([B(i,1) B(i,2) B(i,2) B(i,1)], yl([1 1 2 2]), [0.93 0.93 0.93], 'EdgeColor', 'none');
end
set(gca, 'YLimMode', 'auto');
end

function exportar(f, nombre)
global FP %#ok<GVMIS>
arch = fullfile(FP.salida, nombre);
ajustar_ylim(f);
set(f, 'Units', 'centimeters'); pos = get(f, 'Position');
set(f, 'PaperUnits', 'centimeters', 'PaperSize', pos(3:4), 'PaperPosition', [0 0 pos(3:4)]);
for i = 1:numel(FP.formato)
    switch FP.formato{i}
        case 'eps', print(f, [arch '.eps'], '-depsc', '-painters');
        case 'pdf', print(f, [arch '.pdf'], '-dpdf', '-painters');
        case 'png', print(f, [arch '.png'], '-dpng', '-r300');
    end
end
if ~es_octave, savefig(f, [arch '.fig']); end
close(f);
fprintf('  %s\n', nombre);
end

function ajustar_ylim(f)
% los parches de viento se dibujan con altura grande; se recalcula el ylim con las lineas
ax = findobj(f, 'Type', 'axes');
for i = 1:numel(ax)
    L = findobj(ax(i), 'Type', 'line');
    y = [];
    for j = 1:numel(L), yy = get(L(j), 'YData'); y = [y yy(:).']; end %#ok<AGROW>
    P = findobj(ax(i), 'Type', 'patch');
    for j = 1:numel(P)
        yy = get(P(j), 'YData'); if max(abs(yy(:))) < 1e2, y = [y yy(:).']; end %#ok<AGROW>
    end
    y = y(isfinite(y));
    if isempty(y) || strcmp(get(ax(i), 'Tag'), 'legend'), continue; end
    if ~isequal(round(get(ax(i), 'View')), [0 90]), continue; end   % ejes 3D: no tocar
    lo = min(y); hi = max(y); m = 0.08*max(hi - lo, eps);
    set(ax(i), 'YLim', [lo - m, hi + m]);
end
end

%% ======================================================================
%% Utilidades
%% ======================================================================
function s = txt(s)
% MATLAB interpreta LaTeX; Octave no, asi que alli se simplifica la etiqueta (solo vista previa)
if es_octave, s = txt_simple(s); end
end

function s = txt_simple(s)
if ~es_octave, return; end
rep = {'\|', '||'; '\mathbf', ''; '\hat', ''; '\tilde', ''; '\dot', 'd'; '\ell', 'l'; ...
       '\perp', '\perp'; '$', ''; '{', ''; '}', ''};
for i = 1:size(rep,1), s = strrep(s, rep{i,1}, rep{i,2}); end
end

function b = es_octave(), b = exist('OCTAVE_VERSION', 'builtin') > 0; end
function c = hex(s), c = sscanf(s(2:end), '%2x%2x%2x').'/255; end
function a = envolver(a), a = atan2(sin(a), cos(a)); end
function r = rms1(x), r = sqrt(mean(x.^2)); end
function y = sin_saltos(y)
% inserta NaN donde un angulo envuelto salta 2*pi, para no dibujar lineas verticales
j = find(abs(diff(y)) > pi); y(j+1) = NaN;
end
