function figuras_omar(dir_res, varargin)
% FIGURAS_OMAR  Manuscript figures for the comparison with Omar (2022).
%
%   figuras_omar('results/comp_omar_20261015_022307')
%   figuras_omar(dir_res, 'ancho_mm', 160, 'formato', {'eps','pdf'})
%
% Reads <dir_res>/raw/*.mat (comparar_omar_vs_asta) and omar_ajuste.mat, and writes
% four figures in <dir_res>/paper_figs/ in the same style as figuras_paper.m:
%
%   Benchmark_Omar_Circulo   E4: ||x_tilde||, ||h_h - h*_h||, and alpha (3 panels)
%   Benchmark_Omar_Planta    E4: top view of the last lap (reference and payload)
%   Benchmark_Omar_Viento    E5: the same three panels as the circle, gusts shaded
%   Benchmark_Omar_Ajuste    baseline tuning: ISH vs t_s/T_L (hover and steps);
%                            in (b) only complete runs are drawn (the missing ones
%                            diverged; say so in the caption)
%
% Each time series is the mean of the complete repetitions, interpolated onto a
% common grid; the light band is the minimum and the maximum among them. The
% vertical axis is fitted with t >= t_ini (5 s), after the startup ramp.

op = struct('formato', {{'eps','pdf'}}, 'ancho_mm', 160, 'fuente_pt', 8, 'banda', true, ...
    'salida', '', 't_ini', 0, 'ajuste', '');
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end
if ischar(op.formato), op.formato = {op.formato}; end
if isempty(op.salida), op.salida = fullfile(dir_res, 'paper_figs'); end
if ~exist(op.salida, 'dir'), mkdir(op.salida); end
if isempty(op.ajuste), op.ajuste = fullfile(fileparts(mfilename('fullpath')), 'omar_ajuste.mat'); end
global FO; FO = op; %#ok<GVMIS>

C.ref  = struct('c', [0.10 0.10 0.10], 'ls', '--', 'nom', 'Reference');
C.px4  = struct('c', [0.45 0.45 0.45], 'ls', ':',  'nom', 'PX4-type');
C.omar = struct('c', [184 68 143]/255, 'ls', '-',  'nom', 'ASC [Omar 2022]');
C.asta = struct('c', [42 120 214]/255, 'ls', '-',  'nom', 'Proposed');
leyes = {'px4', 'omar', 'asta'};

raw = fullfile(dir_res, 'raw');
fprintf('Figuras en %s\n', op.salida);

%% E4: circle
S = cargar(raw, 'E4_circ_asta', leyes);
if ~isempty(S)
    tg = grilla(S);
    [f, ax] = panel(3, op.ancho_mm, 0.62*op.ancho_mm);
    var = {@(R) sqrt(sum((R.xd - R.xl).^2, 1)), '$\|\tilde{\mathbf{x}}\|$ [m]';
           @(R) sqrt(sum((R.h(1:2,:) - R.hs).^2, 1)), '$\|\mathbf{h}_h-\mathbf{h}_h^\star\|$ [m]';
           @(R) atan2(sqrt(R.h(1,:).^2 + R.h(2,:).^2), R.h(3,:)), '$\alpha$ [rad]'};
    for v = 1:3
        axes(ax(v)); hold on; %#ok<LAXES>
        h = gobj(3);
        for j = 1:3, h(j) = serie(S.(leyes{j}), var{v,1}, tg, C.(leyes{j})); end
        etq(@ylabel, var{v,2});
        acabado(ax(v), v == 3, tg);
        if v == 1, leyenda(ax(v), h, cellfun(@(l) C.(l).nom, leyes, 'UniformOutput', false)); end
    end
    exportar(f, 'Benchmark_Omar_Circulo');

    % top view of the last lap (repetition 1)
    f = nueva_fig(0.55*op.ancho_mm, 0.55*op.ancho_mm);
    ax = axes(f, 'Units', 'centimeters', 'Position', [1.4 1.2 0.55*op.ancho_mm/10-1.8 0.55*op.ancho_mm/10-2.7]);
    hold on;
    R = S.asta{1}; sel = R.t >= R.t(end) - R.cfg.ref.circ.Tp;
    h = gobj(4);
    h(1) = plot(R.xd(1,sel), R.xd(2,sel), C.ref.ls, 'Color', C.ref.c, 'LineWidth', 0.9);
    for j = 1:3
        R = S.(leyes{j}){1}; sel = R.t >= R.t(end) - R.cfg.ref.circ.Tp;
        h(j+1) = plot(R.xl(1,sel), R.xl(2,sel), C.(leyes{j}).ls, 'Color', C.(leyes{j}).c, 'LineWidth', 1.1);
    end
    axis equal; estilo_ejes(ax);
    etq(@xlabel, '$x$ [m]'); etq(@ylabel, '$y$ [m]');
    leyenda(ax, h, {C.ref.nom, C.px4.nom, C.omar.nom, C.asta.nom}, 2);
    exportar(f, 'Benchmark_Omar_Planta', false);
end

%% E5: lemniscate with wind
S = cargar(raw, 'E5_lemn_viento', leyes);
if ~isempty(S)
    tg = grilla(S);
    B = S.asta{1}.cfg.viento.bloques;
    [f, ax] = panel(3, op.ancho_mm, 0.62*op.ancho_mm);
    var = {@(R) sqrt(sum((R.xd - R.xl).^2, 1)), '$\|\tilde{\mathbf{x}}\|$ [m]';
           @(R) sqrt(sum((R.h(1:2,:) - R.hs).^2, 1)), '$\|\mathbf{h}_h-\mathbf{h}_h^\star\|$ [m]';
           @(R) atan2(sqrt(R.h(1,:).^2 + R.h(2,:).^2), R.h(3,:)), '$\alpha$ [rad]'};
    for v = 1:3
        axes(ax(v)); hold on; %#ok<LAXES>
        sombrear(B);
        h = gobj(3);
        for j = 1:3, h(j) = serie(S.(leyes{j}), var{v,1}, tg, C.(leyes{j})); end
        etq(@ylabel, var{v,2});
        acabado(ax(v), v == 3, tg);
        if v == 1, leyenda(ax(v), h, cellfun(@(l) C.(l).nom, leyes, 'UniformOutput', false)); end
    end
    exportar(f, 'Benchmark_Omar_Viento');
end

%% Baseline tuning
if exist(op.ajuste, 'file')
    A = load(op.ajuste); A = A.ajuste;
    [f, ax] = panel_h(2, op.ancho_mm, 0.42*op.ancho_mm);
    % (a) hover
    axes(ax(1)); hold on;
    Hh = A.hover;
    plot(Hh.tau_TL, Hh.ish(1,:), '-o', 'Color', C.omar.c, 'MarkerFaceColor', C.omar.c, 'MarkerSize', 3, 'LineWidth', 1.0);
    plot([Hh.tau_TL(1) Hh.tau_TL(end)], Hh.sin_asc*[1 1], C.px4.ls, 'Color', C.px4.c, 'LineWidth', 1.0);
    set(gca, 'YScale', 'log'); estilo_ejes(gca);
    etq(@xlabel, '$t_s/T_L$'); etq(@ylabel, 'ISH [rad$^2$s]');
    etq(@title, '(a) Hover, $K_s=0.6$');
    % (b) steps
    axes(ax(2)); hold on;
    G = A.grilla; K = G.K; T = G.tau_TL;
    rampa = [134 182 239; 57 135 229; 28 92 171; 13 54 107; 5 25 55]/255;
    h = gobj(numel(K) + 1);
    for i = 1:numel(K)
        c = rampa(min(i, size(rampa,1)), :);
        y = squeeze(G.ISH(1,i,:)).'; ok = squeeze(G.completo(1,i,:)).' > 0;
        h(i) = plot(T(ok), y(ok), '-o', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 3, 'LineWidth', 1.0);
    end
    h(end) = plot([T(1) T(end)], G.ISH_sin_asc*[1 1], C.px4.ls, 'Color', C.px4.c, 'LineWidth', 1.0);
    set(gca, 'YScale', 'log'); estilo_ejes(gca);
    etq(@xlabel, '$t_s/T_L$');
    etq(@title, '(b) Travel maneuver of [Omar 2022]');
    nom = [arrayfun(@(k) sprintf('$K_s=%.1f$', k), K, 'UniformOutput', false), {'No ASC'}];
    leyenda(ax(2), h, nom);
    exportar(f, 'Benchmark_Omar_Ajuste', false);
end
end

%% ======================================================================
%% Data
%% ======================================================================
function S = cargar(raw, esc, leyes)
S = struct();
for j = 1:numel(leyes)
    L = dir(fullfile(raw, sprintf('%s_%s_rep*.mat', esc, leyes{j})));
    L = L(cellfun(@isempty, strfind({L.name}, '_intento')));
    c = {};
    for i = 1:numel(L)
        d = load(fullfile(raw, L(i).name), 'R');
        if d.R.completo, c{end+1} = d.R; end %#ok<AGROW>
    end
    if isempty(c), S = []; fprintf('  %s: sin corridas completas de %s\n', esc, leyes{j}); return; end
    S.(leyes{j}) = c;
end
end

function tg = grilla(S)
f = fieldnames(S); Tf = inf;
for j = 1:numel(f), for k = 1:numel(S.(f{j})), Tf = min(Tf, S.(f{j}){k}.t(end)); end, end
tg = 0:0.02:Tf;
end

function h = serie(C, fn, tg, K)
global FO %#ok<GVMIS>
Y = nan(numel(C), numel(tg));
for k = 1:numel(C)
    R = C{k}; [tu, iu] = unique(R.t); y = fn(R);
    Y(k,:) = interp1(tu, y(iu), tg, 'linear', NaN);
end
m = mean(Y, 1);
if FO.banda && size(Y,1) > 1
    lo = min(Y, [], 1); hi = max(Y, [], 1); ok = ~isnan(lo);
    fill([tg(ok) fliplr(tg(ok))], [lo(ok) fliplr(hi(ok))], 0.25*K.c + 0.75*[1 1 1], ...
        'EdgeColor', 'none', 'HandleVisibility', 'off', 'Tag', 'banda');
end
h = plot(tg, m, K.ls, 'Color', K.c, 'LineWidth', 1.0);
end

%% ======================================================================
%% Style (same as figuras_paper.m)
%% ======================================================================
function f = nueva_fig(w_mm, h_mm)
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2 2 w_mm/10 h_mm/10], 'PaperUnits', 'centimeters', ...
    'PaperSize', [w_mm/10 h_mm/10], 'PaperPosition', [0 0 w_mm/10 h_mm/10]);
end

function [f, ax] = panel(n, w_mm, h_mm)
f = nueva_fig(w_mm, h_mm);
izq = 16; der = 4; arr = 9; aba = 10; sep = 2.5;
alto = (h_mm - arr - aba - (n-1)*sep)/n;
for i = 1:n
    y0 = h_mm - arr - i*alto - (i-1)*sep;
    ax(i) = axes(f, 'Units', 'centimeters', 'Position', [izq y0 (w_mm-izq-der) alto]/10); %#ok<AGROW>
end
end

function [f, ax] = panel_h(n, w_mm, h_mm)
f = nueva_fig(w_mm, h_mm);
izq = 15; der = 4; arr = 14; aba = 11; sep = 12;
ancho = (w_mm - izq - der - (n-1)*sep)/n;
for i = 1:n
    x0 = izq + (i-1)*(ancho + sep);
    ax(i) = axes(f, 'Units', 'centimeters', 'Position', [x0 aba ancho (h_mm-arr-aba)]/10); %#ok<AGROW>
end
end

function acabado(ax, ultimo, t)
global FO %#ok<GVMIS>
estilo_ejes(ax);
xlim(ax, [t(1) t(end)]);
% vertical axis from the data with t >= t_ini (after startup)
L = findobj(ax, 'Type', 'line'); y = [];
for j = 1:numel(L)
    xx = get(L(j), 'XData'); yy = get(L(j), 'YData');
    y = [y yy(xx >= FO.t_ini & isfinite(yy))]; %#ok<AGROW>
end
P = findobj(ax, 'Tag', 'banda');
for j = 1:numel(P)
    xx = get(P(j), 'XData'); yy = get(P(j), 'YData');
    y = [y yy(xx >= FO.t_ini & isfinite(yy)).']; %#ok<AGROW>
end
if ~isempty(y)
    lo = min(0, min(y)); hi = max(y);
    set(ax, 'YLim', [lo, hi + 0.08*(hi - lo)]);
end
if ultimo, axes(ax); etq(@xlabel, 'Time [s]'); else, set(ax, 'XTickLabel', []); end
end

function estilo_ejes(ax)
global FO %#ok<GVMIS>
set(ax, 'FontSize', FO.fuente_pt, 'LineWidth', 0.5, 'Box', 'off', 'TickDir', 'out', ...
    'XColor', [0.25 0.25 0.25], 'YColor', [0.25 0.25 0.25], 'GridColor', [0.85 0.85 0.85], ...
    'GridAlpha', 1, 'Layer', 'top');
grid(ax, 'on');
if ~es_octave, set(ax, 'TickLabelInterpreter', 'latex'); end
end

function leyenda(ax, h, nombres, ncol)
global FO %#ok<GVMIS>
if nargin < 4, ncol = numel(h); end
nombres = cellfun(@txt, nombres, 'UniformOutput', false);
f = ancestor(ax, 'figure'); fp = get(f, 'Position');
alto = 0.45 * ceil(numel(h)/ncol) + 0.25;
la = axes(f, 'Units', 'centimeters', 'Position', [0 fp(4)-alto fp(3) alto-0.05], 'Visible', 'off');
hold(la, 'on');
for i = 1:numel(h)
    d(i) = plot(la, NaN, NaN, 'LineStyle', get(h(i), 'LineStyle'), 'Color', get(h(i), 'Color'), ...
        'LineWidth', get(h(i), 'LineWidth'), 'Marker', get(h(i), 'Marker'), ...
        'MarkerFaceColor', get(h(i), 'MarkerFaceColor')); %#ok<AGROW>
end
lg = legend(la, d, nombres, 'Location', 'north', 'Orientation', 'horizontal');
set(lg, 'FontSize', FO.fuente_pt, 'Box', 'off');
if ~es_octave
    set(lg, 'Interpreter', 'latex');
    try, set(lg, 'NumColumns', ncol); catch, end %#ok<NOCOM>
end
axes(ax);
end

function sombrear(B)
for i = 1:size(B,1)
    fill([B(i,1) B(i,2) B(i,2) B(i,1)], [-1e3 -1e3 1e3 1e3], [0.93 0.93 0.93], ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end

function exportar(f, nombre, ~)
global FO %#ok<GVMIS>
arch = fullfile(FO.salida, nombre);
set(f, 'Units', 'centimeters'); pos = get(f, 'Position');
set(f, 'PaperUnits', 'centimeters', 'PaperSize', pos(3:4), 'PaperPosition', [0 0 pos(3:4)]);
for i = 1:numel(FO.formato)
    switch FO.formato{i}
        case 'eps', print(f, [arch '.eps'], '-depsc', '-painters');
        case 'pdf', print(f, [arch '.pdf'], '-dpdf', '-painters');
        case 'png', print(f, [arch '.png'], '-dpng', '-r300');
    end
end
if ~es_octave, savefig(f, [arch '.fig']); end
close(f);
fprintf('  %s\n', nombre);
end

function etq(fn, s)
% label with the LaTeX interpreter in MATLAB (in Octave, simplified text)
global FO %#ok<GVMIS>
if es_octave, fn(txt(s)); else, fn(s, 'Interpreter', 'latex', 'FontSize', FO.fuente_pt, 'FontWeight', 'normal'); end
end

function h = gobj(n)
if es_octave, h = zeros(1, n); else, h = gobjects(1, n); end
end

function s = txt(s)
if ~es_octave, return; end
rep = {'\|', '||'; '\mathbf', ''; '\tilde', ''; '\perp', '_perp'; '^\star', '*'; ...
       '$', ''; '{', ''; '}', ''; '\alpha', 'alpha'; '^2', '2'};
for i = 1:size(rep,1), s = strrep(s, rep{i,1}, rep{i,2}); end
end

function b = es_octave(), b = exist('OCTAVE_VERSION', 'builtin') > 0; end
