function export_uav_simulation_video(outFile, src, varargin)
% EXPORT_UAV_SIMULATION_VIDEO  Video MP4 con vista global + recuadro zoom (UAV, cuerda, carga).
%
% Pensado para los .mat de sim_payload_asta / run_paper_experiments:
%   R.t, R.x, R.xl, R.xd, R.psi, R.v, R.viento, R.cfg.planta
% La cuerda no se guarda: plot_uav_payload_into_axes la dibuja recta entre R.x y R.xl.
% El tiempo de cada frame es R.t(k) (muestreo no uniforme si usar_dt_real = true).
%
% Uso:
%   export_uav_simulation_video('E5.mp4', ...
%       'results/E5_lemn_viento/raw/E5_lemn_viento_rep1.mat')
%
%   S = load('results/E5_lemn_viento/raw/E5_lemn_viento_rep1.mat');
%   export_uav_simulation_video('E5.mp4', S)      % o S.R
%
% Opcional (Name-Value):
%   'WindHist'       — matriz 3 x N. Default: R.viento
%   'FrameStart'     — indice inicial k (default 1)
%   'FrameEnd'       — indice final k (default numel(R.t))
%   'TimeStart'      — instante [s]; si se da, reemplaza FrameStart (muestra mas cercana)
%   'TimeEnd'        — instante [s]; si se da, reemplaza FrameEnd
%   'Stride'         — 1 = todas las muestras; 2 = una de cada dos (default 2)
%   'LocalRadius'    — semilado del recuadro zoom [m] (default 1.2)
%   'FigureSize'     — [ancho alto] pixeles (default [1280 720])
%   'MainAxesPos'    — posicion axes principal [l b w h] (default [0.06 0.10 0.92 0.82])
%   'InsetAxesPos'   — posicion recuadro zoom (default [0.62 0.14 0.34 0.30])
%   'VideoFrameRate' — FPS del archivo. Default: tiempo real, 1/(dt_mediano*Stride)
%   'HeaderStr'      — titulo de la figura
%   'EstimarActitud' — inclinar el UAV segun su aceleracion (default true; R no guarda roll/pitch)
%   'Margen'         — margen [m] de la vista global (default 1.5)
%   'Limites'        — [xmin xmax ymin ymax zmin zmax] de la vista global (default: segun los datos)

p = inputParser;
addParameter(p, 'WindHist', []);
addParameter(p, 'FrameStart', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'FrameEnd', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'TimeStart', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'TimeEnd', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'Stride', 2, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'LocalRadius', 1.2, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'FigureSize', [1280 720], @(v) isnumeric(v) && numel(v) == 2);
addParameter(p, 'MainAxesPos', [0.06 0.10 0.92 0.82], @(v) isnumeric(v) && numel(v) == 4);
addParameter(p, 'InsetAxesPos', [0.62 0.14 0.34 0.30], @(v) isnumeric(v) && numel(v) == 4);
addParameter(p, 'VideoFrameRate', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
addParameter(p, 'HeaderStr', ...
    'Payload Transport with a Quadrotor UAV Using a Composite Adaptive Robust Control Architecture', ...
    @(s) ischar(s) || isstring(s));
addParameter(p, 'EstimarActitud', true, @(x) islogical(x) && isscalar(x));
addParameter(p, 'Margen', 1.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'Limites', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 6));
parse(p, varargin{:});

R = cargar_resultado(src);
op = p.Results;

N = numel(R.t);
k0 = round(op.FrameStart);
k1 = op.FrameEnd;
if isempty(k1), k1 = N; else, k1 = round(k1); end
if ~isempty(op.TimeStart), [~, k0] = min(abs(R.t - op.TimeStart)); end
if ~isempty(op.TimeEnd),   [~, k1] = min(abs(R.t - op.TimeEnd));   end
k0 = max(1, min(k0, N));
k1 = max(k0, min(k1, N));
stride = round(op.Stride);

dt = median(diff(R.t(:)));
if ~(dt > 0), dt = 0.02; end
if isempty(op.VideoFrameRate)
    vfps = 1 / (dt * stride);          % reproduccion en tiempo real
else
    vfps = op.VideoFrameRate;
end

argsPlot = {'EstimarActitud', op.EstimarActitud, 'Margen', op.Margen, 'LocalRadius', op.LocalRadius};
if ~isempty(op.WindHist)
    argsPlot = [argsPlot, {'WindHist', op.WindHist}];
end
argsGlobal = argsPlot;
if ~isempty(op.Limites)
    argsGlobal = [argsGlobal, {'Limites', op.Limites}];
end

figSz = op.FigureSize;
fig = figure('Color', 'w', 'Position', [80 80 figSz(1) figSz(2)], ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');
axMain = axes('Parent', fig, 'Position', op.MainAxesPos);
axIns  = axes('Parent', fig, 'Position', op.InsetAxesPos);

annotation(fig, 'textbox', [0.06 0.94 0.88 0.045], ...
    'String', char(op.HeaderStr), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left', ...
    'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none');

v = VideoWriter(outFile, 'MPEG-4');
v.FrameRate = vfps;
v.Quality = 92;
open(v);

idx = k0:stride:k1;
fprintf('Exportando %s: %d frames, %.2f fps (t = %.2f s .. %.2f s)\n', ...
    outFile, numel(idx), vfps, R.t(k0), R.t(k1));

try
    nEscritos = 0;
    for k = idx
        cla(axMain);
        cla(axIns);

        plot_uav_payload_into_axes(axMain, R, k, argsGlobal{:}, ...
            'AxesMode', 'global', ...
            'DrawWind', true, ...
            'TitleStr', sprintf('Vista global — t = %.2f s', R.t(k)));

        plot_uav_payload_into_axes(axIns, R, k, argsPlot{:}, ...
            'AxesMode', 'local', ...
            'DrawWind', false, ...
            'TitleStr', sprintf('Zoom — t = %.2f s', R.t(k)));

        set(axIns, 'LineWidth', 1.25, ...
            'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
        uistack(axIns, 'top');

        drawnow;
        writeVideo(v, getframe(fig));

        nEscritos = nEscritos + 1;
        if nEscritos == 1 || mod(nEscritos, 25) == 0 || nEscritos == numel(idx)
            fprintf('  %d / %d  (t = %.2f s)\n', nEscritos, numel(idx), R.t(k));
        end
    end
catch ME
    try %#ok<CTCH>
        close(v);
    end %#ok<CTCH>
    if ishandle(fig), close(fig); end
    rethrow(ME);
end

close(v);
close(fig);
fprintf('Video guardado: %s (%d frames, %.2f fps)\n', outFile, numel(idx), vfps);
end

function R = cargar_resultado(src)
% Acepta ruta .mat, S = load(archivo) o la estructura R.
if ischar(src) || isstring(src)
    archivo = char(src);
    if exist(archivo, 'file') ~= 2
        error('export_uav_simulation_video:archivo', 'No existe el archivo: %s', archivo);
    end
    S = load(archivo);
    if ~isfield(S, 'R')
        error('export_uav_simulation_video:formato', ...
            'El .mat no contiene la estructura R de sim_payload_asta.');
    end
    R = S.R;
elseif isstruct(src)
    R = src;
    if isfield(R, 'R') && isstruct(R.R) && isfield(R.R, 't')
        R = R.R;
    end
else
    error('export_uav_simulation_video:formato', ...
        'El segundo argumento debe ser una ruta .mat o la estructura R.');
end
if ~isfield(R, 't') || ~isfield(R, 'x') || ~isfield(R, 'xl') || ~isfield(R, 'xd')
    error('export_uav_simulation_video:formato', ...
        'Faltan campos de sim_payload_asta (t, x, xl, xd).');
end
end
