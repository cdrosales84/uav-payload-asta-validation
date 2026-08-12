function export_uav_simulation_video(outFile, xd, yd, zd, yQuad, yXil, yRope, ...
    payload, tam_carga, frameRate, varargin)

% export_uav_simulation_video — Video MP4 con vista global + recuadro zoom (UAV, cuerda, carga)
%
% Uso:
%   export_uav_simulation_video('sim_dual.mp4', xd, yd, zd, yQuad, yXil, yRope, ...
%       payload, tam_carga, frameRate)
%
% Opcional (Name-Value):
%   'WindHist'       — matriz 3 x N (igual que en simulacion)
%   'FrameStart'     — indice inicial k (default 1)
%   'FrameEnd'       — indice final k (default N = size(yQuad,2))
%   'Stride'         — 1 = todos los frames; 2 = mitad de frames, etc. (default 2)
%   'LocalRadius'    — zoom del recuadro [m] (default 1.2)
%   'FigureSize'     — [ancho alto] pixeles (default [1280 720])
%   'MainAxesPos'    — posicion axes principal [l b w h] normalizado (default [0.06 0.10 0.92 0.82])
%   'InsetAxesPos'   — posicion recuadro zoom (default [0.62 0.14 0.34 0.30])
%   'VideoFrameRate' — FPS del archivo (default frameRate/stride)
%
% Cargar desde .mat (ejemplo):
%   S = load('ResultadosPaper/..._datos.mat');
%   fr = 1 / S.deltat;
%   export_uav_simulation_video('out.mp4', S.xd, S.yd, S.zd, S.yQuad, S.yXil, S.yRope, ...
%       S.payload, S.tam_carga, fr, 'FrameEnd', S.yTFind);
%
% Nota: el .mat debe contener yRope y tam_carga si se quiere el detalle de la cuerda.
%   Si falta yRope, vuelva a exportar datos desde el script de simulacion.

p = inputParser;
addParameter(p, 'WindHist', []);
addParameter(p, 'FrameStart', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'FrameEnd', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'Stride', 2, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'LocalRadius', 1.2, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'FigureSize', [1280 720], @(v) isnumeric(v) && numel(v) == 2);
addParameter(p, 'MainAxesPos', [0.06 0.10 0.92 0.82], @(v) isnumeric(v) && numel(v) == 4);
addParameter(p, 'InsetAxesPos', [0.62 0.14 0.34 0.30], @(v) isnumeric(v) && numel(v) == 4);
addParameter(p, 'VideoFrameRate', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
parse(p, varargin{:});

wind_hist = p.Results.WindHist;
k0 = round(p.Results.FrameStart);
stride = round(p.Results.Stride);
localR = p.Results.LocalRadius;
figSz = p.Results.FigureSize;
posMain = p.Results.MainAxesPos;
posIns = p.Results.InsetAxesPos;

N = size(yQuad, 2);
k1 = p.Results.FrameEnd;
if isempty(k1)
    k1 = N;
end
k0 = max(1, min(k0, N));
k1 = max(k0, min(k1, N));

if isempty(p.Results.VideoFrameRate)
    vfps = frameRate / stride;
else
    vfps = p.Results.VideoFrameRate;
end

fig = figure('Color', 'w', 'Position', [100 100 figSz(1) figSz(2)], 'MenuBar', 'none', 'ToolBar', 'none');
axMain = axes('Parent', fig, 'Position', posMain);
axIns = axes('Parent', fig, 'Position', posIns);

annotation(fig, 'textbox', [0.06 0.94 0.88 0.04], ...
    'String', 'Payload Transport with a Quadrotor UAV Using a Composite Adaptive Robust Control Architecture', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left', ...
    'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'none');

v = VideoWriter(outFile, 'MPEG-4');
v.FrameRate = vfps;
v.Quality = 92;
open(v);

try
    for k = k0:stride:k1
        cla(axMain);
        cla(axIns);

        plot_uav_payload_into_axes(axMain, k, xd, yd, zd, yQuad, yXil, yRope, ...
            payload, tam_carga, frameRate, ...
            'WindHist', wind_hist, ...
            'AxesMode', 'global', ...
            'DrawWind', true, ...
            'TitleStr', sprintf('Vista global — t = %.2f s', (k - 1) / frameRate));

        plot_uav_payload_into_axes(axIns, k, xd, yd, zd, yQuad, yXil, yRope, ...
            payload, tam_carga, frameRate, ...
            'WindHist', wind_hist, ...
            'AxesMode', 'local', ...
            'LocalRadius', localR, ...
            'DrawWind', false, ...
            'TitleStr', sprintf('Zoom — t = %.2f s', (k - 1) / frameRate));

        drawnow nocallbacks;
        frm = getframe(fig);
        writeVideo(v, frm);
    end
catch ME
    try %#ok<CTCH>
        close(v);
    end %#ok<CTCH>
    close(fig);
    rethrow(ME);
end

close(v);
close(fig);
idx = k0:stride:k1;
fprintf('Video guardado: %s (%d frames, %.2f fps)\n', outFile, length(idx), vfps);
end
