function fig = plot_uav_payload_frame(varargin)
% PLOT_UAV_PAYLOAD_FRAME  Draws one offline frame of the UAV-payload simulation (one figure, one axes).
%
% New format (results from sim_payload_asta / run_paper_experiments):
%   S   = load('results/E2_circ_cinematico/raw/E2_circ_cinematico_rep1.mat');
%   fig = plot_uav_payload_frame(S.R, k)
%   fig = plot_uav_payload_frame(S.R, k, 'AxesMode', 'local', 'Figura', fig)
%       k : sample index. For an instant t:  [~, k] = min(abs(S.R.t - t));
%
%   Own options:
%     'Figura'  existing figure to reuse (it is cleared). Useful for animating
%               without opening one figure per frame.
%   The remaining options are passed to plot_uav_payload_into_axes ('AxesMode',
%   'LocalRadius', 'DrawWind', 'TitleStr', 'Limites', 'EstimarActitud', 'WindHist', ...).
%
% Legacy format (compatibility):
%   fig = plot_uav_payload_frame(k, xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate)
%   fig = plot_uav_payload_frame(..., wind_hist)
%
% Example of a 30 fps animation with the new format:
%   R = S.R; fig = [];
%   for t = 0:1/30:R.t(end)
%       [~, k] = min(abs(R.t - t));
%       fig = plot_uav_payload_frame(R, k, 'Figura', fig);
%   end
%
% The drawing logic is in plot_uav_payload_into_axes.

if isempty(varargin)
    error('plot_uav_payload_frame:args', 'Faltan argumentos.');
end

if isstruct(varargin{1})
    % ---------------- new format: (R, k, options) ----------------
    R = varargin{1};
    if isfield(R, 'R'), R = R.R; end            % S = load(file) was passed
    k = varargin{2};
    opciones = varargin(3:end);

    % split the 'Figura' option from those passed to plot_uav_payload_into_axes
    fig = [];
    i = 1;
    while i <= numel(opciones) - 1
        if ischar(opciones{i}) && strcmpi(opciones{i}, 'Figura')
            fig = opciones{i+1};
            opciones(i:i+1) = [];
        else
            i = i + 2;
        end
    end

    fig = preparar_figura(fig);
    ax = axes('Parent', fig);
    kk = max(1, min(round(k), numel(R.t)));
    plot_uav_payload_into_axes(ax, R, kk, ...
        'AxesMode', 'global', ...
        'DrawWind', true, ...
        'TitleStr', sprintf('t = %.2f s', R.t(kk)), ...
        opciones{:});                           % the caller's options take priority
else
    % ---------------- legacy format ----------------
    [k_frame, xd, yd, zd, yQuad, yXil, yRope, payload, tam_carga, frameRate] = varargin{1:10};
    wind_hist = [];
    if numel(varargin) > 10, wind_hist = varargin{11}; end

    fig = preparar_figura([]);
    ax = axes('Parent', fig);
    plot_uav_payload_into_axes(ax, k_frame, xd, yd, zd, yQuad, yXil, yRope, ...
        payload, tam_carga, frameRate, ...
        'WindHist', wind_hist, ...
        'AxesMode', 'global', ...
        'DrawWind', true, ...
        'TitleStr', sprintf('t = %.2f s',k_frame (k_frame - 1) / frameRate));
end

drawnow;
end

function fig = preparar_figura(fig)
if isempty(fig) || ~ishandle(fig)
    fig = figure('Color', 'w');
    s = get(0, 'ScreenSize');
    set(fig, 'Position', [s(3) * 0.08 s(4) * 0.08 s(3) * 0.80 s(4) * 0.80]);
else
    clf(fig);
    set(0, 'CurrentFigure', fig);
end
end
