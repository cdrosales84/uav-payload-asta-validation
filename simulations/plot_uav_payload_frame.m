function fig = plot_uav_payload_frame(k_frame, xd, yd, zd, yQuad, yXil, yRope, ...
    payload, tam_carga, frameRate, varargin)

% ============================================================
% plot_uav_payload_frame
% Dibuja un frame offline de la simulacion UAV-carga (una figura, un axes).
%
% Uso:
% fig = plot_uav_payload_frame(k, xd, yd, zd, yQuad, yXil, yRope, ...
%     payload, tam_carga, frameRate)
%
% Opcional — historial de viento (misma API que antes):
% fig = plot_uav_payload_frame(..., wind_hist)
%
% La logica de dibujo esta en plot_uav_payload_into_axes.
% Para video con zoom embebido use export_uav_simulation_video.
% ============================================================

wind_hist = [];
if ~isempty(varargin)
    wind_hist = varargin{1};
end

fig = figure('Color', 'w');
s = get(0, 'ScreenSize');
fig.Position = [s(3) * 0.08 s(4) * 0.08 s(3) * 0.80 s(4) * 0.80];
ax = axes('Parent', fig);

plot_uav_payload_into_axes(ax, k_frame, xd, yd, zd, yQuad, yXil, yRope, ...
    payload, tam_carga, frameRate, ...
    'WindHist', wind_hist, ...
    'AxesMode', 'global', ...
    'DrawWind', true, ...
    'TitleStr', sprintf('t = %.2f s', (k_frame - 1) / frameRate));

drawnow;
end
