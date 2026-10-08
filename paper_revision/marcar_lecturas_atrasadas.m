function [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y, ventana, umbral)
% MARCAR_LECTURAS_ATRASADAS  Quita de una senal ya guardada los picos causados por
% lecturas de la carga atrasadas un frame (ver corregir_lectura_carga.m).
%
%   [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y)
%   [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y, ventana, umbral)
%
%   R       : resultado de sim_payload_asta (se usan R.t, R.x, R.xl)
%   Y       : senal a limpiar, nxN (p.ej. R.vt, R.ukh, R.hp). Si se pasa [] solo se
%             devuelven malo y ev.
%   ventana : segundos reemplazados a partir de cada evento (por defecto 0.1 s, que
%             cubre el pico y su caida). Con ventana = 0 se reemplaza solo la muestra
%             del evento por el promedio de la anterior y la siguiente.
%   umbral  : salto de h que se considera lectura atrasada (por defecto 0.01 m)
%
%   Y    : senal con los tramos marcados reemplazados por interpolacion lineal entre
%          la ultima muestra buena anterior y la primera buena posterior (para una
%          sola muestra, es el promedio de la anterior y la siguiente). Sin NaN.
%   malo : 1xN logico, muestras reemplazadas
%   ev   : indices de las muestras con la lectura atrasada
%
% Ejemplo (figura y metricas sin los picos):
%   S = load('E2_circ_cinematico_rep1.mat'); R = S.R;
%   [vt, malo, ev] = marcar_lecturas_atrasadas(R, R.vt);
%   plot(R.t, vt);
%   sel = R.t >= R.cfg.met.t_inicio;
%   rms_vt = sqrt(mean(vt(:, sel).^2, 2)).';
%   fprintf('%d eventos, %d muestras reemplazadas\n', numel(ev), nnz(malo));

if nargin < 2, Y = []; end
if nargin < 3 || isempty(ventana), ventana = 0.1;   end
if nargin < 4 || isempty(umbral),  umbral  = 0.010; end

t = R.t(:).';  N = numel(t);

% 1) detectar eventos
st = []; ev = [];
for k = 1:N
    m.x = R.x(:, k); m.xl = R.xl(:, k);
    [~, st, c] = corregir_lectura_carga(m, st, t(k), umbral);
    if c, ev(end+1) = k; end %#ok<AGROW>
end

% 2) marcar el evento y la ventana posterior
malo = false(1, N);
malo(ev) = true;
for k = ev
    malo(t >= t(k) & t < t(k) + ventana) = true;
end
malo([1 N]) = false;                    % los extremos se conservan (anclas de la interpolacion)

% 3) reemplazar por interpolacion lineal entre vecinos buenos
if ~isempty(Y) && any(malo)
    if size(Y, 2) ~= N, Y = Y.'; end    % aceptar vectores columna
    bueno = ~malo;
    for i = 1:size(Y, 1)
        Y(i, malo) = interp1(t(bueno), Y(i, bueno), t(malo), 'linear');
    end
end
end