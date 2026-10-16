function [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y, ventana, umbral)
% MARCAR_LECTURAS_ATRASADAS  Removes, from an already saved signal, the spikes caused
% by payload readings delayed by one frame (see corregir_lectura_carga.m).
%
%   [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y)
%   [Y, malo, ev] = marcar_lecturas_atrasadas(R, Y, ventana, umbral)
%
%   R       : result of sim_payload_asta (uses R.t, R.x, R.xl)
%   Y       : signal to clean, n x N (e.g. R.vt, R.ukh, R.hp). If [] is passed, only
%             malo and ev are returned.
%   ventana : seconds replaced after each event (default 0.1 s, which covers the
%             spike and its decay). With ventana = 0 only the event sample is
%             replaced by the average of the previous and the next sample.
%   umbral  : jump in h treated as a delayed reading (default 0.01 m)
%
%   Y    : signal with the marked segments replaced by linear interpolation between
%          the last good sample before and the first good sample after (for a single
%          sample, that is the average of the previous and the next). No NaNs.
%   malo : 1 x N logical, replaced samples
%   ev   : indices of the samples with the delayed reading
%
%   Note: if R.t has repeated instants, among the good samples at the same time
%   the last one is used as the interpolation anchor.

if nargin < 2, Y = []; end
if nargin < 3 || isempty(ventana), ventana = 0.1;   end
if nargin < 4 || isempty(umbral),  umbral  = 0.010; end

t = R.t(:).';  N = numel(t);

% 1) detect events
st = []; ev = [];
for k = 1:N
    m.x = R.x(:, k); m.xl = R.xl(:, k);
    [~, st, c] = corregir_lectura_carga(m, st, t(k), umbral);
    if c, ev(end+1) = k; end %#ok<AGROW>
end

% 2) mark the event and the following window
malo = false(1, N);
malo(ev) = true;
for k = ev
    malo(t >= t(k) & t < t(k) + ventana) = true;
end
malo([1 N]) = false;                    % the endpoints are kept (interpolation anchors)

% 3) replace by linear interpolation between good neighbors
if ~isempty(Y) && any(malo)
    if size(Y, 2) ~= N, Y = Y.'; end    % accept column vectors
    iBueno = find(~malo);

    % interp1 requires unique times: if good samples share an instant,
    % the last reading at that instant is kept
    [tb, iu] = unique(t(iBueno), 'last');
    iBueno   = iBueno(iu);

    if numel(tb) < 2
        warning('marcar_lecturas_atrasadas:pocasMuestras', ...
                'Menos de 2 instantes buenos distintos; no se interpola.');
        return
    end
    for i = 1:size(Y, 1)
        Y(i, malo) = interp1(tb, Y(i, iBueno), t(malo), 'linear');
    end
end
end