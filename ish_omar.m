function [ish, ish_dev] = ish_omar(R, t_inicio)
% ISH_OMAR  Indice de Omar (2022), Ec. (4): ISH = int(phi_L^2 + theta_L^2) dt.
%
%   [ish, ish_dev] = ish_omar(R, t_inicio)
%
% ish     indice tal como lo define el articulo (angulos absolutos del cable).
% ish_dev mismo indice sobre el desvio respecto de la inclinacion cuasiestatica
%         h*_h = (l/g) a_h (Ec. href). En el circulo la inclinacion de equilibrio
%         no es oscilacion; ish_dev separa las dos cosas. Integracion trapezoidal
%         en el tiempo del simulador.
if nargin < 2, t_inicio = 0; end
sel = R.t >= t_inicio;
t = R.t(sel); h = R.h(:, sel);
px = atan2(-h(1,:), h(3,:)); py = atan2(-h(2,:), h(3,:));
ish = trapz(t, px.^2 + py.^2);
hs = R.hs(:, sel);
qx = atan2(-(h(1,:) - hs(1,:)), h(3,:)); qy = atan2(-(h(2,:) - hs(2,:)), h(3,:));
ish_dev = trapz(t, qx.^2 + qy.^2);
end
