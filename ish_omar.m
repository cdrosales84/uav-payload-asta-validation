function [ish, ish_dev] = ish_omar(R, t_inicio)
% ISH_OMAR  Omar index (2022), Eq. (4): ISH = int(phi_L^2 + theta_L^2) dt.
%
%   [ish, ish_dev] = ish_omar(R, t_inicio)
%
% ish     index as defined in the paper (absolute cable angles).
% ish_dev same index on the deviation from the quasi-static tilt
%         h*_h = (l/g) a_h (Eq. href). On the circle the equilibrium tilt
%         is not oscillation; ish_dev separates the two. Trapezoidal
%         integration on the simulator time base.
if nargin < 2, t_inicio = 0; end
sel = R.t >= t_inicio;
t = R.t(sel); h = R.h(:, sel);
px = atan2(-h(1,:), h(3,:)); py = atan2(-h(2,:), h(3,:));
ish = trapz(t, px.^2 + py.^2);
hs = R.hs(:, sel);
qx = atan2(-(h(1,:) - hs(1,:)), h(3,:)); qy = atan2(-(h(2,:) - hs(2,:)), h(3,:));
ish_dev = trapz(t, qx.^2 + qy.^2);
end
