function [u, st, lg] = controlador_omar(cfg, st, m, r, t, dt)
% CONTROLADOR_OMAR  One period of the external baseline: delayed-feedback anti-swing
% (Omar, 2022) on a PX4-type position tracker.
%
%   [u, st, lg] = controlador_omar(cfg, st, m, r, t, dt)
%
% Same interface as controlador_paso.m (same measurements m, same reference r,
% same command u = [v_xh; v_yh; v_zh; w_zh] in the UAV yaw frame).
%
% Paper: H. M. Omar, "Hardware-In-the-Loop Simulation of Time-Delayed Anti-Swing
% Controller for Quadrotor with Suspended Load", Applied Sciences 12(3):1706, 2022,
% doi:10.3390/app12031706.
%
% What is implemented from the paper (Sec. 3):
%   Eqs. (1)-(2)  trajectory correction  x_s = K_sx L phi_L(t - t_sx)
%                                          y_s = K_sy L theta_L(t - t_sy)
%                 which is ADDED to the UAV position reference (Fig. 2).
%   Eq. (3)       T_L = 2 pi / sqrt(g/L): the only plant datum the method uses.
%   Eq. (4)       ISH = int(phi_L^2 + theta_L^2) dt, the tuning index (ajustar_omar.m).
%   Sec. 3.1      tracking is done by the PX4 cascade controller: the ASC output
%                 is published as a position setpoint (Sec. 4.8).
%
% Mapping onto SimQuad1.exe. The simulator takes velocity in the yaw frame and
% has its own velocity and attitude loops (the counterpart of the PX4 inner
% loop). What is missing is the PX4 position loop, implemented here as the
% proportional law of the PX4 multicopter position controller with velocity
% feedforward:
%       v_sp = v_ff + K_p (p_sp - x),   K_p = diag(MPC_XY_P, MPC_XY_P, MPC_Z_P)
% with the PX4 defaults (0.95, 0.95, 1.0). This is the same split the paper
% uses: the ASC modifies p_sp and nothing else.
%
% Test-bench choices, the same for both controllers (state them):
%   - p_sp = x_d + [0;0;L] + [x_s; y_s; 0]: the UAV tracks the payload reference
%     shifted up by L (vertical cable); v_ff = xdot_d (a trajectory setpoint
%     with velocity, as in the PX4 trajectory_setpoint).
%   - Startup ramp sigma(t) = 1 - exp(-t/Ts) on the command, the same one as
%     Eq. final_reference of the proposed controller.
%   - Yaw law of Eq. yaw_reference and the same saturation u_max.
%
% Angle convention. Omar defines phi_L (xz plane) and theta_L (out of plane)
% with his own sign, which the paper does not fix. Here phi_x and phi_y are the
% payload displacement relative to the attachment point, projected onto xz and yz:
%       phi_x = atan2(-h_x, h_z),  phi_y = atan2(-h_y, h_z),   h = x - x_l.
% If the UAV tracked p_sp with no lag, the pendulum phi'' + w^2 phi = -xddot/L with
% x = s K L phi(t - tau) would have pole real part -(s K w/2) sin(w tau): it damps
% for s = +1 and tau in (0, T_L/2). The UAV tracks p_sp with the lag of its
% position and velocity loops, so the total phase lag is what matters. The paper
% neither fixes the sign of its angles nor gives a readable delay value:
% ajustar_omar.m chooses sign, K, and tau by minimum ISH, which is the paper's
% criterion (Sec. 3.2).

P = cfg.omar;
l = cfg.planta.l; g = cfg.planta.g;
h = m.x - m.xl;
phi = [atan2(-h(1), h(3)); atan2(-h(2), h(3))];        % [phi_L; theta_L]

%% State: angle history for the delay
if isempty(st)
    st.tb = -inf(1, P.n_buf);
    st.pb = zeros(2, P.n_buf);
    st.k  = 0;
end
st.k = st.k + 1;
ib = mod(st.k - 1, P.n_buf) + 1;
st.tb(ib) = t;
st.pb(:, ib) = phi;

%% Eqs. (1)-(2): delayed correction
phi_ret = valor_retardado(st, t - P.tau, phi);
xs = P.signo * P.K .* l .* phi_ret;                      % [x_s; y_s]
if ~P.activo, xs = zeros(2,1); end

%% PX4-type position tracker (Sec. 3.1)
p_sp = r.p + [0; 0; l] + [xs; 0];
v_ff = r.v;
v_sp = v_ff + P.Kp * (p_sp - m.x);
sig  = 1 - exp(-t/cfg.ext.Ts);                           % same ramp as Eq. final_reference
v_c  = sig * v_sp;

%% Yaw (Eq. yaw_reference) and change to the yaw frame (Eq. yaw_frame)
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));
wz   = r.dpsi + cfg.ext.kpsi * tanh(cfg.ext.kpsit * epsi);
Rt   = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
u_xy = Rt * v_c(1:2);
umax = cfg.int.u_max;
u    = [min(max(u_xy, -umax), umax); min(max(v_c(3), -umax), umax); wz];

%% Log (same fields as controlador_paso)
lg.h = h;
lg.hp = nan(3,1);                                        % the method does not estimate hdot
lg.hs_h = (l/g) * r.a(1:2);                              % only for the swing metric (Eq. href)
lg.hsd_h = (l/g) * r.j(1:2);
lg.ex = r.p - m.xl;
lg.epsi = epsi;
lg.v_c = v_c;
lg.uk_h = [Rt*v_c(1:2); v_c(3); wz];
lg.v_h = Rt * m.v(1:2);
lg.vt = lg.uk_h(1:2) - lg.v_h;
lg.s = phi;                                              % current angles
lg.nu = xs;                                              % trajectory correction
lg.w = phi_ret;                                          % delayed angles
lg.phith = zeros(2,1);
lg.theta = zeros(4,1);
lg.ad_on = [false; false];
lg.sat = abs(u_xy) > umax;
lg.u = u;
lg.sigma = sig;
end

%% ------------------------------------------------------------------------
function y = valor_retardado(st, tq, y_actual)
% Linear interpolation of the history at tq. Before there is enough history,
% returns the oldest available value (at startup the cable is vertical).
n = min(st.k, numel(st.tb));
if n < 2, y = y_actual; return; end
idx = mod((st.k - n : st.k - 1), numel(st.tb)) + 1;      % from oldest to newest
tt = st.tb(idx); pp = st.pb(:, idx);
if tq <= tt(1), y = pp(:,1); return; end
if tq >= tt(end), y = pp(:,end); return; end
j = find(tt <= tq, 1, 'last');
if tt(j+1) <= tt(j), y = pp(:,j); return; end
a = (tq - tt(j)) / (tt(j+1) - tt(j));
y = pp(:,j) + a * (pp(:,j+1) - pp(:,j));
end
