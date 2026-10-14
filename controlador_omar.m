function [u, st, lg] = controlador_omar(cfg, st, m, r, t, dt)
% CONTROLADOR_OMAR  Un periodo del baseline externo: anti-swing por realimentacion
% retardada (Omar, 2022) sobre un seguidor de posicion tipo PX4.
%
%   [u, st, lg] = controlador_omar(cfg, st, m, r, t, dt)
%
% Misma interfaz que controlador_paso.m (mismas mediciones m, misma referencia r,
% mismo comando u = [v_xh; v_yh; v_zh; w_zh] en el marco de yaw del UAV).
%
% Articulo: H. M. Omar, "Hardware-In-the-Loop Simulation of Time-Delayed Anti-Swing
% Controller for Quadrotor with Suspended Load", Applied Sciences 12(3):1706, 2022,
% doi:10.3390/app12031706.
%
% Que se implementa del articulo (Secc. 3):
%   Ecs. (1)-(2)  correccion de trayectoria  x_s = K_sx L phi_L(t - t_sx)
%                                             y_s = K_sy L theta_L(t - t_sy)
%                 que se SUMA a la referencia de posicion del UAV (Fig. 2).
%   Ec. (3)       T_L = 2 pi / sqrt(g/L): unico dato del sistema que usa el metodo.
%   Ec. (4)       ISH = int(phi_L^2 + theta_L^2) dt, indice de ajuste (ajustar_omar.m).
%   Secc. 3.1     el seguimiento lo hace el controlador en cascada de PX4: la salida
%                 del ASC se publica como setpoint de posicion (Secc. 4.8).
%
% Correspondencia con SimQuad1.exe. El simulador recibe velocidad en el marco de yaw
% y tiene sus propios lazos de velocidad y actitud (el equivalente del lazo interno
% de PX4). Lo que falta es el lazo de posicion de PX4, que aqui es el proporcional
% del multicopter position controller de PX4 con prealimentacion de velocidad:
%       v_sp = v_ff + K_p (p_sp - x),   K_p = diag(MPC_XY_P, MPC_XY_P, MPC_Z_P)
% con los valores por defecto de PX4 (0.95, 0.95, 1.0). Es el mismo corte que usa el
% articulo: el ASC modifica p_sp y nada mas.
%
% Decisiones del banco de pruebas, iguales para los dos controladores (declararlas):
%   - p_sp = x_d + [0;0;L] + [x_s; y_s; 0]: el UAV sigue la referencia de la carga
%     desplazada L hacia arriba (cable vertical); v_ff = xdot_d (setpoint de
%     trayectoria con velocidad, como el trajectory_setpoint de PX4).
%   - Rampa de arranque sigma(t) = 1 - exp(-t/Ts) sobre el comando, la misma de la
%     Ec. final_reference del controlador propuesto.
%   - Ley de yaw de la Ec. yaw_reference y la misma saturacion u_max.
%
% Convencion de angulos. Omar define phi_L (plano xz) y theta_L (fuera del plano)
% con su propio signo, que el articulo no fija. Aqui phi_x y phi_y son el
% desplazamiento de la carga respecto del punto de enganche, proyectado en xz y yz:
%       phi_x = atan2(-h_x, h_z),  phi_y = atan2(-h_y, h_z),   h = x - x_l.
% Si el UAV siguiera p_sp sin atraso, el pendulo phi'' + w^2 phi = -xddot/L con
% x = s K L phi(t - tau) tendria parte real del polo -(s K w/2) sin(w tau): amortigua
% con s = +1 y tau en (0, T_L/2). El UAV sigue p_sp con el atraso de su lazo de
% posicion y velocidad, asi que lo que cuenta es el desfase total. El articulo no
% fija el signo de sus angulos ni da un valor legible del retardo: ajustar_omar.m
% elige signo, K y tau por minimo ISH, que es el criterio del articulo (Secc. 3.2).

P = cfg.omar;
l = cfg.planta.l; g = cfg.planta.g;
h = m.x - m.xl;
phi = [atan2(-h(1), h(3)); atan2(-h(2), h(3))];        % [phi_L; theta_L]

%% Estado: historia de los angulos para el retardo
if isempty(st)
    st.tb = -inf(1, P.n_buf);
    st.pb = zeros(2, P.n_buf);
    st.k  = 0;
end
st.k = st.k + 1;
ib = mod(st.k - 1, P.n_buf) + 1;
st.tb(ib) = t;
st.pb(:, ib) = phi;

%% Ecs. (1)-(2): correccion retardada
phi_ret = valor_retardado(st, t - P.tau, phi);
xs = P.signo * P.K .* l .* phi_ret;                      % [x_s; y_s]
if ~P.activo, xs = zeros(2,1); end

%% Seguidor de posicion tipo PX4 (Secc. 3.1)
p_sp = r.p + [0; 0; l] + [xs; 0];
v_ff = r.v;
v_sp = v_ff + P.Kp * (p_sp - m.x);
sig  = 1 - exp(-t/cfg.ext.Ts);                           % misma rampa que Ec. final_reference
v_c  = sig * v_sp;

%% Yaw (Ec. yaw_reference) y cambio al marco de yaw (Ec. yaw_frame)
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));
wz   = r.dpsi + cfg.ext.kpsi * tanh(cfg.ext.kpsit * epsi);
Rt   = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
u_xy = Rt * v_c(1:2);
umax = cfg.int.u_max;
u    = [min(max(u_xy, -umax), umax); min(max(v_c(3), -umax), umax); wz];

%% Registro (mismos campos que controlador_paso)
lg.h = h;
lg.hp = nan(3,1);                                        % el metodo no estima hdot
lg.hs_h = (l/g) * r.a(1:2);                              % solo para la metrica de swing (Ec. href)
lg.hsd_h = (l/g) * r.j(1:2);
lg.ex = r.p - m.xl;
lg.epsi = epsi;
lg.v_c = v_c;
lg.uk_h = [Rt*v_c(1:2); v_c(3); wz];
lg.v_h = Rt * m.v(1:2);
lg.vt = lg.uk_h(1:2) - lg.v_h;
lg.s = phi;                                              % angulos actuales
lg.nu = xs;                                              % correccion de trayectoria
lg.w = phi_ret;                                          % angulos retardados
lg.phith = zeros(2,1);
lg.theta = zeros(4,1);
lg.ad_on = [false; false];
lg.sat = abs(u_xy) > umax;
lg.u = u;
lg.sigma = sig;
end

%% ------------------------------------------------------------------------
function y = valor_retardado(st, tq, y_actual)
% Interpolacion lineal de la historia en tq. Antes de tener historia suficiente,
% devuelve el valor mas viejo disponible (al arrancar, el cable esta vertical).
n = min(st.k, numel(st.tb));
if n < 2, y = y_actual; return; end
idx = mod((st.k - n : st.k - 1), numel(st.tb)) + 1;      % del mas viejo al mas nuevo
tt = st.tb(idx); pp = st.pb(:, idx);
if tq <= tt(1), y = pp(:,1); return; end
if tq >= tt(end), y = pp(:,end); return; end
j = find(tt <= tq, 1, 'last');
if tt(j+1) <= tt(j), y = pp(:,j); return; end
a = (tq - tt(j)) / (tt(j+1) - tt(j));
y = pp(:,j) + a * (pp(:,j+1) - pp(:,j));
end
