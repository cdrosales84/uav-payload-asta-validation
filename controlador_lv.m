function [u, st, lg] = controlador_lv(cfg, st, m, r, t, dt)
% CONTROLADOR_LV  Un periodo de la cascada de Lv, Wu y Wang (IEEE TVT, 2020).
%
%   [u, st, lg] = controlador_lv(cfg, st, m, r, t, dt)
%
% Articulo: Z. Lv, Y. Wu and R. Wang, "Nonlinear motion control for a quadrotor
% transporting a cable-suspended payload", IEEE Trans. Veh. Technol., 69(8),
% 8192-8206, 2020. DOI 10.1109/TVT.2020.2997733.
% Las ecuaciones citadas abajo estan tomadas de la version de congreso del mismo
% grupo, donde la cascada de tres lazos esta escrita en forma explicita:
% Z. Lv, Y. Wu, W. Xia and W. Wang, "Nonlinear cascade control for a quadrotor
% transporting a slung load", IFAC-PapersOnLine, 53(2), 2020, eqs. (24)-(31).
%
% Que se implementa de la cascada publicada:
%   (31) tension deseada a partir del error de velocidad de la carga;
%   (30) angulos de balanceo deseados a partir de esa fuerza;
%   (25) se evalua y se registra. No entra en el comando: su salida es una
%        aceleracion del soporte, y este simulador no recibe empuje ni cupla.
% El comando es de velocidad. El UAV sigue la velocidad deseada de la carga y
% corrige el cable con el termino proporcional del lazo de balanceo,
%   v = v_ld + L * K_s * (sigma_d - sigma),
% mas un termino de amortiguacion -L*sigma_dot. El par de actitud (24) y el
% desacoplador (26)-(29) no se aplican.
%
% El teorema del articulo es para una velocidad de referencia constante. Para
% seguir una trayectoria, el propio articulo agrega un PID de posicion cuyas
% ganancias no publica. Aqui esa extension es solo proporcional, cfg.lv.Kp.
% K_s publicado vale 6 para un cable de 0.505 m; comparar_lv_vs_asta lo escala
% con sqrt(0.505/L) para no cambiar el orden de la correccion de velocidad.
%
% cfg.lv lo arma comparar_lv_vs_asta. No modifica controlador_paso.m.

P = cfg.lv;
g = cfg.planta.g;
h = m.x - m.xl;
Ln = max(norm(h), 1e-3);
sigma = angulos_cable(h);

if isempty(st)
    st.sigma = sigma;
    st.sigma_d = sigma;
    st.xl = m.xl;
    st.vl_f = zeros(3,1);
    st.sd_f = zeros(2,1);
    st.v_ld_f = zeros(3,1);
    st.sigma_d_f = sigma;
end

vl_raw = (m.xl - st.xl) / dt;
sd_raw = (sigma - st.sigma) / dt;
[vl, st.vl_f] = filtro1(vl_raw, st.vl_f, P.tau_med, dt);
[sigma_dot, st.sd_f] = filtro1(sd_raw, st.sd_f, P.tau_med, dt);
st.xl = m.xl;
st.sigma = sigma;

% Extension de trayectoria (PID de posicion del articulo, solo el termino P).
% El teorema pide una velocidad de referencia constante (aceleracion nula). La
% referencia de este trabajo no lo es: se filtra v_ld con P.tau_ref.
e_p = r.p - m.xl;
[v_ld, st.v_ld_f] = filtro1(r.v + P.Kp * e_p, st.v_ld_f, P.tau_ref, dt);
e_v = v_ld - vl;

% (31) fuerza de tension deseada. Fpg = [0;0;mp*g] con z hacia arriba.
% (30) angulos deseados. El par (24) y el desacoplador (26)-(29) piden empuje
% y cupla; este simulador recibe velocidad. La consigna realiza (30)-(31) en
% velocidad: el UAV se mueve con la carga y corrige el cable con el termino
% proporcional del lazo de balanceo, v = v_ld + L*K_s*(sigma_d - sigma).
Ftd = P.Kv * e_v + [0; 0; P.mp*g] + P.Dd * vl;
Ftd(3) = max(Ftd(3), 0.2*P.mp*g);
sigma_d_raw = angulos_fuerza(Ftd);
[sigma_d, st.sigma_d_f] = filtro1(sigma_d_raw, st.sigma_d_f, P.tau_sigma, dt);
sigma_d = max(min(sigma_d, P.sigma_max), -P.sigma_max);

e_s = wrap2(sigma_d - sigma);
e_ps = -sigma_dot + P.Ks * e_s;
sdd = (eye(2) - P.Ks*P.Ks) * e_s + (P.Ks + P.Kps) * e_ps; % (25), queda en el registro
st.sigma_d = sigma_d;

sigma_dot_cmd = P.Ks * e_s - sigma_dot;
v_des = v_ld + [Ln * sigma_dot_cmd(2); -Ln * sigma_dot_cmd(1); 0];
Rt = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
u_xy = Rt * v_des(1:2);
u_xy_s = min(max(u_xy, -P.u_max), P.u_max);
u_z = min(max(v_des(3), -P.u_max), P.u_max);
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));
wz = min(max(r.dpsi + P.k_yaw * epsi, -P.w_max), P.w_max);
u = [u_xy_s; u_z; wz];

ud = direccion_cable(sigma_d);
lg.h = h;
lg.hp = sd_raw_a_hp(sigma_dot, h, Ln);
lg.hs_h = Ln * ud(1:2);
lg.ex = e_p;
lg.epsi = epsi;
lg.v_c = v_des;
lg.uk_h = u;
lg.v_h = Rt * m.v(1:2);
lg.vt = e_v(1:2);
lg.s = e_s;
lg.nu = sdd;
lg.w = sigma_dot_cmd;
lg.phith = Ftd(1:2);
lg.theta = [sigma; sigma_d];
lg.ad_on = [false; false];
lg.sat = [abs(u_xy(1)) > P.u_max; abs(u_xy(2)) > P.u_max];
lg.u = u;
lg.sigma = 1;
end

%% ------------------------------------------------------------------------
function s = angulos_cable(h)
% h = x - x_l. alpha = 0, beta = 0 con el cable vertical (UAV arriba de la carga).
L = max(norm(h), 1e-6);
ux = h(1)/L; uy = h(2)/L; uz = max(h(3)/L, 1e-3);
beta = atan2(ux, uz);
alpha = -atan2(uy, uz*cos(beta));
s = [alpha; beta];
end

function s = angulos_fuerza(F)
% (30): beta_d = atan(Ftx/Ftz), alpha_d = -atan(Fty/(Ftz*cos(beta_d))).
beta = atan2(F(1), F(3));
alpha = -atan2(F(2), F(3)*cos(beta));
s = [alpha; beta];
end

function u = direccion_cable(s)
alpha = s(1); beta = s(2);
u = [sin(beta); -sin(alpha)*cos(beta); cos(alpha)*cos(beta)];
end

function hp = sd_raw_a_hp(sigma_dot, h, Ln)
% hdot coherente con la cinematica h = L*u(alpha,beta), para el registro.
alpha = angulos_cable(h);
a = alpha(1); b = alpha(2);
ad = sigma_dot(1); bd = sigma_dot(2);
% du/dalpha * ad + du/dbeta * bd
du = [-0; -cos(a)*cos(b); -sin(a)*cos(b)] * ad + ...
     [cos(b); sin(a)*sin(b); -cos(a)*sin(b)] * bd;
hp = Ln * du;
end

function a = wrap2(a)
a = atan2(sin(a), cos(a));
end

function [y, ys] = filtro1(x, ys, tau, dt)
if tau <= 0, y = x; ys = x; return; end
if isempty(ys), ys = x; end
ys = ys + (1 - exp(-dt/tau)) * (x - ys);
y = ys;
end
