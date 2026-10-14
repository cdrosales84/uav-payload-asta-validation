function [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
% CONTROLADOR_PASO  Un periodo del controlador en cascada (SN_Payload.tex).
%
%   [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
%
%   cfg : estructura de cfg_paper (ganancias Ktraj, Kh, Kdh, K1, K2, beta, K_smc, phi)
%   st  : estado interno ([] en la primera llamada)
%   m   : mediciones -> m.x, m.v (inercial), m.psi, m.xl
%   r   : referencia -> r.p, r.v, r.a, r.j, r.psi, r.dpsi
%   t, dt : tiempo y periodo de muestreo [s]
%
%   u : [u_x; u_y; v_c,z; psi_dot_c] en el marco de yaw.
%       x,y son el comando del compensador (AccionControl o smc_baseline),
%       recortado a [-1, +1]. z y yaw son el comando cinematico.
%
%   El compensador es n = 2 (canales x,y). kappa_u y kappa_v son matrices 2x2,
%   inicializadas en 0.5*I y 0.75*I. gamma se lee de cfg.ad.gamma; si el campo
%   no existe, gamma = 1 (el manuscrito no publica el valor).

g = cfg.planta.g; l = cfg.planta.l;
E = cfg.ext; P = cfg.int;
if isfield(cfg, 'ad') && isfield(cfg.ad, 'gamma') && ~isempty(cfg.ad.gamma)
    gamma = cfg.ad.gamma;
else
    gamma = 1;
end
u_lim = 1;

%% Geometria (Ec. cable_vector)
h = m.x - m.xl;

%% Inicializacion del estado
if isempty(st)
    st.hhat    = h;                  % h_hat(0) = h(0)
    st.hp      = zeros(3,1);
    st.vd_prev = [];
    st.Iv      = zeros(2,1);         % integral de v_tilde
    st.nu      = zeros(2,1);         % z de AccionControl
    st.ku      = 0.5 * eye(2);       % kappa_u inicial del manuscrito
    st.kv      = 0.75 * eye(2);      % kappa_v inicial del manuscrito
end

%% Estimador de dos etapas de hdot (Ecs. h_raw, h_p_filter, h_hat_dyn)
hp_raw  = (1/cfg.est.eps1) * cfg.est.K1 * (h - st.hhat);
a_h     = 1 - exp(-dt/cfg.est.tau_h);
st.hp   = st.hp + a_h*(hp_raw - st.hp);
hp      = st.hp;
st.hhat = st.hhat + dt*hp;

%% Errores (Ecs. payload_tracking_error, yaw_reference)
ex   = r.p - m.xl;
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));

%% Referencia cuasiestatica del cable (Ec. href)
hs_h  = (l/g) * r.a(1:2);
hsd_h = (l/g) * r.j(1:2);

%% Controlador cinematico (Ecs. base_reference, swing_compensation, final_reference)
v_base = r.v + [hsd_h; 0] + E.Ktraj*ex;
switch E.ley
    case 'completo'
        eh     = h(1:2) - hs_h;
        v_comp = [-E.Kh*eh - E.Kdh*(hp(1:2) - hsd_h); 0];
        sig    = 1 - exp(-t/E.Ts);
        v_c    = sig*(v_base + v_comp);
    case 'sin_swing'
        v_comp = zeros(3,1);
        sig    = 1;
        v_c    = v_base;
    otherwise
        error('controlador_paso:ext', 'Ley externa desconocida: %s', E.ley);
end
psid_c = r.dpsi + E.kpsi*tanh(E.kpsit*epsi);
uk_w   = [v_c; psid_c];

%% Cambio al marco de yaw (Ec. yaw_frame)
Rt   = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
uk_h = [Rt*uk_w(1:2); uk_w(3:4)];
v_h  = Rt*m.v(1:2);
vd_h = uk_h(1:2);

%% Compensador dinamico, canales x,y
vt = v_h - vd_h;
if isempty(st.vd_prev)
    vdot_d = zeros(2,1);
else
    vdot_d = (vd_h - st.vd_prev)/dt;
end
st.vd_prev = vd_h;

s = zeros(2,1); w = zeros(2,1); phith = zeros(2,1);
ad_on = false(2,1); sat = false(2,1);
switch P.ley
    case 'ninguno'
        u_xy = vd_h;
    case 'smc'
        st.Iv = st.Iv + vt*dt;
        s     = vt + ganancia(P.beta, st.Iv);
        den   = norm(s) + P.phi_smc;
        if den == 0
            u_raw = zeros(2,1);
        else
            u_raw = -ganancia(P.K_smc, s) / den;          % Ec. smc_baseline
        end
        [u_xy, sat] = recortar(u_raw, u_lim);
    case {'asta', 'sta_fijo'}
        st.Iv = st.Iv + vt*dt;
        s     = vt + ganancia(P.beta, st.Iv);             % s = v_tilde + beta*int(v_tilde)
        ns    = norm(s);
        sg    = sign(s);
        w     = ganancia(P.K1, sqrt(ns)*sg) + st.nu;      % w = K1*sqrt(||s||)*sign(s) + z
        rhs   = -ganancia(P.beta, vt) + vdot_d + st.kv*v_h;
        phith = st.ku \ rhs;
        u_raw = st.ku \ (rhs - w);                        % Ec. AccionControl
        [u_xy, sat] = recortar(u_raw, u_lim);
        st.nu = st.nu + dt*ganancia(P.K2, sg);            % z_dot = K2*sign(s)
        if strcmp(P.ley, 'asta')                          % Ec. ajuste2
            ad_on = true(2,1);
            st.ku = st.ku + dt*((s*u_xy.')/gamma - ns*st.ku);
            st.kv = st.kv + dt*((-s*v_h.')/gamma - ns*st.kv);
        end
    otherwise
        error('controlador_paso:int', 'Ley interna desconocida: %s', P.ley);
end

%% z y yaw: comando cinematico. x,y: compensador.
u = [u_xy; uk_h(3); uk_h(4)];

%% Registro. theta guarda la reparametrizacion diagonal del manuscrito:
%% theta_i = [1/kappa_u,ii ; kappa_v,ii/kappa_u,ii], i en {x,y}.
ku_d = diag(st.ku); kv_d = diag(st.kv);
lg.h = h; lg.hp = hp; lg.hs_h = hs_h; lg.hsd_h = hsd_h; lg.ex = ex; lg.epsi = epsi;
lg.v_c = v_c; lg.uk_h = uk_h; lg.v_h = v_h; lg.vt = vt; lg.s = s; lg.nu = st.nu;
lg.w = w; lg.phith = phith; lg.ad_on = ad_on; lg.sat = sat; lg.u = u; lg.sigma = sig;
lg.theta = [1/ku_d(1); kv_d(1)/ku_d(1); 1/ku_d(2); kv_d(2)/ku_d(2)];
end

%% ------------------------------------------------------------------------
function y = ganancia(K, x)
% Producto de una ganancia escalar, diagonal (vector) o matricial por x.
if numel(K) == 1
    y = K * x;
elseif numel(K) == numel(x)
    y = K(:) .* x(:);
else
    y = K * x;
end
end

function [y, sat] = recortar(x, lim)
y = min(max(x, -lim), lim);
sat = abs(x) > lim;
end
