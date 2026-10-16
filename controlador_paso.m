function [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
% CONTROLADOR_PASO  One period of the controller in SN_Payload.tex.
%
%   [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
%
%   cfg : cfg_paper.  m : m.x, m.v (inertial), m.psi, m.xl.
%   r   : r.p, r.v, r.a, r.j, r.psi, r.dpsi.  t, dt in seconds.
%   u   : [u_x; u_y; v_c,z; psi_dot_c] in the yaw frame.
%
%   Equations, in this order:
%     cable_vector, h_raw, h_p_filter, h_hat_dyn,
%     payload_tracking_error, href, base_reference, swing_compensation,
%     final_reference, yaw_reference, yaw_frame,
%     superficie, AccionControl, ajuste2, smc_baseline.
%   eq:yaw_frame fixes n = 2 (x,y channels). z and yaw copy the kinematic command.
%   The paragraph that still says n = 4 next to eq:ajuste2 has no equation of its own.
%   gamma is not published: it is read from cfg.ad.gamma and, if absent, equals 1.
%   AccionControl and smc_baseline do not include saturation.

g = cfg.planta.g; l = cfg.planta.l;
E = cfg.ext; P = cfg.int;
if isfield(cfg, 'ad') && isfield(cfg.ad, 'gamma') && ~isempty(cfg.ad.gamma)
    gamma = cfg.ad.gamma;
else
    gamma = 1;
end
n = 2;
Beta = como_matriz(P.beta, n);
K1   = como_matriz(P.K1, n);
K2   = como_matriz(P.K2, n);

%% Geometry (Eq. cable_vector): h = x - x_l
h = m.x - m.xl;

%% Initialization. Diagonals 0.5 and 0.75: Section 3, reduced to n = 2.
if isempty(st)
    st.hhat    = h;                         % h_hat(0) = h(0)
    st.hp      = zeros(3,1);
    st.vd_prev = [];
    st.Iv      = zeros(n,1);                % integral of v_tilde
    st.nu      = zeros(n,1);                % z = int K2 sign(s) dt
    st.ku      = 0.5 * eye(n);
    st.kv      = 0.75 * eye(n);
end

%% Estimator (Eqs. h_raw, h_p_filter, h_hat_dyn). Filter discretized in exact form.
hp_raw  = (1/cfg.est.eps1) * cfg.est.K1 * (h - st.hhat);
a_h     = 1 - exp(-dt/cfg.est.tau_h);
st.hp   = st.hp + a_h*(hp_raw - st.hp);
hp      = st.hp;
st.hhat = st.hhat + dt*hp;

%% Errors (Eqs. payload_tracking_error, yaw_reference)
ex   = r.p - m.xl;
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));

%% Quasi-static reference (Eq. href)
hs_h  = (l/g) * r.a(1:2);
hsd_h = (l/g) * r.j(1:2);

%% Kinematic loop (Eqs. base_reference, swing_compensation, final_reference)
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

%% Yaw frame (Eq. yaw_frame)
Rt   = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
uk_h = [Rt*uk_w(1:2); uk_w(3:4)];
v_h  = Rt*m.v(1:2);
vd_h = uk_h(1:2);

%% Compensator, x,y channels. v_d is the kinematic command in this frame.
vt = v_h - vd_h;
if isempty(st.vd_prev)
    vdot_d = zeros(n,1);
else
    vdot_d = (vd_h - st.vd_prev)/dt;
end
st.vd_prev = vd_h;

s = zeros(n,1); w = zeros(n,1); phith = zeros(n,1);
ad_on = false(n,1); sat = false(n,1);
switch P.ley
    case 'ninguno'
        u_xy = vd_h;
    case 'smc'
        % Eq. smc_baseline: u = -K_SMC * s / (||s|| + phi)
        st.Iv = st.Iv + vt*dt;
        s     = vt + Beta*st.Iv;
        Ksmc  = como_matriz(P.K_smc, n);
        u_xy  = -Ksmc * (s / (norm(s) + P.phi_smc));
    case {'asta', 'sta_fijo'}
        % Sliding surface and Eq. AccionControl
        st.Iv = st.Iv + vt*dt;
        s     = vt + Beta*st.Iv;
        sg    = sign(s);
        w     = K1*(sqrt(norm(s))*sg) + st.nu;
        rhs   = -Beta*vt + vdot_d + st.kv*v_h;
        phith = st.ku \ rhs;
        u_xy  = st.ku \ (rhs - w);
        st.nu = st.nu + dt*(K2*sg);
        if strcmp(P.ley, 'asta')
            % Eq. ajuste2, integrated by Euler. The matrix is updated, not its transpose.
            ad_on = true(n,1);
            ns    = norm(s);
            ku_dot_T = (u_xy*s.')/gamma - ns*st.ku.';
            kv_dot_T = -(v_h*s.')/gamma - ns*st.kv.';
            st.ku = st.ku + dt*ku_dot_T.';
            st.kv = st.kv + dt*kv_dot_T.';
        end
    otherwise
        error('controlador_paso:int', 'Ley interna desconocida: %s', P.ley);
end

%% z and yaw receive the kinematic command (Eq. yaw_frame).
u = [u_xy; uk_h(3); uk_h(4)];

%% theta is only the log of the diagonal: it is not the state that is adapted.
ku_d = diag(st.ku); kv_d = diag(st.kv);
lg.h = h; lg.hp = hp; lg.hs_h = hs_h; lg.hsd_h = hsd_h; lg.ex = ex; lg.epsi = epsi;
lg.v_c = v_c; lg.uk_h = uk_h; lg.v_h = v_h; lg.vt = vt; lg.s = s; lg.nu = st.nu;
lg.w = w; lg.phith = phith; lg.ad_on = ad_on; lg.sat = sat; lg.u = u; lg.sigma = sig;
lg.theta = [1/ku_d(1); kv_d(1)/ku_d(1); 1/ku_d(2); kv_d(2)/ku_d(2)];
end

%% ------------------------------------------------------------------------
function M = como_matriz(K, n)
% Scalar or diagonal vector -> n x n matrix, as beta, K1, K2, and K_SMC in the manuscript.
if isscalar(K)
    M = K * eye(n);
elseif isvector(K) && numel(K) == n
    M = diag(K(:));
elseif isequal(size(K), [n n])
    M = K;
else
    error('controlador_paso:ganancia', 'Se esperaba escalar, vector %d o matriz %dx%d.', n, n, n);
end
end
