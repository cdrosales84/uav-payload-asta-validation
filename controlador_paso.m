function [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
% CONTROLADOR_PASO  Un periodo del controlador en cascada (Secc. 2 del manuscrito revisado).
%
%   [u, st, lg] = controlador_paso(cfg, st, m, r, t, dt)
%
%   cfg : estructura de cfg_paper
%   st  : estado interno del controlador ([] en la primera llamada)
%   m   : mediciones en t  -> m.x (UAV, 3x1), m.v (UAV, 3x1, inercial), m.psi, m.xl (carga, 3x1)
%   r   : referencia en t  -> r.p, r.v, r.a, r.j (3x1), r.psi, r.dpsi
%   t   : tiempo [s];  dt : periodo de muestreo real [s]
%
%   u   : comando al simulador [v_xh; v_yh; v_zh; w_zh] en el marco de yaw del UAV
%   lg  : senales internas para registro
%
% Cada bloque lleva la etiqueta de la ecuacion que implementa (ver EQUATION_MAP.md).
% No hay otros terminos que los listados: sin saturacion tanh en el lazo externo y
% sin fuga en las integrales. El lazo externo incluye un termino de velocidad de la
% carga (Kv) y un washout sobre el termino Kh (tau_w), ambos lineales.

g = cfg.planta.g; l = cfg.planta.l;
E = cfg.ext; P = cfg.int; A = cfg.ad; F = cfg.filt;

%% Geometria (Ec. cable_vector)
h = m.x - m.xl;

%% Inicializacion del estado
if isempty(st)
    st.hhat   = h;                 % h_hat(0) = h(0): evita un pico espurio de h_p en k = 1
    st.hp     = zeros(3,1);
    st.ex_f   = [];  st.epsi_f = [];  st.vc_f = [];
    st.vd_prev = [];  st.vdot_f = zeros(2,1);
    st.Iv     = zeros(2,1);        % integral de v_tilde (canales x,y)
    st.nu     = zeros(2,1);        % estado integral del super-twisting
    st.theta  = repmat(A.theta_ini(:), 1, 2);   % columnas: canal x, canal y
    st.ehw    = zeros(2,1);        % componente lenta de (h_h - h*_h) en marco de yaw (washout)
end

%% Estimador de dos etapas de hdot (Ecs. h_raw, h_p_filter, h_hat_dyn)
% Discretizacion: h_p filtrado con a_h = 1 - exp(-dt/tau_h); h_hat propagado por Euler
% explicito al instante siguiente.
hp_raw = (1/cfg.est.eps1) * cfg.est.K1 * (h - st.hhat);         % Ec. h_raw
a_h    = 1 - exp(-dt/cfg.est.tau_h);
st.hp  = st.hp + a_h*(hp_raw - st.hp);                            % Ec. h_p_filter
hp     = st.hp;                                                   % estimacion usada en t
st.hhat = st.hhat + dt*hp;                                        % Ec. h_hat_dyn (para t+dt)

%% Errores (Ecs. payload_tracking_error, yaw_reference)
ex   = r.p - m.xl;                                                % x_tilde = x_d - x_l
%% Error de velocidad de la carga (Ec. payload_velocity_error)
xl_dot_hat = m.v - hp;                 % xdot_l = xdot - hdot (Ec. cable_vector derivada)
exd        = r.v - xl_dot_hat;         % xdot_tilde = xdot_d - xdot_l
epsi = atan2(sin(r.psi - m.psi), cos(r.psi - m.psi));
[ex,   st.ex_f]   = filtro1(ex,   st.ex_f,   F.tau_ex,   dt);
[epsi, st.epsi_f] = filtro1(epsi, st.epsi_f, F.tau_epsi, dt);

%% Referencia cuasiestatica del cable (Ec. href, signo +: el UAV va adelantado a la carga)
hs_h  = (l/g) * r.a(1:2);                                         % h*_h
hsd_h = (l/g) * r.j(1:2);                                         % hdot*_h, analitica

%% Controlador cinematico (Ecs. base_reference, swing_compensation, final_reference)
switch E.ley
    case 'completo'
        if isfield(E, 'seguimiento') && strcmp(E.seguimiento, 'abril')
            v_base = r.v + E.Ktraj_abril*tanh(E.Ktrajt*ex) + E.Kvload*tanh(E.Kvloadt*exd);
        else
            ff     = [E.usar_ff_hdot*hsd_h; 0];
            % Ec. base_reference + termino de velocidad de la carga (amortigua el modo
            % pendular; exd usa la estimacion hp, por eso va despues del estimador)
            v_base = r.v + ff + E.Ktraj*ex + E.Kv*exd;
        end
        % Washout del termino proporcional de swing (Ec. swing_compensation).
        % El arrastre de la carga produce una inclinacion permanente del cable, constante
        % en el marco de yaw. Sin washout, Kh la "corrige" frenando al UAV y la convierte
        % en error de seguimiento: x_tilde ~ (Kh/Ktraj)*h_arrastre. El pasaaltos (tau_w)
        % deja pasar el pendulo (~1.6-2.8 rad/s en ese marco) y elimina la parte lenta.
        eh = h(1:2) - hs_h;
        if E.tau_w > 0
            Rt_w = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];
            if t > E.t_w_on                                   % no absorber el arranque
                st.ehw = st.ehw + (1 - exp(-dt/E.tau_w))*(Rt_w*eh - st.ehw);
            end
            eh = eh - Rt_w.'*st.ehw;
        end
        v_comp = [-E.Kh*eh - E.Kdh*(hp(1:2) - hsd_h); 0];   % Ec. swing_compensation
    case 'sin_swing'
        v_base = r.v + E.Ktraj*ex;                                % ley sin compensacion (Fig. 6a)
        v_comp = zeros(3,1);
    otherwise
        error('controlador_paso:ext', 'Ley externa desconocida: %s', E.ley);
end
sig  = 1 - exp(-t/E.Ts);                                          % sigma(t)
v_c  = sig*(v_base + v_comp);                                     % Ec. final_reference
psid_c = r.dpsi + E.kpsi*tanh(E.kpsit*epsi);                      % Ec. yaw_reference
uk_w = [v_c; psid_c];                                             % u_k en marco inercial

%% Cambio al marco de yaw (Ec. yaw_frame): el simulador recibe velocidades en ese marco
Rt   = [cos(m.psi) sin(m.psi); -sin(m.psi) cos(m.psi)];           % R_psi^T (2x2)
uk_h = [Rt*uk_w(1:2); uk_w(3:4)];
[uk_h, st.vc_f] = filtro1(uk_h, st.vc_f, F.tau_vc, dt);
v_h  = Rt*m.v(1:2);                                               % velocidad medida, marco de yaw
vd_h = uk_h(1:2);                                                 % v_d de los canales x,y

%% Compensador dinamico (Secc. dynamic), canales x,y
vt = v_h - vd_h;                                                  % v_tilde
if isempty(st.vd_prev), vdot_raw = zeros(2,1); else, vdot_raw = (vd_h - st.vd_prev)/dt; end
st.vd_prev = vd_h;
[vdot_d, st.vdot_f] = filtro1(vdot_raw, st.vdot_f, F.tau_vdot, dt);   % derivada numerica filtrada de v_d
a_reg = vdot_d - P.beta*vt;                                       % a = vd_dot - beta*v_tilde

s = zeros(2,1); w = zeros(2,1); phith = zeros(2,1); ad_on = false(2,1); sat = false(2,1);
switch P.ley
    case 'ninguno'
        u_xy = vd_h;                                              % solo controlador cinematico
    case {'asta', 'sta_fijo', 'smc'}
        st.Iv = st.Iv + vt*dt;
        s     = vt + P.beta*st.Iv;                                % Ec. superficie
        for i = 1:2
            phi_i    = [a_reg(i), v_h(i)];                        % regresor phi_i (1x2)
            phith(i) = phi_i * st.theta(:,i);                     % phi_i*theta_hat_i
        end
        if strcmp(P.ley, 'smc')
            u_xy = phith - P.K_smc .* s ./ (abs(s) + P.phi_smc);  % ablacion, normalizacion por componente
        else
            sg   = sgn(s, P.phi_bl);
            w1   = P.K1 .* sqrt(abs(s)) .* sg;                    % K1*sqrt|s|*sign(s)
            nu_n = st.nu + P.K2 .* sg * dt;                       % nu_dot = K2*sign(s)
            u_try = phith - (w1 + nu_n);                          % Ec. AccionControl: u = phi*theta_hat - w
            for i = 1:2                                           % integracion condicional (anti-windup)
                sat(i) = abs(u_try(i)) > P.u_max;
                if ~(sat(i) && u_try(i)*(-sg(i)) > 0)
                    st.nu(i) = nu_n(i);
                end
            end
            w    = w1 + st.nu;
            u_xy = phith - w;

            %% Ley de adaptacion (Ec. ajuste_theta)
            if strcmp(P.ley, 'asta') && t > A.t_on
                for i = 1:2
                    if abs(s(i)) <= A.s_on
                        ad_on(i) = true;
                        phi_i = [a_reg(i); v_h(i)];
                        nus   = min(max(st.nu(i), -A.nu_bar), A.nu_bar);       % sat(nu)
                        thd   = -A.Gamma*phi_i*nus - A.sigma*(st.theta(:,i) - A.theta_prior(:));
                        st.theta(:,i) = min(max(st.theta(:,i) + dt*thd, A.Theta_min(:)), A.Theta_max(:));  % Proj_Theta
                    end
                end
            end
        end
    otherwise
        error('controlador_paso:int', 'Ley interna desconocida: %s', P.ley);
end
u_xy = min(max(u_xy, -P.u_max), P.u_max);

%% Comando al simulador: x,y del compensador; z y yaw reciben el comando cinematico
u = [u_xy; uk_h(3); uk_h(4)];

%% Registro
lg.ehw = st.ehw; lg.h = h; lg.hp = hp; lg.hs_h = hs_h; lg.hsd_h = hsd_h; lg.ex = ex; lg.epsi = epsi;
lg.v_c = v_c; lg.uk_h = uk_h; lg.v_h = v_h; lg.vt = vt; lg.s = s; lg.nu = st.nu; lg.w = w;
lg.phith = phith; lg.theta = st.theta(:); lg.ad_on = ad_on; lg.sat = sat; lg.u = u; lg.sigma = sig;
end

%% ------------------------------------------------------------------------
function [y, ys] = filtro1(x, ys, tau, dt)
% Filtro de primer orden discreto; tau = 0 lo desactiva.
if tau <= 0, y = x; ys = x; return; end
if isempty(ys), ys = x; end
ys = ys + (1 - exp(-dt/tau))*(x - ys);
y = ys;
end

function y = sgn(s, phi)
if phi > 0, y = s ./ (abs(s) + phi); else, y = sign(s); end
end