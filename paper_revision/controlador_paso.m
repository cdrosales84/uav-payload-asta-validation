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
% No hay otros terminos que los listados: sin saturacion tanh en el lazo externo,
% sin termino de velocidad de la carga, sin fuga en las integrales.
%
% Correspondencia con Paper/SN_Payload.tex. El lazo externo ya es el impreso
% (Ecs. cable_vector, h_raw, h_p_filter, h_hat_dyn, payload_tracking_error,
% href, base_reference, swing_compensation, final_reference, yaw_reference,
% yaw_frame). El lazo interno impreso no: eq:AccionControl, eq:ajuste2 y
% eq:smc_baseline. Los comentarios [TEX] dicen la sustitucion que igualaria
% ese texto. No cambian la ley que corre. El parrafo del modelo (~l. 349)
% ya describe esta implementacion (n = 2, theta, z y yaw sin compensador);
% igualar eq:AccionControl contradice ese parrafo.

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
    % [TEX] La seccion de simulacion inicializa kappa_u_hat = 0.5 y
    % kappa_v_hat = 0.75 en cuatro canales, o sea theta = [2; 1.5].
    % Para igualar: n = 4 y theta_ini = [1/0.5; 0.75/0.5]. Aqui theta_ini
    % llega de cfg (nominal [0.4; 1]) y solo hay columnas x e y.
    st.u_prev = zeros(4,1);
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
            % [TEX] Esta rama (tanh del error y de la velocidad de la carga) no esta en el articulo.
            v_base = r.v + E.Ktraj_abril*tanh(E.Ktrajt*ex) + E.Kvload*tanh(E.Kvloadt*exd);
        else
            ff     = [E.usar_ff_hdot*hsd_h; 0];
            v_base = r.v + ff + E.Ktraj*ex;                       % Ec. base_reference
        end
        v_comp = [-E.Kh*(h(1:2) - hs_h) - E.Kdh*(hp(1:2) - hsd_h); 0];   % Ec. swing_compensation
    case 'sin_swing'
        % [TEX] La corrida sin compensacion del articulo es eq:base_reference,
        % que conserva [hdot*_h; 0]. Esta rama tambien lo quita.
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
% [TEX eq:AccionControl] Para igualar la ley impresa, este bloque pasa a n = 4
% (x, y, z, yaw) y la accion queda dentro de la inversa, con la norma del vector:
%   w     = K1*sqrt(norm(s))*sign(s) + nu
%   nudot = K2*sign(s)
%   u     = kappa_u_hat \ (vdot_d - beta*v_tilde + kappa_v_hat*v - w)
% con kappa_u_hat, kappa_v_hat en R^{4x4}. Reescrita en theta, esa ecuacion es
% u_i = phi_i*theta_i - theta_1,i*w_i. Lo que corre es u_i = phi_i*theta_hat_i - w_i
% en dos canales, con sqrt(|s_i|) por componente y w fuera de la inversion.
vt = v_h - vd_h;                                                  % v_tilde
if isempty(st.vd_prev), vdot_raw = zeros(2,1); else, vdot_raw = (vd_h - st.vd_prev)/dt; end
st.vd_prev = vd_h;
% [TEX] eq:AccionControl usa vdot_d; no declara el filtro tau_vdot = 0.05 s.
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
            % [TEX eq:smc_baseline] u = -K_SMC * s / (norm(s) + phi), sin phi*theta.
            u_xy = phith - P.K_smc .* s ./ (abs(s) + P.phi_smc);  % ablacion, normalizacion por componente
        else
            % [TEX] sign(s) puro y sqrt(norm(s)) comun a los cuatro canales.
            % phi_bl > 0 aproxima el signo y no esta en el articulo (cfg lo deja en 0).
            sg   = sgn(s, P.phi_bl);
            w1   = P.K1 .* sqrt(abs(s)) .* sg;                    % K1*sqrt|s|*sign(s), por canal
            nu_n = st.nu + P.K2 .* sg * dt;                       % nu_dot = K2*sign(s)
            % Forma que corre: u = phi*theta_hat - w. No es la eq:AccionControl impresa.
            u_try = phith - (w1 + nu_n);
            % [TEX] El articulo supone comandos en rango lineal: no hay u_max
            % ni esta integracion condicional de nu.
            for i = 1:2                                           % integracion condicional (anti-windup)
                sat(i) = abs(u_try(i)) > P.u_max;
                if ~(sat(i) && u_try(i)*(-sg(i)) > 0)
                    st.nu(i) = nu_n(i);
                end
            end
            w    = w1 + st.nu;
            u_xy = phith - w;

            %% Ley de adaptacion. El .tex no tiene eq:ajuste_theta; la impresa es eq:ajuste2.
            % [TEX eq:ajuste2] Siempre activa. Gradiente con s y fuga hacia cero:
            %   d/dt kappa_u_hat.' = (1/gamma)*u*s.' - norm(s)*kappa_u_hat.'
            %   d/dt kappa_v_hat.' = -(1/gamma)*v*s.' - norm(s)*kappa_v_hat.'
            % Para igualar hay que integrar esas matrices, no theta. La fuga del
            % codigo, -sigma*(theta_hat - theta_prior), y la proyeccion a Theta
            % no estan. eq:ajuste2 se apaga cuando s entra en un entorno de 0;
            % la compuerta de abajo se enciende ahi (t > t_on y |s_i| <= s_on)
            % y el gradiente usa sat(nu), no s.
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
% [TEX] eq:AccionControl no recorta u a u_max.
u_xy = min(max(u_xy, -P.u_max), P.u_max);

%% Comando al simulador: x,y del compensador; z y yaw reciben el comando cinematico
% [TEX eq:AccionControl] Con el n = 4 de ese parrafo, z y yaw tambien pasan
% por el compensador. El modelo (~l. 349) y esta linea los dejan en v_c,z y psid_c.
u = [u_xy; uk_h(3); uk_h(4)];
st.u_prev = u;

%% Registro
lg.h = h; lg.hp = hp; lg.hs_h = hs_h; lg.hsd_h = hsd_h; lg.ex = ex; lg.epsi = epsi;
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
% [TEX] eq:AccionControl y eq:ajuste2 usan sign(s). La rama phi > 0 no esta en el articulo.
if phi > 0, y = s ./ (abs(s) + phi); else, y = sign(s); end
end
