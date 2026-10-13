function out = planta_sustituta(op, varargin)
% PLANTA_SUSTITUTA  Modelo simplificado UAV + cable + carga, con la misma interfaz que
% planta_simquad. Sirve para probar el codigo sin Windows ni SimQuad1.exe (por ejemplo
% en Octave). NO se usa para ningun resultado del paper.
%
% Modelo:
%   - comando u = [v_xh v_yh v_zh w_zh] en el marco de yaw, con retardo de computo de
%     cfg.sust.retardo_muestras muestras y un filtro de primer orden tau_att (actitud);
%   - lazo de velocidad horizontal en el marco de yaw: v_h_dot = (u_f - kv_ku*v_h)/tau
%     (theta1 verdadero = tau, theta2 verdadero = kv_ku), mas fuerzas externas;
%   - carga puntual unida al UAV por un resorte-amortiguador (solo traccion);
%   - viento como velocidad del aire con arrastre lineal sobre UAV y carga.

persistent S C
switch op
    case 'iniciar'
        C = varargin{1}; x0 = varargin{2}(:); psi0 = varargin{3};
        S.t = 0; S.p = x0; S.v = zeros(3,1); S.psi = psi0; S.r = 0;
        S.pl = x0 - [0;0;C.planta.l]; S.vl = zeros(3,1);
        S.uf = zeros(4,1); S.buf = zeros(4, C.sust.retardo_muestras + 1);
        S.w = zeros(3,1);
        S.l0 = C.planta.l - (C.planta.mL*C.planta.g)/C.sust.k_cable;   % largo libre del resorte
        out = [];
    case 'leer'
        out.t = S.t; out.x = S.p; out.v = S.v; out.psi = S.psi; out.r = S.r;
        out.att = [0; 0; S.psi];
        out.xl = S.pl + C.sust.ruido_h*randn(3,1);
        s = linspace(0, 1, 14).';
        out.rope = (1 - s) * S.p.' + s * out.xl.';
        out.ok = true;
    case 'comandar'
        u = varargin{1}(:);
        S.buf = [u, S.buf(:,1:end-1)];
        ua = S.buf(:,end);                         % comando efectivo (con retardo)
        k = C.sust; g = C.planta.g; mL = C.planta.mL; m = k.m_uav;
        n = round(C.planta.dt/k.dt_int);
        dt = C.planta.dt/n;
        for i = 1:n
            S.uf = S.uf + dt/k.tau_att*(ua - S.uf);
            % cable (resorte-amortiguador, solo traccion)
            d  = S.p - S.pl; L = norm(d); e = d/L;
            Tn = k.k_cable*(L - S.l0) + k.c_cable*dot(S.v - S.vl, e);
            Tn = max(Tn, 0);
            Fc = Tn*e;                             % fuerza sobre la carga (hacia el UAV)
            % carga
            al = Fc/mL - [0;0;g] - k.cd_carga/mL*(S.vl - S.w);
            % UAV: lazo de velocidad en el marco de yaw
            R  = [cos(S.psi) -sin(S.psi); sin(S.psi) cos(S.psi)];
            vh = R.'*S.v(1:2);
            ah = (S.uf(1:2) - k.kv_ku.*vh)./k.tau_xy;
            a  = [R*ah; (S.uf(3) - S.v(3))/k.tau_z] - Fc/m - k.cd_uav/m*(S.v - S.w);
            a(3) = a(3) + mL*g/m;                  % empuje de equilibrio: compensa el peso estatico de la carga
            rd = (S.uf(4) - S.r)/k.tau_psi;
            % integracion semi-implicita
            S.v = S.v + dt*a;   S.p = S.p + dt*S.v;
            S.vl = S.vl + dt*al; S.pl = S.pl + dt*S.vl;
            S.r = S.r + dt*rd;  S.psi = S.psi + dt*S.r;
            S.t = S.t + dt;
        end
        out = [];
    case 'viento'
        S.w = varargin{1}(:); out = [];
    case 'cerrar'
        out = [];
    otherwise
        error('planta_sustituta:op', 'Operacion desconocida: %s', op);
end
end
