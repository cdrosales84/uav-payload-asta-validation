function out = planta_sustituta(op, varargin)
% PLANTA_SUSTITUTA  Simplified UAV + cable + payload model, with the same interface as
% planta_simquad. Used to test the code without Windows or SimQuad1.exe (for example
% in Octave). NOT used for any paper result.
%
% Model:
%   - command u = [v_xh v_yh v_zh w_zh] in the yaw frame, with a computation delay of
%     cfg.sust.retardo_muestras samples and a first-order filter tau_att (attitude);
%   - horizontal velocity loop in the yaw frame: v_h_dot = (u_f - kv_ku*v_h)/tau
%     (true theta1 = tau, true theta2 = kv_ku), plus external forces;
%   - point-mass payload attached to the UAV by a spring-damper (tension only);
%   - wind as air velocity with linear drag on the UAV and the payload.

persistent S C
switch op
    case 'iniciar'
        C = varargin{1}; x0 = varargin{2}(:); psi0 = varargin{3};
        S.t = 0; S.p = x0; S.v = zeros(3,1); S.psi = psi0; S.r = 0;
        S.pl = x0 - [0;0;C.planta.l]; S.vl = zeros(3,1);
        S.uf = zeros(4,1); S.buf = zeros(4, C.sust.retardo_muestras + 1);
        S.w = zeros(3,1);
        S.l0 = C.planta.l - (C.planta.mL*C.planta.g)/C.sust.k_cable;   % spring free length
        out = [];
    case 'leer'
        out.t = S.t; out.x = S.p; out.v = S.v; out.psi = S.psi; out.r = S.r;
        out.xl = S.pl + C.sust.ruido_h*randn(3,1); out.ok = true;
    case 'comandar'
        u = varargin{1}(:);
        S.buf = [u, S.buf(:,1:end-1)];
        ua = S.buf(:,end);                         % effective command (with delay)
        k = C.sust; g = C.planta.g; mL = C.planta.mL; m = k.m_uav;
        n = round(C.planta.dt/k.dt_int);
        dt = C.planta.dt/n;
        for i = 1:n
            S.uf = S.uf + dt/k.tau_att*(ua - S.uf);
            % cable (spring-damper, tension only)
            d  = S.p - S.pl; L = norm(d); e = d/L;
            Tn = k.k_cable*(L - S.l0) + k.c_cable*dot(S.v - S.vl, e);
            Tn = max(Tn, 0);
            Fc = Tn*e;                             % force on the payload (toward the UAV)
            % payload
            al = Fc/mL - [0;0;g] - k.cd_carga/mL*(S.vl - S.w);
            % UAV: velocity loop in the yaw frame
            R  = [cos(S.psi) -sin(S.psi); sin(S.psi) cos(S.psi)];
            vh = R.'*S.v(1:2);
            ah = (S.uf(1:2) - k.kv_ku.*vh)./k.tau_xy;
            a  = [R*ah; (S.uf(3) - S.v(3))/k.tau_z] - Fc/m - k.cd_uav/m*(S.v - S.w);
            a(3) = a(3) + mL*g/m;                  % trim thrust: cancels the static payload weight
            rd = (S.uf(4) - S.r)/k.tau_psi;
            % semi-implicit integration
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
