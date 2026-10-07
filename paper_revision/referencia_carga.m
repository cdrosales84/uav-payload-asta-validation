function r = referencia_carga(t, cfgref)
% REFERENCIA_CARGA  Referencia de la carga y sus derivadas, en forma analitica.
%
%   r = referencia_carga(t, cfg.ref)
%
%   r.p, r.v, r.a, r.j : posicion, velocidad, aceleracion y jerk (3 x N)
%   r.psi, r.dpsi      : yaw de referencia y su derivada (1 x N)
%
% Reemplaza a ReferenciasCarga_ST.m para los experimentos del paper. Diferencias:
%   - se evalua en el tiempo real del simulador (t puede ser no uniforme);
%   - entrega el jerk, necesario para hdot* = (l/g)*jerk_h (Ec. href);
%   - corrige la aceleracion de la lemniscata. En ReferenciasCarga_ST (casos 4 y 5)
%     xdd = -2 a c^2 (sin(ct) + cos(ct)), cuando lo correcto es
%     x = (a/2) sin(2ct)  ->  xdd = -2 a c^2 sin(2ct).
%   - psi_d = atan2(yd_dot, xd_dot) y su derivada exacta.

t = t(:).';
N = numel(t);
switch cfgref.tipo
    case 'circulo'                       % Ec. ref_circ
        R = cfgref.circ.R; w = 2*pi/cfgref.circ.Tp;
        z0 = cfgref.circ.z0; vz = cfgref.circ.vz;
        s = sin(w*t); c = cos(w*t);
        r.p = [ R*s;         R*c;          z0 + vz*t];
        r.v = [ R*w*c;      -R*w*s;        vz*ones(1,N)];
        r.a = [-R*w^2*s;    -R*w^2*c;      zeros(1,N)];
        r.j = [-R*w^3*c;     R*w^3*s;      zeros(1,N)];
    case 'lemniscata'                    % Ec. ref_lemn
        a = cfgref.lemn.a; b = cfgref.lemn.b; c0 = cfgref.lemn.c;
        d = cfgref.lemn.d; z0 = cfgref.lemn.z0;
        s1 = sin(c0*t); c1 = cos(c0*t); s2 = sin(2*c0*t); c2 = cos(2*c0*t);
        r.p = [ a/2*s2;            b*s1;          z0 + d*s1];
        r.v = [ a*c0*c2;           b*c0*c1;       d*c0*c1];
        r.a = [-2*a*c0^2*s2;      -b*c0^2*s1;    -d*c0^2*s1];
        r.j = [-4*a*c0^3*c2;      -b*c0^3*c1;    -d*c0^3*c1];
    otherwise
        error('referencia_carga:tipo', 'Tipo de referencia desconocido: %s', cfgref.tipo);
end
vx = r.v(1,:); vy = r.v(2,:); ax = r.a(1,:); ay = r.a(2,:);
r.psi  = atan2(vy, vx);
r.dpsi = (vx.*ay - vy.*ax) ./ max(vx.^2 + vy.^2, 1e-9);
end
