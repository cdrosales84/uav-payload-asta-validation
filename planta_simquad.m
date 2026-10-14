function out = planta_simquad(op, varargin)
% PLANTA_SIMQUAD  Interfaz con el simulador de alta fidelidad (SimQuad1.exe + mex, Windows).
%
%   planta_simquad('iniciar', cfg, x0, psi0)
%   m = planta_simquad('leer')           -> m.t, m.x, m.v, m.psi, m.r, m.xl, m.rope, m.ok
%   planta_simquad('comandar', u)        -> u = [v_xh v_yh v_zh w_zh] (marco de yaw, m/s y rad/s)
%   planta_simquad('viento', w)          -> w = [wx wy wz] aplicado al UAV, la cuerda y la carga
%   planta_simquad('cerrar')
%
% Requiere que la carpeta ../simulations (mex y SimQuad1.exe) este en el path.
% Indices de quad1 (ver quad1.cpp): 1:3 posicion, 4:6 velocidad inercial, 15 yaw,
% 23 velocidad de yaw (wzh), 24 tiempo del simulador. rope1 devuelve las 40
% particulas de la cuerda; la ultima es la carga.

persistent cfgp
switch op
    case 'iniciar'
        cfgp = varargin{1}; x0 = varargin{2}; psi0 = varargin{3};
        p = cfgp.planta;
        exe = 'SimQuad1.exe';
        [~, msg] = system(['tasklist /fi "ImageName eq ' exe '"']);
        quad1(1, [0 0 0 0]);
        if ~isempty(strfind(msg, exe))
            simulation1(1); pause(0.5);
        else
            system([exe ' &']);
        end
        simulation1([1, 2*p.T, p.mL, p.tam_carga, p.l, p.spring_friction, p.spring_constant, ...
            p.air_friction, p.ground_repel, p.ground_slide, p.ground_absorption, 0], ...
            [x0(:); psi0], 0, [], []);
        pause(1);                                  % inicializacion del simulador
        planta_simquad('viento', [0 0 0]);
        out = [];
    case 'leer'
        q  = quad1(1);
        rp = rope1(1);
        out.t   = q(24);
        out.x   = q(1:3);
        out.v   = q(4:6);
        out.psi = q(15);
        out.r   = q(23);
        out.xl  = rp(end,:).';
        out.ok  = logical(simulation1);
    case 'comandar'
        quad1(1, varargin{1}(:).');
        out = [];
    case 'viento'
        w = varargin{1}(:).';
        wind1(1, 1, w); wind1(2, 1, w); wind1(3, w);
        out = [];
    case 'cerrar'
        quad1(1, [0 0 0 0]);
        simulation1(1);
        out = [];
    otherwise
        error('planta_simquad:op', 'Operacion desconocida: %s', op);
end
end
