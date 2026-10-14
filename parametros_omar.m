function P = parametros_omar(cfg, varargin)
% PARAMETROS_OMAR  Parametros del baseline de Omar (Appl. Sci. 2022) para cfg.omar.
%
%   P = parametros_omar(cfg)                      % usa omar_ajuste.mat si existe
%   P = parametros_omar(cfg, 'K', 0.6, 'tau_TL', 0.1, 'signo', +1)
%   P = parametros_omar(cfg, 'activo', false)    % solo el seguidor tipo PX4 (sin ASC)
%
% Valores:
%   K       ganancia de las Ecs. (1)-(2). Omar usa K_sx = K_sy = 0.6 (Secc. 5.1),
%           multiplicada por L en la ecuacion.
%   tau     retardo t_sx = t_sy [s]. El articulo dice que se elige por minimo ISH
%           (Ec. 4) y solo usa T_L (Ec. 3). El desfase util es el TOTAL, retardo
%           mas el atraso del lazo de posicion y velocidad del UAV (~0.28 T_L a la
%           frecuencia del pendulo con Kp = 0.95). Por eso el retardo agregado es
%           chico: sin ajuste, tau = 0.1 T_L y signo +1 (prueba de hover en la
%           planta sustituta: ISH -51 % respecto de sin ASC).
%           ajustar_omar.m hace la busqueda por ISH (signo, K, tau) y guarda
%           omar_ajuste.mat, que esta funcion carga automaticamente.
%   Kp      lazo de posicion de PX4, valores por defecto MPC_XY_P = 0.95, MPC_Z_P = 1.0.

op = struct('K', [], 'tau_TL', [], 'signo', [], 'activo', true, 'archivo', '');
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

g = cfg.planta.g; l = cfg.planta.l;
P.TL = 2*pi / sqrt(g/l);                         % Ec. (3)
P.K = 0.6;                                       % Omar, Secc. 5.1
P.tau_TL = 0.10;
P.signo = +1;
P.origen = 'por defecto (K = 0.6 del articulo, tau = 0.1 T_L, sin ajuste en SimQuad)';

aqui = fileparts(mfilename('fullpath'));
if isempty(op.archivo), op.archivo = fullfile(aqui, 'omar_ajuste.mat'); end
if exist(op.archivo, 'file')
    S = load(op.archivo);
    P.K = S.ajuste.K; P.tau_TL = S.ajuste.tau_TL;
    if isfield(S.ajuste, 'signo'), P.signo = S.ajuste.signo; end
    P.origen = sprintf('omar_ajuste.mat (minimo ISH, %s)', S.ajuste.fecha);
end
if ~isempty(op.K), P.K = op.K; P.origen = 'argumento'; end
if ~isempty(op.tau_TL), P.tau_TL = op.tau_TL; P.origen = 'argumento'; end
if ~isempty(op.signo), P.signo = op.signo; P.origen = 'argumento'; end

P.K   = P.K * [1; 1];                            % K_sx = K_sy
P.tau = P.tau_TL * P.TL;                         % t_sx = t_sy
P.activo = op.activo;
P.Kp = diag([0.95 0.95 1.0]);                    % PX4: MPC_XY_P, MPC_XY_P, MPC_Z_P
P.n_buf = max(50, ceil(1.5 * P.tau / cfg.planta.dt) + 20);
end
