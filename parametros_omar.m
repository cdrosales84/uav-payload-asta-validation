function P = parametros_omar(cfg, varargin)
% PARAMETROS_OMAR  Parameters of the Omar baseline (Appl. Sci. 2022) for cfg.omar.
%
%   P = parametros_omar(cfg)                      % uses omar_ajuste.mat if it exists
%   P = parametros_omar(cfg, 'K', 0.6, 'tau_TL', 0.1, 'signo', +1)
%   P = parametros_omar(cfg, 'activo', false)    % PX4-type tracker only (no ASC)
%
% Values:
%   K       gain of Eqs. (1)-(2). Omar uses K_sx = K_sy = 0.6 (Sec. 5.1),
%           multiplied by L in the equation.
%   tau     delay t_sx = t_sy [s]. The paper says it is chosen to minimize ISH
%           (Eq. 4) and only uses T_L (Eq. 3). The useful phase lag is the TOTAL,
%           the delay plus the lag of the UAV position and velocity loops (~0.28 T_L
%           at the pendulum frequency with Kp = 0.95). That is why the added delay
%           is small: without tuning, tau = 0.1 T_L and sign +1 (hover test on the
%           surrogate plant: ISH -51% relative to no ASC).
%           ajustar_omar.m searches by ISH (sign, K, tau) and saves
%           omar_ajuste.mat, which this function loads automatically.
%   Kp      PX4 position loop, default values MPC_XY_P = 0.95, MPC_Z_P = 1.0.

op = struct('K', [], 'tau_TL', [], 'signo', [], 'activo', true, 'archivo', '');
for i = 1:2:numel(varargin), op.(varargin{i}) = varargin{i+1}; end

g = cfg.planta.g; l = cfg.planta.l;
P.TL = 2*pi / sqrt(g/l);                         % Eq. (3)
P.K = 0.6;                                       % Omar, Sec. 5.1
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
