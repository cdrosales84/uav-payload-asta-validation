function M = metricas_payload(R)
% METRICAS_PAYLOAD  Metricas de seguimiento y de oscilacion a partir de la verdad del simulador.
%
%   M = metricas_payload(R)       con R = sim_payload_asta(cfg)
%
% Las velocidades de la carga NO se toman del estimador: se obtienen derivando la
% posicion de la carga que entrega el simulador (ultima particula de la cuerda) con
% diferencias centrales sobre el tiempo real del simulador. El estimador h_p se valida
% contra esa derivada (M.est_*).

t = R.t; N = numel(t);
cfg = R.cfg;
sel = t >= cfg.met.t_inicio;

% velocidad verdadera de la carga y de hdot (diferencias centrales, tiempo no uniforme)
vl = dercentral(R.xl, t);
hdot = R.v - vl;

% seguimiento de la carga (errores sin filtrar)
ex  = R.xd - R.xl;
evl = R.vd - vl;
M.rmse_ex  = rmsn(ex(:,sel));
M.rmse_evl = rmsn(evl(:,sel));
M.rmse_ex_xyz = sqrt(mean(ex(:,sel).^2, 2)).';
M.rmse_epsi = sqrt(mean(wrap(R.psid(sel) - R.psi(sel)).^2));

% oscilacion
h = R.h; rho = sqrt(h(1,:).^2 + h(2,:).^2);
alpha = atan2(rho, h(3,:));                         % h = x - x_l  ->  h_z > 0 con la carga abajo
hu = h ./ sqrt(sum(h.^2,1));
vrel = vl - R.v;                                    % v_L - v_U = -hdot
vperp = vrel - hu .* sum(vrel.*hu, 1);
nvp = sqrt(sum(vperp.^2,1));
swing = sqrt(sum((h(1:2,:) - R.hs).^2, 1));         % desvio respecto de la inclinacion cuasiestatica
M.rms_alpha = rms1(alpha(sel));  M.max_alpha = max(alpha(sel));
M.rms_rho   = rms1(rho(sel));    M.max_rho   = max(rho(sel));
M.rms_vperp = rms1(nvp(sel));    M.max_vperp = max(nvp(sel));
M.rms_swing = rms1(swing(sel));  M.max_swing = max(swing(sel));

% validacion del estimador de hdot contra la verdad del simulador (canales horizontales)
se = t >= cfg.met.t_inicio_est;
eh = R.hp(1:2,se) - hdot(1:2,se);
M.est_rms_err  = rmsn(eh);
M.est_rms_true = rmsn(hdot(1:2,se));
[M.est_retardo, M.est_ganancia] = retardo_ganancia(R.hp(1:2,se), hdot(1:2,se), t(se));

% lazo interno
M.rms_vt    = sqrt(mean(R.vt(:,sel).^2, 2)).';
M.rms_s     = sqrt(mean(R.s(:,sel).^2, 2)).';
M.rms_nu    = sqrt(mean(R.nu(:,sel).^2, 2)).';
M.frac_sat  = mean(any(R.sat(:,sel),1));
M.theta_fin = R.theta(:,end).';                     % [th1_x th2_x th1_y th2_y]
du = diff(R.u(1:2,sel), 1, 2) ./ diff(t(sel));
M.rms_du    = sqrt(mean(du.^2, 2)).';               % diagnostico de chattering
M.tv_u      = sum(abs(diff(R.u(1:2,sel), 1, 2)), 2).';

% corrida
M.completo = R.completo; M.t_final = t(end);
M.dt_medio = mean(R.dt); M.dt_max = max(R.dt);

% series para figuras
M.serie = struct('t',t,'vl',vl,'hdot',hdot,'alpha',alpha,'rho',rho,'vperp',nvp,'swing',swing);
end

%% ------------------------------------------------------------------------
function d = dercentral(X, t)
N = numel(t); d = nan(size(X));
if N < 3, d(:) = 0; return; end
d(:,2:N-1) = (X(:,3:N) - X(:,1:N-2)) ./ (t(3:N) - t(1:N-2));
d(:,1) = (X(:,2) - X(:,1)) / (t(2) - t(1));
d(:,N) = (X(:,N) - X(:,N-1)) / (t(N) - t(N-1));
end

function r = rmsn(X), r = sqrt(mean(sum(X.^2, 1))); end
function r = rms1(x), r = sqrt(mean(x.^2)); end
function a = wrap(a), a = atan2(sin(a), cos(a)); end

function [tau, g] = retardo_ganancia(y, x, t)
% retardo (s) que maximiza la correlacion entre la estimacion y y la verdad x, y ganancia
dt = mean(diff(t)); best = -inf; lag = 0;
for L = 0:15
    a = y(:, 1+L:end); b = x(:, 1:end-L);
    c = sum(a(:).*b(:)) / sqrt(sum(a(:).^2)*sum(b(:).^2));
    if c > best, best = c; lag = L; end
end
tau = lag*dt;
a = y(:, 1+lag:end); b = x(:, 1:end-lag);
g = sum(a(:).*b(:)) / sum(b(:).^2);
end
