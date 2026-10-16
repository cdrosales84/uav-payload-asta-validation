function M = metricas_payload(R)
% METRICAS_PAYLOAD  Tracking and swing metrics from the simulator ground truth.
%
%   M = metricas_payload(R)       with R = sim_payload_asta(cfg)
%
% Payload velocities are not taken from the estimator: they are obtained by
% differentiating the payload position returned by the simulator (last rope
% particle) with central differences on the simulator's real time. The h_p
% estimator is validated against that derivative (M.est_*).

t = R.t; N = numel(t);
cfg = R.cfg;
sel = t >= cfg.met.t_inicio;

% true payload velocity and hdot (central differences, nonuniform time)
vl = dercentral(R.xl, t);
hdot = R.v - vl;

% payload tracking (unfiltered errors)
ex  = R.xd - R.xl;
evl = R.vd - vl;
M.rmse_ex  = rmsn(ex(:,sel));
M.rmse_evl = rmsn(evl(:,sel));
M.rmse_ex_xyz = sqrt(mean(ex(:,sel).^2, 2)).';
M.rmse_epsi = sqrt(mean(wrap(R.psid(sel) - R.psi(sel)).^2));

% swing
h = R.h; rho = sqrt(h(1,:).^2 + h(2,:).^2);
alpha = atan2(rho, h(3,:));                         % h = x - x_l  ->  h_z > 0 with the payload below
hu = h ./ sqrt(sum(h.^2,1));
vrel = vl - R.v;                                    % v_L - v_U = -hdot
vperp = vrel - hu .* sum(vrel.*hu, 1);
nvp = sqrt(sum(vperp.^2,1));
swing = sqrt(sum((h(1:2,:) - R.hs).^2, 1));         % deviation from the quasi-static tilt
M.rms_alpha = rms1(alpha(sel));  M.max_alpha = max(alpha(sel));
M.rms_rho   = rms1(rho(sel));    M.max_rho   = max(rho(sel));
M.rms_vperp = rms1(nvp(sel));    M.max_vperp = max(nvp(sel));
M.rms_swing = rms1(swing(sel));  M.max_swing = max(swing(sel));

% hdot estimator validation against the simulator ground truth (horizontal channels)
se = t >= cfg.met.t_inicio_est;
eh = R.hp(1:2,se) - hdot(1:2,se);
M.est_rms_err  = rmsn(eh);
M.est_rms_true = rmsn(hdot(1:2,se));
[M.est_retardo, M.est_ganancia] = retardo_ganancia(R.hp(1:2,se), hdot(1:2,se), t(se));

% inner loop
M.rms_vt    = sqrt(mean(R.vt(:,sel).^2, 2)).';
M.rms_s     = sqrt(mean(R.s(:,sel).^2, 2)).';
M.rms_nu    = sqrt(mean(R.nu(:,sel).^2, 2)).';
M.frac_sat  = mean(any(R.sat(:,sel),1));
M.theta_fin = R.theta(:,end).';                     % [th1_x th2_x th1_y th2_y]
ts = t(sel); dts = diff(ts); ok = dts > 0;         % the simulator may repeat a sample time
du = diff(R.u(1:2,sel), 1, 2); du = du(:,ok) ./ dts(ok);
M.rms_du    = sqrt(mean(du.^2, 2)).';               % chattering diagnostic
M.tv_u      = sum(abs(diff(R.u(1:2,sel), 1, 2)), 2).';

% run
M.completo = R.completo; M.t_final = t(end);
M.dt_medio = mean(R.dt); M.dt_max = max(R.dt);

% series for figures
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
% delay (s) that maximizes the correlation between estimate y and truth x, and the gain
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
