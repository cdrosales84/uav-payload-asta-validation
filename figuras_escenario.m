function figuras_escenario(R, M, prefijo)
% FIGURAS_ESCENARIO  Figures for one run (manuscript figures and validation figures).
%   1 payload position and yaw vs reference        4 inner-loop velocity errors
%   2 alpha, rho, and |v_perp| (simulator truth)   5 theta_hat (adaptation)
%   3 position and yaw errors                      6 hdot estimator validation

t = R.t; S = M.serie; lw = 1.2;
f = figure('Visible', 'off');
lab = {'x','y','z'};
for i = 1:3
    subplot(4,1,i); plot(t, R.xd(i,:), '--', t, R.xl(i,:), 'LineWidth', lw); grid on;
    ylabel([lab{i} ' [m]']);
end
wr = @(a) atan2(sin(a), cos(a));
subplot(4,1,4); plot(t, wr(R.psid), '--', t, wr(R.psi), 'LineWidth', lw); grid on; ylabel('\psi [rad]'); xlabel('t [s]');
legend('referencia', 'carga / UAV'); guardar(f, [prefijo '_1_posicion']);

f = figure('Visible', 'off');
subplot(3,1,1); plot(t, S.alpha, 'LineWidth', lw); grid on; ylabel('\alpha [rad]');
subplot(3,1,2); plot(t, S.rho, t, S.swing, 'LineWidth', lw); grid on; ylabel('[m]'); legend('\rho', '||h_h - h^*_h||');
subplot(3,1,3); plot(t, S.vperp, 'LineWidth', lw); grid on; ylabel('||v_\perp|| [m/s]'); xlabel('t [s]');
guardar(f, [prefijo '_2_oscilacion']);

f = figure('Visible', 'off'); ex = R.xd - R.xl;
for i = 1:3, subplot(4,1,i); plot(t, ex(i,:), 'LineWidth', lw); grid on; ylabel(['e_' lab{i} ' [m]']); end
subplot(4,1,4); plot(t, R.epsi, 'LineWidth', lw); grid on; ylabel('e_\psi [rad]'); xlabel('t [s]');
guardar(f, [prefijo '_3_errores']);

f = figure('Visible', 'off');
for i = 1:2
    subplot(3,1,i); plot(t, R.vt(i,:), t, R.s(i,:), 'LineWidth', lw); grid on;
    ylabel(['canal ' lab{i}]); legend('v_{tilde}', 's');
end
subplot(3,1,3); plot(t, R.nu, 'LineWidth', lw); grid on; ylabel('\nu'); xlabel('t [s]');
guardar(f, [prefijo '_4_lazo_interno']);

f = figure('Visible', 'off');
subplot(2,1,1); plot(t, R.theta([1 3],:), 'LineWidth', lw); grid on; ylabel('\theta_1 = 1/\kappa_u'); legend('x', 'y');
subplot(2,1,2); plot(t, R.theta([2 4],:), 'LineWidth', lw); grid on; ylabel('\theta_2 = \kappa_v/\kappa_u'); xlabel('t [s]');
guardar(f, [prefijo '_5_theta']);

f = figure('Visible', 'off');
for i = 1:2
    subplot(2,1,i); plot(t, S.hdot(i,:), t, R.hp(i,:), 'LineWidth', lw); grid on;
    ylabel(['dh_' lab{i} '/dt [m/s]']); legend('verdad del simulador', 'estimador');
end
xlabel('t [s]'); guardar(f, [prefijo '_6_estimador']);
end

function guardar(f, nombre)
print(f, [nombre '.png'], '-dpng', '-r130');
if ~exist('OCTAVE_VERSION', 'builtin'), savefig(f, [nombre '.fig']); end
close(f);
end
