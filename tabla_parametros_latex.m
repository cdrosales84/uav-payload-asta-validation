function tabla_parametros_latex(cfg, archivo)
% TABLA_PARAMETROS_LATEX  Writes the controller parameter table from cfg,
% so the manuscript table always matches the values used in the code.

dt = cfg.planta.dt;
fid = fopen(archivo, 'w');
fprintf(fid, '%% Generado por tabla_parametros_latex.m desde cfg_paper.m (%s)\n', cfg.version);
fprintf(fid, '\\begin{tabular}{llll}\n\\toprule\nBloque & S\\''imbolo & Valor & Ec. \\\\\n\\midrule\n');
L = @(b, s, v, e) fprintf(fid, '%s & %s & %s & %s \\\\\n', b, s, v, e);
L('Cinem\''atico', '$K_{\mathrm{traj}}$', diagstr(cfg.ext.Ktraj), '\eqref{eq:base_reference}');
L('', '$K_h$', diagstr(cfg.ext.Kh), '\eqref{eq:swing_compensation}');
L('', '$K_{dh}$', diagstr(cfg.ext.Kdh), '\eqref{eq:swing_compensation}');
L('', '$T_s$', sprintf('%.1f s', cfg.ext.Ts), '\eqref{eq:final_reference}');
L('', '$k_\psi,\ k_\psi^t$', sprintf('%.2f, %.2f', cfg.ext.kpsi, cfg.ext.kpsit), '\eqref{eq:yaw_reference}');
L('Estimador', '$\varepsilon_1,\ K_1,\ \tau_h$', sprintf('%.2f, %s, %.3f s', cfg.est.eps1, diagstr(cfg.est.K1), cfg.est.tau_h), '\eqref{eq:h_raw}--\eqref{eq:h_p_filter}');
L('Interno ($n=2$)', '$\beta$', sprintf('%.2f', cfg.int.beta), 'superficie');
L('', '$K_1,\ K_2$', sprintf('%s, %s', vecstr(cfg.int.K1), vecstr(cfg.int.K2)), '\eqref{eq:AccionControl}');
if cfg.int.phi_bl > 0
    L('', '$\phi$ (capa l\''imite)', sprintf('%.3f m/s', cfg.int.phi_bl), '\eqref{eq:AccionControl}');
end
L('', '$u_{\max}$', sprintf('%.1f m/s', cfg.int.u_max), 'implementaci\''on');
L('Adaptaci\''on', '$\boldsymbol{\theta}_0$', vecstr(cfg.ad.theta0), 'nominal');
L('', '$\Gamma,\ \sigma$', sprintf('%s, %.3g', diagstr(cfg.ad.Gamma), cfg.ad.sigma), '\eqref{eq:ajuste_theta}');
L('', '$\bar\nu,\ s_{\mathrm{on}},\ t_{\mathrm{on}}$', sprintf('%.1f, %.3f, %.1f s', cfg.ad.nu_bar, cfg.ad.s_on, cfg.ad.t_on), '\eqref{eq:ajuste_theta}');
L('', '$\Theta$', sprintf('$[%.2f,%.1f]\\times[%.1f,%.1f]$', cfg.ad.Theta_min(1), cfg.ad.Theta_max(1), cfg.ad.Theta_min(2), cfg.ad.Theta_max(2)), '\eqref{eq:ajuste_theta}');
f = {'tau_ex','$\tilde{\mathbf{x}}$'; 'tau_epsi','$e_\psi$'; 'tau_vc','$\mathbf{u}_k$'; 'tau_vdot','$\dot{\mathbf{v}}_d$'};
for i = 1:size(f,1)
    tau = cfg.filt.(f{i,1});
    if tau > 0
        L('Filtro', f{i,2}, sprintf('$\\tau = %.0f$ ms ($%.2f\\,T_c$)', 1e3*tau, tau/dt), 'implementaci\''on');
    end
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);
end

function s = diagstr(K), s = ['diag(' strjoin(arrayfun(@(x) sprintf('%g', x), diag(K).', 'UniformOutput', false), ', ') ')']; end
function s = vecstr(v),  s = ['[' strjoin(arrayfun(@(x) sprintf('%g', x), v(:).', 'UniformOutput', false), ', ') ']$^\top$']; end
