function [R, fallidas] = sim_con_reintento(fh, cfg, n_max, arch_base)
% SIM_CON_REINTENTO  sim_controlador, retried if SimQuad1.exe stops responding.
%
%   [R, fallidas] = sim_con_reintento(@controlador_paso, cfg, 2, 'raw/E4_asta_rep1')
%
% A run is retried only when it ended with reason 'simulador_detenido', that is,
% when simulation1 stopped returning data (the simulator closed or hung).
% Before the retry, SimQuad1.exe is killed and planta_simquad starts it again.
% A run that ends with 'divergencia' is NOT retried: that is a controller result
% and it goes into the table as an incomplete run.
%
% Each failed attempt is saved as <arch_base>_intento<i>.mat (if arch_base is
% not empty) so it can be inspected. fallidas = number of discarded attempts.

if nargin < 3 || isempty(n_max), n_max = 2; end
if nargin < 4, arch_base = ''; end
fallidas = 0;
for intento = 1:(n_max + 1)
    R = sim_controlador(fh, cfg);
    if ~strcmp(R.motivo, 'simulador_detenido') || ~strcmp(cfg.planta.backend, 'simquad')
        return;
    end
    fallidas = fallidas + 1;
    fprintf('\n    simulador detenido en t = %.1f s (intento %d). ', R.t(end), intento);
    if ~isempty(arch_base)
        Rf = R; %#ok<NASGU>
        save(sprintf('%s_intento%d.mat', arch_base, intento), 'Rf', '-v7');
    end
    if intento <= n_max
        fprintf('Reiniciando SimQuad1.exe y repitiendo ... ');
        reiniciar_simquad();
    end
end
fprintf('Sin exito despues de %d intentos. ', n_max + 1);
end

function reiniciar_simquad()
try, quad1(1, [0 0 0 0]); catch, end
[~, ~] = system('taskkill /F /IM SimQuad1.exe');
pause(3);                                     % planta_simquad('iniciar') starts it again
end
