%% =========================================================================
%  ExportarFigurasComparativa.m
%
%  Convierte automáticamente las figuras generadas por
%  Comparativa_SMC_Kusznir_vs_STA_Propuesto.m a formato EPS vectorial en
%  ../Paper/Figures/ con los nombres esperados por el .tex del paper:
%
%     fig05.fig  ->  Comp_Baseline_Oscilaciones.eps
%                    (métricas de oscilación alpha, rho, ||v_perp||)
%     fig03.fig  ->  Comp_Baseline_Chattering.eps
%                    (acción de corrección del lazo interno, muestra
%                     el diente-de-sierra del chattering del SMC)
%
%  Toma automáticamente el corrida MÁS RECIENTE de ResultadosPaper/.
% =========================================================================

clear; close all;

carpetaFigs   = fullfile(pwd,'ResultadosPaper');
carpetaSalida = fullfile(pwd,'..','Paper','Figures');
if ~exist(carpetaSalida,'dir'); mkdir(carpetaSalida); end

% Encontrar la última corrida disponible
d = dir(fullfile(carpetaFigs,'Comp_SMC_vs_STA_*_fig05.fig'));
if isempty(d)
    error('No se encontraron figuras en %s. ¿Ya corriste la comparativa?', carpetaFigs);
end
[~,idx] = max([d.datenum]);
prefijo = regexprep(d(idx).name,'_fig05\.fig$','');
fprintf('Usando corrida: %s\n', prefijo);

% Mapeo verificado visualmente (fig05 = oscilaciones, fig03 = chattering)
mapa = { ...
    'fig05', 'Comp_Baseline_Oscilaciones', 'oscilaciones alpha/rho/vperp'; ...
    'fig03', 'Comp_Baseline_Chattering',   'accion de correccion (chattering)' };

for i = 1:size(mapa,1)
    figNum   = mapa{i,1};
    nomEps   = mapa{i,2};
    descr    = mapa{i,3};

    figPath = fullfile(carpetaFigs, [prefijo '_' figNum '.fig']);
    epsPath = fullfile(carpetaSalida, [nomEps '.eps']);

    fprintf('\n[%d/%d] %s\n', i, size(mapa,1), descr);
    fprintf('  cargar : %s\n', figPath);
    fprintf('  export : %s\n', epsPath);

    h = openfig(figPath, 'invisible');

    try
        exportgraphics(h, epsPath, 'ContentType','vector', ...
            'BackgroundColor','white');
        fprintf('  OK (exportgraphics ContentType=vector).\n');
    catch ME
        warning('exportgraphics fallo (%s). Reintento con print -depsc2.', ...
            ME.message);
        set(h,'PaperPositionMode','auto');
        print(h, epsPath, '-depsc2', '-painters');
        fprintf('  OK (print -depsc2 -painters).\n');
    end

    close(h);
end

fprintf('\nListo. Figuras exportadas en %s\n', carpetaSalida);
fprintf('   - Comp_Baseline_Oscilaciones.eps\n');
fprintf('   - Comp_Baseline_Chattering.eps\n');
