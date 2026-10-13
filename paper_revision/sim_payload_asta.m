function R = sim_payload_asta(cfg)
% SIM_PAYLOAD_ASTA  Una corrida del controlador en cascada del manuscrito revisado.
%
%   R = sim_payload_asta(cfg)      con cfg = cfg_paper(escenario)
%
% El lazo lee la planta, evalua la referencia en el tiempo del simulador, llama a
% controlador_paso (que contiene todas las ecuaciones de la Secc. 2) y envia el
% comando. Devuelve todas las senales en R; R.cfg es la configuracion usada.
% Con cfg.planta.grafico_en_linea = true, cada grafico_cada muestras dibuja el UAV
% (Nquadrotor), la cuerda y la carga en una figura 3D, como el lazo de
% simulations/CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m.
%
% [CORR] Con el simulador en tiempo real (backend 'simquad'), la posicion de la carga
% a veces llega con un frame de atraso respecto de la del UAV (quad1 y rope1 se leen
% por separado). corregir_lectura_carga detecta esas muestras y reemplaza x_l por la
% prediccion antes de que entren al controlador. R.xl guarda el valor corregido,
% R.xl_crudo la lectura original y R.corr_xl marca las muestras corregidas.

planta = str2func(['planta_' cfg.planta.backend]);
es_rt  = strcmp(cfg.planta.backend, 'simquad');      % co-simulacion en tiempo real
dt0    = cfg.planta.dt;
Nmax   = ceil(1.2*cfg.planta.T/dt0) + 10;

r0 = referencia_carga(0, cfg.ref);
x0 = r0.p + [0; 0; cfg.planta.l];                     % carga en la referencia, cable vertical
planta('iniciar', cfg, x0, r0.psi);
if grafica_en_linea(cfg), graficar_en_linea('iniciar', cfg); end

Z = @(n) nan(n, Nmax);
L = struct('t',Z(1),'dt',Z(1),'x',Z(3),'v',Z(3),'psi',Z(1),'xl',Z(3), ...
    'xd',Z(3),'vd',Z(3),'ad',Z(3),'jd',Z(3),'psid',Z(1), ...
    'h',Z(3),'hp',Z(3),'hs',Z(2),'hsd',Z(2),'ex',Z(3),'epsi',Z(1), ...
    'vc',Z(3),'ukh',Z(4),'vh',Z(2),'vt',Z(2),'s',Z(2),'nu',Z(2),'w',Z(2), ...
    'phith',Z(2),'theta',Z(4),'ad_on',Z(2),'sat',Z(2),'u',Z(4),'viento',Z(3), ...
    'xl_crudo',Z(3),'corr_xl',Z(1));                  % [CORR] 1) nuevos registros

st = []; t_prev = []; t0 = []; k = 0; motivo = 'fin'; w_ant = [0;0;0];
stc = [];                                             % [CORR] 2) estado del corrector
if es_rt, tic; end
while true
    if es_rt
        while toc < dt0, end
        tic;
    end
    m = planta('leer');
    if ~m.ok, motivo = 'simulador_detenido'; break; end
    if isempty(t0), t0 = m.t; end
    if cfg.planta.usar_dt_real
        t = m.t - t0;
    else
        t = k*dt0;
    end
    if isempty(t_prev), dtk = dt0; else, dtk = t - t_prev; end
    if dtk <= 0, dtk = dt0; end                       % el simulador no avanzo: se usa el nominal
    t_prev = t;
    if t > cfg.planta.T || k >= Nmax, break; end
    k = k + 1;

    % [CORR] 3) corregir la lectura de la carga ANTES de usar m (controlador,
    % registro y corte por divergencia). Solo en tiempo real: la planta sustituta
    % no tiene este problema.
    xl_crudo = m.xl;
    corr = false;
    if es_rt
        [m, stc, corr] = corregir_lectura_carga(m, stc, t);
    end

    rr = referencia_carga(t, cfg.ref);
    if grafica_en_linea(cfg), graficar_en_linea('paso', m, rr.p, t); end
    [u, st, lg] = controlador_paso(cfg, st, m, rr, t, dtk);

    w = viento(t, cfg.viento.bloques);
    if any(w ~= w_ant), planta('viento', w); w_ant = w; end
    planta('comandar', u);

    L.t(k) = t; L.dt(k) = dtk; L.x(:,k) = m.x; L.v(:,k) = m.v; L.psi(k) = m.psi; L.xl(:,k) = m.xl;
    L.xd(:,k) = rr.p; L.vd(:,k) = rr.v; L.ad(:,k) = rr.a; L.jd(:,k) = rr.j; L.psid(k) = rr.psi;
    L.h(:,k) = lg.h; L.hp(:,k) = lg.hp; L.hs(:,k) = lg.hs_h; L.hsd(:,k) = lg.hsd_h;
    L.ex(:,k) = lg.ex; L.epsi(k) = lg.epsi; L.vc(:,k) = lg.v_c; L.ukh(:,k) = lg.uk_h;
    L.vh(:,k) = lg.v_h; L.vt(:,k) = lg.vt; L.s(:,k) = lg.s; L.nu(:,k) = lg.nu; L.w(:,k) = lg.w;
    L.phith(:,k) = lg.phith; L.theta(:,k) = lg.theta; L.ad_on(:,k) = lg.ad_on; L.sat(:,k) = lg.sat;
    L.u(:,k) = lg.u; L.viento(:,k) = w;
    L.xl_crudo(:,k) = xl_crudo; L.corr_xl(k) = corr;  % [CORR] 4) registro

    % corte por divergencia (Fig. 6a)
    alpha = atan2(norm(lg.h(1:2)), lg.h(3));
    if alpha > cfg.exp.alpha_abort || norm(rr.p - m.xl) > cfg.exp.ex_abort
        motivo = 'divergencia'; break;
    end
end
planta('cerrar');

f = fieldnames(L);
for i = 1:numel(f), L.(f{i}) = L.(f{i})(:, 1:k); end
R = L;
R.cfg = cfg;
R.motivo = motivo;
R.completo = strcmp(motivo, 'fin');
R.n_corr_xl = sum(R.corr_xl == 1);                    % [CORR] cantidad de lecturas corregidas
R.fecha = datestr(now, 'yyyy-mm-dd HH:MM:SS');
R.entorno = entorno();
end

%% ------------------------------------------------------------------------
function w = viento(t, B)
w = [0;0;0];
for i = 1:size(B,1)
    if t >= B(i,1) && t < B(i,2), w = B(i,3:5).'; end
end
end

function si = grafica_en_linea(cfg)
si = isfield(cfg.planta, 'grafico_en_linea') && cfg.planta.grafico_en_linea;
end

function e = entorno()
if exist('OCTAVE_VERSION', 'builtin')
    e = ['Octave ' OCTAVE_VERSION];
else
    e = ['MATLAB ' version];
end
e = [e ' / ' computer];
end