function R = sim_controlador(fh, cfg)
% SIM_CONTROLADOR  One run of controller fh on the plant in cfg.
%
%   R = sim_controlador(@controlador_paso, cfg)
%   R = sim_controlador(@controlador_omar, cfg)
%
% This is the loop of sim_payload_asta.m without the online plot: the same plant
% reading, the same correction of the delayed payload reading, the same reference
% evaluated at the simulator time, the same wind, and the same divergence cutoff.
% Both controllers run on exactly the same test bench.

planta = str2func(['planta_' cfg.planta.backend]);
es_rt = strcmp(cfg.planta.backend, 'simquad');
dt0 = cfg.planta.dt;
Nmax = ceil(1.2*cfg.planta.T/dt0) + 10;
r0 = referencia_carga(0, cfg.ref);
x0 = r0.p + [0; 0; cfg.planta.l];
if isfield(cfg.exp, 'x0_offset') && ~isempty(cfg.exp.x0_offset)
    x0 = x0 + cfg.exp.x0_offset(:);           % initial error (barrido_ganancias); without the field, no change
end
planta('iniciar', cfg, x0, r0.psi);

Z = @(n) nan(n, Nmax);
L = struct('t',Z(1),'dt',Z(1),'x',Z(3),'v',Z(3),'psi',Z(1),'xl',Z(3), ...
    'xd',Z(3),'vd',Z(3),'ad',Z(3),'jd',Z(3),'psid',Z(1), ...
    'h',Z(3),'hp',Z(3),'hs',Z(2),'hsd',Z(2),'ex',Z(3),'epsi',Z(1), ...
    'vc',Z(3),'ukh',Z(4),'vh',Z(2),'vt',Z(2),'s',Z(2),'nu',Z(2),'w',Z(2), ...
    'phith',Z(2),'theta',Z(4),'ad_on',Z(2),'sat',Z(2),'u',Z(4),'viento',Z(3), ...
    'xl_crudo',Z(3),'corr_xl',Z(1));

st = []; t_prev = []; t0 = []; k = 0; motivo = 'fin'; w_ant = [0;0;0]; stc = [];
if es_rt, tic; end
while true
    if es_rt
        while toc < dt0, end
        tic;
    end
    m = planta('leer');
    if ~m.ok, motivo = 'simulador_detenido'; break; end
    if isempty(t0), t0 = m.t; end
    if cfg.planta.usar_dt_real, t = m.t - t0; else, t = k*dt0; end
    if isempty(t_prev), dtk = dt0; else, dtk = t - t_prev; end
    if dtk <= 0, dtk = dt0; end
    t_prev = t;
    if t > cfg.planta.T || k >= Nmax, break; end
    k = k + 1;

    xl_crudo = m.xl; corr = false;
    if es_rt, [m, stc, corr] = corregir_lectura_carga(m, stc, t); end
    rr = referencia_carga(t, cfg.ref);
    [u, st, lg] = fh(cfg, st, m, rr, t, dtk);
    w = viento_de(t, cfg.viento.bloques);
    if any(w ~= w_ant), planta('viento', w); w_ant = w; end
    planta('comandar', u);

    L.t(k) = t; L.dt(k) = dtk; L.x(:,k) = m.x; L.v(:,k) = m.v; L.psi(k) = m.psi; L.xl(:,k) = m.xl;
    L.xd(:,k) = rr.p; L.vd(:,k) = rr.v; L.ad(:,k) = rr.a; L.jd(:,k) = rr.j; L.psid(k) = rr.psi;
    L.h(:,k) = lg.h; L.hp(:,k) = lg.hp; L.hs(:,k) = lg.hs_h; L.hsd(:,k) = lg.hsd_h;
    L.ex(:,k) = lg.ex; L.epsi(k) = lg.epsi; L.vc(:,k) = lg.v_c; L.ukh(:,k) = lg.uk_h;
    L.vh(:,k) = lg.v_h; L.vt(:,k) = lg.vt; L.s(:,k) = lg.s; L.nu(:,k) = lg.nu; L.w(:,k) = lg.w;
    L.phith(:,k) = lg.phith; L.theta(:,k) = lg.theta(:); L.ad_on(:,k) = lg.ad_on;
    L.sat(:,k) = lg.sat; L.u(:,k) = lg.u; L.viento(:,k) = w;
    L.xl_crudo(:,k) = xl_crudo; L.corr_xl(k) = corr;

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
R.n_corr_xl = sum(R.corr_xl == 1);
R.fecha = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end

function w = viento_de(t, B)
w = [0;0;0];
for i = 1:size(B,1)
    if t >= B(i,1) && t < B(i,2), w = B(i,3:5).'; end
end
end
