function test_referencias()
% TEST_REFERENCIAS  Verifica que velocidad, aceleracion, jerk y psi_dot de
% referencia_carga coincidan con la derivada numerica de la senal anterior.
% Ademas muestra el error de la aceleracion de la lemniscata en ReferenciasCarga_ST.

tipos = {'circulo', 'lemniscata'};
cfg = cfg_paper();
t = 0:1e-3:80;
for i = 1:numel(tipos)
    cfg.ref.tipo = tipos{i};
    r = referencia_carga(t, cfg.ref);
    e = [maxerr(r.p, r.v, t), maxerr(r.v, r.a, t), maxerr(r.a, r.j, t), ...
         maxerr(unwrap(r.psi), r.dpsi, t)];
    fprintf('%-11s  max|d/dt - analitica|: v %.1e  a %.1e  j %.1e  psi_dot %.1e\n', tipos{i}, e);
    assert(all(e < 1e-4), 'test_referencias:%s', tipos{i});
end

% Comparacion con la expresion de ReferenciasCarga_ST (casos 4 y 5)
cfg.ref.tipo = 'lemniscata'; L = cfg.ref.lemn;
r = referencia_carga(t, cfg.ref);
xdd_viejo = -L.a*L.c^2*2*sin(L.c*t) - L.a*L.c^2*2*cos(L.c*t);
fprintf('ReferenciasCarga_ST, lemniscata: max|xdd_viejo - xdd| = %.3f m/s^2 (|xdd| max = %.3f)\n', ...
    max(abs(xdd_viejo - r.a(1,:))), max(abs(r.a(1,:))));
fprintf('test_referencias: OK\n');
end

function e = maxerr(f, df, t)
d = (f(:,3:end) - f(:,1:end-2)) ./ (t(3:end) - t(1:end-2));
e = max(max(abs(d - df(:,2:end-1))));
end
