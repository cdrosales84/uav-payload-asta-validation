function [m, st, corregido] = corregir_lectura_carga(m, st, t, umbral)
% CORREGIR_LECTURA_CARGA  Detecta y corrige lecturas de la carga atrasadas un frame.
%
%   [m, st, corregido] = corregir_lectura_carga(m, st, t)
%   [m, st, corregido] = corregir_lectura_carga(m, st, t, umbral)
%
% Problema: quad1 y rope1 se leen por separado de la memoria compartida. A veces
% (~1 de cada 500 muestras) el simulador ya actualizo el dron pero la cuerda todavia
% no, y m.xl queda un frame atrasado. El vector cuerda h = x - x_l salta unos 3 cm y
% vuelve en la muestra siguiente; el estimador de hdot lo amplifica y aparece un pico
% en u_kh y en v_tilde.
%
% Deteccion: h cambia suavemente (|hdot| < ~0.6 m/s -> < 1 cm por paso). Se predice h
% por extrapolacion lineal con las dos muestras anteriores; si la medicion se aparta
% mas de 'umbral' (por defecto 1 cm), se reemplaza x_l por x - h_pred. Nunca se
% corrigen dos muestras seguidas, asi un cambio real de h no queda bloqueado.
%
% Uso en sim_payload_asta.m, justo antes de controlador_paso:
%     [m, stc, corr] = corregir_lectura_carga(m, stc, t);
%     L.corr_xl(k) = corr;                 % registrar que muestras se corrigieron
% (inicializar stc = [] antes del while y agregar 'corr_xl',Z(1) al struct L).

if nargin < 4 || isempty(umbral), umbral = 0.010; end
corregido = false;
h = m.x - m.xl;

if isempty(st)
    st.h1 = h; st.t1 = t; st.h2 = []; st.t2 = []; st.ultimo = false;
    return;
end

if ~isempty(st.h2) && ~st.ultimo
    d1 = st.t1 - st.t2;  d = t - st.t1;
    if d1 > 0 && d > 0
        h_pred = st.h1 + (st.h1 - st.h2) * (d / d1);
        if norm(h - h_pred) > umbral
            h = h_pred;
            m.xl = m.x - h;
            corregido = true;
        end
    end
end

st.ultimo = corregido;
st.h2 = st.h1;  st.t2 = st.t1;
st.h1 = h;      st.t1 = t;
end