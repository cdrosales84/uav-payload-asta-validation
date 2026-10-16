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
% Deteccion: se predice h por extrapolacion lineal con las dos ultimas mediciones
% ACEPTADAS; si la medicion se aparta mas de 'umbral' (1 cm por defecto), se
% reemplaza x_l por x - h_pred. Nunca se corrigen dos muestras seguidas, asi un
% cambio real de h no queda bloqueado.
%
% v2 (15-oct-2026). La version anterior guardaba en la historia el valor CORREGIDO y
% extrapolaba a partir de el. Con el periodo irregular del lazo en tiempo real
% (dt de 16 y 33 ms alternados, d/d1 = 2) el error de una prediccion se duplicaba en
% la siguiente: el corrector entraba en un ciclo que corregia una de cada dos
% muestras buenas, h crecia hasta 6-12 m y la corrida se cortaba por "divergencia"
% con cualquier controlador, aunque las lecturas crudas eran suaves. Ahora la
% historia solo contiene mediciones crudas aceptadas, y la extrapolacion se limita
% a 1.5 veces el intervalo de la historia.

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
        r = min(d / d1, 1.5);                       % no amplificar el error de la historia
        h_pred = st.h1 + (st.h1 - st.h2) * r;
        if norm(h - h_pred) > umbral
            m.xl = m.x - h_pred;                    % se corrige la medicion entregada,
            corregido = true;                       % pero la historia no se toca
        end
    end
end

st.ultimo = corregido;
if ~corregido                                       % historia: solo mediciones crudas aceptadas
    st.h2 = st.h1;  st.t2 = st.t1;
    st.h1 = h;      st.t1 = t;
end
end
