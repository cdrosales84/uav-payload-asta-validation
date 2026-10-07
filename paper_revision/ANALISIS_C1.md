# Análisis del código frente al Comentario 1 del Revisor 1

Fecha: 07-oct-2026. Repositorio: `C:\Repositorios\uav-payload-asta-validation` (último commit `a2827fe`, 06-oct).

## 1. Qué pide el revisor

El código público implementa un controlador distinto del analizado en el manuscrito, en cinco puntos:

1. realimentación de posición saturada y un término de velocidad de la carga ausente de la Ec. (8);
2. el signo de la referencia cuasiestática del cable difiere del de la Ec. (12);
3. filtros adicionales, saturación e integral con fuga;
4. el compensador interno actúa solo en los canales horizontales;
5. el manuscrito adapta κu, κv, mientras que el código adapta K1, K2 del super-twisting.

Conclusión del revisor: las simulaciones no validan las Ecs. (29)–(40) ni el Teorema 2.

## 2. Estado del repositorio

| Archivo | Fecha | Qué es |
|---|---|---|
| `CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m` | jul–sep | El script que vio el revisor (figuras de la versión enviada) |
| `controller.m` | 22-sep | Versión "incremental" (pasos 2–6), validada en SimQuad con K1/K2 = 1.0/1.5 |
| `controller_mejorado.m` | 23-sep | Sin filtros extra, ST en x, y, z, Kh/Kdh = 3.0/0.3 |
| `Comparativa_SMC_Kusznir_vs_STA_Propuesto.m` | jul | Benchmark del manuscrito |
| `CompensacionConsiderandoCarga_STMejorado_Claude.m` | abr | Escenario con viento |

**La respuesta a C1 que ya está en `ResponsesReviewers.tex` describe archivos que no existen en el repo:** `sim_payload_asta.m`, `cfg_paper.m`, `run_paper_experiments.m`, `EQUATION_MAP.md`. Tampoco hay ningún script que implemente la ley de adaptación de θ: en `controller_mejorado.m` el flag `UsarAdaptacionST = true` no tiene código detrás.

## 3. Los cinco puntos del revisor, en cada versión del código

| Punto | Script enviado (`8_Graficas`) | `controller.m` | `controller_mejorado.m` | `paper_revision/` (nuevo) |
|---|---|---|---|---|
| 1. Posición saturada | `Ktraj·tanh(Ktrajt·x̃)` | lineal | lineal | lineal |
| 1. Término de velocidad de la carga | `Kvload·tanh(·ẋ̃)` | quitado | quitado | quitado |
| — Feedforward ḣ* en v_base | no | no | no | **sí, analítico** |
| 2. Signo de h* | + (correcto) | + | + | + (y el paper se corrige) |
| 2. ḣ* | diferencia numérica filtrada | ídem | diferencia de ẍ_d, sin filtro | **(l/g)·jerk analítico** |
| 3. Saturación de v_comp | `comp_max·tanh` | quitada | quitada | quitada |
| 3. Integral con fuga | sí | no | no | no |
| 3. Filtros | x̃, e_ψ, u_k, v̇_d, ḣ*, estimador STA | x̃ (46 ms), e_ψ (70 ms), u_k (46 ms), v̇_d (17 ms), ḣ* | solo v̇_d | **solo v̇_d (50 ms), declarado** |
| 3. Estimador de ḣ | diferenciador STA (no es Ecs. 9–11) | Ecs. 9–11, propagado a k+1 | ídem | ídem, filtro discretizado exacto |
| 4. Canales del compensador | x, y | x, y | x, y, z | x, y (marco de yaw), declarado |
| 5. Qué se adapta | K1, K2 | nada | nada | **θ = [1/κu, κv/κu] por canal** |
| — Inversión de modelo φθ̂ | no (u = u_c + corrección ST) | sí, θ fijo | sí, θ fijo | sí, θ adaptado |
| — Ganancias | Kh/Kdh 1.8/0.6, K1/K2 0.5/0.75 adaptadas | 1.8/0.6, 1.0/1.5 | 3.0/0.3, 1.0/1.5 | **3.0/0.3, 0.5/0.75** |
| — Métricas de velocidad | estimador | estimador (y `v_payload` antes de actualizarlo) | estimador | **verdad del simulador** |
| — Período | `deltat` fijo | fijo | fijo | **tiempo del simulador** |

Ninguna versión anterior responde los puntos 4 y 5 juntos, que son el núcleo del comentario.

## 4. El código nuevo (`paper_revision/`)

- `controlador_paso.m` contiene todas las ecuaciones de la Sección 2 en orden, con su etiqueta. Fuera de ese archivo no hay lógica de control.
- `cfg_paper.m` concentra todos los parámetros y los escenarios E1–E7; la configuración se guarda dentro de cada `.mat`.
- La tabla de parámetros del manuscrito se genera desde `cfg_paper.m` (`tabla_parametros_latex.m`), así no puede volver a desincronizarse.
- `planta_simquad.m` encapsula los mex y `SimQuad1.exe`. `planta_sustituta.m` es un modelo simplificado con la misma interfaz, solo para probar sin Windows.
- `referencia_carga.m` entrega posición, velocidad, aceleración y jerk analíticos, evaluados en el tiempo real del simulador.

Detalles en `README.md` y `EQUATION_MAP.md`.

## 5. Resultados de la prueba (Octave 8.4, planta sustituta)

Todo el pipeline corre de punta a punta: 14 escenarios, CSV, tablas LaTeX y figuras. La planta sustituta tiene θ verdadero = [0.3, 1.0], un retardo de actitud de 50 ms, un retardo de cómputo de una muestra, un cable elástico y arrastre sobre la carga. **Ninguno de estos números va al paper**; sirven para verificar el código y anticipar lo que puede pasar en SimQuad.

### 5.1 La implementación es exacta

Sin arrastre sobre la carga, el error permanente de la carga en el círculo es **4 mm**. Con arrastre es 0.18 m, y ese valor no depende del lazo interno (0.181 m con cualquier K1/K2). El feedforward ḣ* y el lazo externo hacen lo que dice la teoría; el error residual viene de fuerzas que el lazo cinemático, sin acción integral, no rechaza. En SimQuad la carga tiene arrastre de cubo (C_d = 1.05), así que hay que esperar un error residual del mismo tipo.

### 5.2 Ciclo límite del super-twisting

Con K1/K2 = 1.0/1.5 la superficie s oscila con período ≈ 0.38 s y RMS ≈ 0.09 m/s. Es el mismo fenómeno que en SimQuad (0.8 s): el STA con retardo y dinámica de actitud no modelada no converge a s = 0.

| K1 / K2 | φ (capa límite) | desvío de s | RMS(u̇_x) | adaptación activa |
|---|---|---|---|---|
| 1.0 / 1.5 | 0 | 0.090 | 7.6 | 27 % del tiempo |
| 0.5 / 0.75 | 0 | 0.030 | 2.3 | 99 % |
| 0.5 / 0.75 | 0.05 | 0.002 | 0.17 | 100 % |
| 1.0 / 1.5 | 0.05 | 0.006 | 0.29 | 100 % |

El doble de K1/K2 era un parche para la inestabilidad del lazo externo con Kh/Kdh = 1.8/0.6, que no cumplen K_h > (1+K_dh)K_traj. Con 3.0/0.3 esa condición se cumple, y por eso los valores por defecto vuelven a 0.5/0.75. La capa límite queda como opción (`cfg.int.phi_bl`): si se usa, hay que declararla en el paper.

### 5.3 Qué puede y qué no puede mostrar la adaptación

| Caso | θ̂ final [θ1x θ2x θ1y θ2y] | Comentario |
|---|---|---|
| Círculo, θ̂(0) = [0.2, 0.5] | [0.19 1.00 0.20 0.34] | θ2,x converge a 1.0; θ1 no se mueve |
| Círculo, θ̂(0) = [0.4, 1.3] | [0.40 1.10 0.40 1.10] | ídem |
| Círculo, θ̂(0) = [1.0, 1.0] | sin cambio | θ1 sobreestimado ×3: chattering fuerte, adaptación activa 6 % |
| Círculo, θ̂(0) = [2.0, 1.5] (lo prometido en C1) | sin cambio | comando saturado el 82 % del tiempo; la adaptación nunca arranca |

1. **En el círculo θ1 no es identificable.** La velocidad es constante y el yaw sigue la tangente, así que en el marco de yaw v_h ≈ [V, 0] y a ≈ 0. Solo θ2 del canal longitudinal se puede estimar. El canal lateral queda sesgado por el término centrípeto (ψ̇·V ≈ 1.2 m/s²), que entra en δ_y con media no nula.
2. **Los valores "arbitrarios" viejos no sirven como condición inicial.** θ̂ = [2.0, 1.5] equivale a κ̂u = 0.5, κ̂v = 0.75. Con la ley de inversión de modelo, θ̂2 > θ2 introduce realimentación positiva de velocidad y θ̂1 ≫ θ1 amplifica el retardo. El comando satura y nunca se llega al modo deslizante que activa la adaptación. En el script viejo esos valores no causaban problema porque no había inversión de modelo.
3. **En la lemniscata la adaptación funciona** (θ̂(0) = [0.2, 0.5], 160 s):

| m_L | θ̂1,x final | θ̂2,x final | RMS ν_x fijo → adaptativo | RMSE x̃ fijo / adaptativo |
|---|---|---|---|---|
| 50 g | 0.36 | 1.05 | 0.745 → 0.186 | 0.371 / 0.371 |
| 100 g | 0.37 | 1.05 | 0.747 → 0.189 | 0.293 / 0.292 |
| 200 g | 0.39 | 1.05 | 0.750 → 0.195 | 0.267 / 0.267 |
| 300 g | 0.42 | 1.05 | 0.752 → 0.198 | 0.265 / 0.264 |

θ̂1,x crece con la masa, como el 1/κu efectivo (≈ τ(1 + m_L/m_UAV): 0.33 → 0.49 en la sustituta). El RMS del estado integral ν baja unas 4 veces: el modelo identificado asume el trabajo que antes hacía el término robusto. **El error de seguimiento no cambia**, porque el STA ya rechaza esa incertidumbre. Es la conclusión honesta para el paper: la adaptación reduce el esfuerzo del término robusto, no el error de seguimiento.

Por eso E7 se rediseñó: lemniscata en lugar de círculo, 160 s y θ̂(0) = [0.2, 0.5] en lugar de [2.0, 1.5].

### 5.4 Estimador de ḣ

Discretizado exacto (a_h = 1 − e^(−Δt/τ_h)): ζ ≈ 0.44, ganancia 1.00 y retardo ≈ 37 ms en la frecuencia del péndulo. Contra la verdad de la sustituta: retardo medido 40–60 ms y ganancia 1.0–1.07. Sin lazo interno, el error RMS es 0.006 m/s; con el STA sube a 0.04 m/s por el contenido de alta frecuencia que agrega el ciclo límite.

### 5.5 Otros hallazgos

- `ReferenciasCarga_ST.m`, casos 4 y 5 (ocho y lemniscata): `ddxt = -2ac²(sin(ct) + cos(ct))` está mal; lo correcto es −2ac² sin(2ct). El error llega a **1.19 m/s²** con una aceleración real de 0.49 m/s² como máximo, lo que mueve h* hasta 0.24 m. Afecta todo el escenario con viento (es el punto 3 del Comentario 5).
- La ley sin compensación de swing (E1) llega a α = 79° en la sustituta. En SimQuad la corrida no terminaba (Fig. 6a).
- `ResponsesReviewers.tex` usa `\XX` pero no lo define; hay que agregar `\newcommand{\XX}{\textbf{XX}}` o reemplazarlos.

## 6. Qué cambió en la respuesta a C1

`respuesta_C1_revisada.tex` reemplaza la respuesta actual. Cambios respecto de la que está en `ResponsesReviewers.tex`:

| Ítem | Antes | Ahora |
|---|---|---|
| (iii) filtros | "four filters remain, 8.6–60 ms" | queda solo el de v̇_d (50 ms); se declara la saturación del comando (5 m/s) con integración condicional |
| (iv) canales | igual | agrega la Ec. nueva del marco de yaw |
| (v) adaptación | ley correcta | agrega t_on, la dinámica de s y las condiciones de identificabilidad (círculo sin PE, sesgo lateral) |
| (vi) nominales | experimento con θ̂(0) = [2.0, 1.5] | explica por qué [2.0, 1.5] no sirve; E7 en la lemniscata con θ̂(0) = [0.2, 0.5]; dice que la adaptación baja ν, no el error |
| (ii) feedforward | "error proportional to the reference jerk" | error dinámico homogéneo; el residuo se atribuye a fuerzas no modeladas (marcado para verificar) |

Quedan dos frases marcadas `% VERIFICAR`, que dependen de lo que muestre SimQuad.

## 7. Inconsistencias que quedan en el manuscrito

Hay que cerrarlas para que la respuesta sea cierta. `manuscrito_cambios_C1.tex` trae el texto listo para pegar.

1. Supuesto 3 y Ec. href con signo − → +.
2. Ec. base_reference sin ḣ* → agregarlo.
3. "obtained by filtered numerical differentiation" → analítico.
4. Modelo con n = 4 y matrices κ ∈ ℝ⁴ˣ⁴ → n = 2, θ_i ∈ ℝ² por canal, marco de yaw.
5. Ley `ajuste2` (matrices κ con fuga ‖s‖κ̂) → `ajuste_theta`.
6. **Teorema 2 y la respuesta a C3** describen la ley vieja en κ. Con la respuesta a C1 nueva, el revisor va a ver dos leyes de adaptación distintas. Hay que reescribir C3 y el teorema para la ley en θ (lo mismo vale para las respuestas a R2-C9 y R2-C10, que hablan de κ ∈ ℝ⁴ˣ⁴).
7. Sección 3: "arbitrary values κ̂u = 0.5, κ̂v = 0.75" y "All runs were deterministic".
8. Falta la subsección "Discrete-time implementation" y la tabla de parámetros.
9. Ec. (44): α = atan2(ρ, −h_z) → atan2(ρ, h_z).
10. El Teorema 1 (respuesta a C2) usa una cascada de primer orden. Con el péndulo incluido, la dinámica del error es de tercer orden, (1+K_dh)(l/g)s³ + K_h(l/g)s² + s + K_traj = 0, estable si K_h > (1+K_dh)K_traj. Las ganancias viejas (1.8/0.6 con K_traj = 1.4) no la cumplían. No es parte de C1, pero conviene revisarlo antes de enviar.

## 8. Próximos pasos (requieren SimQuad1.exe en Windows)

1. Copiar `paper_revision/` al repo (hecho) y correr en MATLAB, en este orden: `E2`, `E3`, `E4`, `E7_masa_300_asta`, con `n_rep = 1` (ver `README.md`).
2. Si s muestra un ciclo límite: probar `cfg.int.phi_bl = 0.05` y declararlo.
3. Verificar las dos frases marcadas `% VERIFICAR` en la respuesta.
4. Correr `run_paper_experiments()` completo (5 repeticiones; unas 3 h en tiempo real) y completar los `\XX` desde `tabla_metricas.tex` y `tabla_masa.tex`.
5. Aplicar `manuscrito_cambios_C1.tex` y reescribir el Teorema 2 y la respuesta a C3 para la ley en θ.
