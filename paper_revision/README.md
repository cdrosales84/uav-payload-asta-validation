# paper_revision — código del manuscrito revisado

Implementa exactamente el controlador de la Sección 2 del manuscrito revisado (respuesta al Comentario 1 del Revisor 1). Las ecuaciones están en un solo archivo, `controlador_paso.m`, y cada línea lleva la etiqueta de la ecuación. La correspondencia completa está en `EQUATION_MAP.md`.

## Archivos

| Archivo | Qué hace |
|---|---|
| `cfg_paper.m` | Todos los parámetros y los escenarios E1–E7. Se guarda con cada resultado. |
| `controlador_paso.m` | Un período del controlador: estimador de ḣ, lazo cinemático, compensador A-STA, adaptación. |
| `sim_payload_asta.m` | Una corrida: lee la planta, evalúa la referencia en el tiempo del simulador, llama al controlador. |
| `referencia_carga.m` | Círculo y lemniscata con velocidad, aceleración y jerk analíticos. |
| `metricas_payload.m` | Métricas con la verdad del simulador y validación del estimador de ḣ. |
| `run_paper_experiments.m` | Corre todos los escenarios (5 repeticiones), guarda `.mat`, CSV, tablas LaTeX y figuras. |
| `tabla_parametros_latex.m` | Tabla de parámetros del manuscrito, generada desde `cfg_paper.m`. |
| `figuras_escenario.m` | Figuras rápidas de control de cada escenario (PNG). |
| `figuras_paper.m` | Figuras del manuscrito, con los nombres de `\includegraphics`, en EPS y PDF. |
| `planta_simquad.m` | Interfaz con `SimQuad1.exe` y los mex de `../simulations`. |
| `planta_sustituta.m` | Modelo simplificado con la misma interfaz, para probar el código sin Windows. **No se usa para resultados del paper.** |
| `test_referencias.m` | Verifica las derivadas analíticas de las referencias. |

## Cómo correr

En MATLAB (Windows), con `paper_revision/` como carpeta actual:

```matlab
run_paper_experiments()                                    % todo, 5 repeticiones, SimQuad1.exe
run_paper_experiments('escenarios', {'E4_circ_asta'}, 'n_rep', 1)   % una corrida de prueba
```

Los resultados quedan en `results/<fecha>/`: `raw/*.mat` (serie completa + `cfg`), `resumen.csv`, `tabla_metricas.tex`, `tabla_masa.tex`, `tabla_parametros.tex` y `figuras/`.

Sin Windows ni MATLAB (por ejemplo, Octave en Linux), para probar el código:

```matlab
run_paper_experiments('backend', 'sustituta', 'n_rep', 1)
```

## Figuras para el paper

`run_paper_experiments` llama al final a `figuras_paper`. Para regenerarlas sin volver a simular, por ejemplo con otro tamaño o formato:

```matlab
figuras_paper('results/20261010_120000')                               % EPS + PDF, 160 mm de ancho
figuras_paper('results/20261010_120000', 'ancho_mm', 150, 'formato', {'pdf'})
```

Quedan en `results/<fecha>/paper_figs/` con el mismo nombre que usa `SN_Payload.tex` (`Compensador_Angles.eps`, `Viento_ErroresControl.eps`, …), así que alcanza con copiarlas a `Paper/Figures/`. Hay tres figuras nuevas: `Estimador_Validacion` (Comentario 4), `Comp_Baseline_Chattering_zoom` y `Masa_Adaptacion` (barrido de masa).

- **Tamaño de letra real.** `ancho_mm` debe coincidir con el `\textwidth` del manuscrito: compilar una vez con `\showthe\textwidth` (1 pt = 0.3515 mm), pasar ese valor e incluir con `\includegraphics[width=\linewidth]{...}`. Así LaTeX no reescala la figura y la letra queda en 8 pt.
- **Repeticiones.** Cada curva es la media de las 5 repeticiones, interpoladas a una grilla común; la banda clara es el mínimo y el máximo entre ellas (`'banda', false` la quita).
- **Comparaciones.** `Compensador_Angles`, `Compensador_ErroresControl` y `Compensador_ErroresVelocidad` superponen el controlador cinemático solo (naranja) y la cascada completa (azul); el benchmark superpone el SMC (verde) y la propuesta. Los colores son fijos por controlador en todas las figuras.
- **Viento.** Los intervalos de ráfaga se sombrean en gris.
- Las etiquetas usan el intérprete LaTeX de MATLAB. En Octave se simplifican y solo sirven como vista previa.

## Escenarios

| Escenario | Qué es | Uso en el paper |
|---|---|---|
| `E1_circ_sin_swing` | Círculo, v_c = ẋ_d + K_traj x̃, sin compensación ni lazo interno | Fig. 6(a) |
| `E2_circ_cinematico` | Círculo, controlador cinemático completo, sin lazo interno | Tabla de métricas, fila "Kinematic" |
| `E3_circ_sta_fijo` | Cascada con θ̂ = θ0 fija | Separa el efecto de la adaptación |
| `E4_circ_asta` | Controlador propuesto | Tabla de métricas, figuras de la Secc. 3.2 |
| `E5_lemn_viento` | Lemniscata, 0.3 kg, ráfagas [0 6 0] m/s (20–40 s) y [3√2 3√2 0] m/s (60–80 s) | Secc. 3.4 |
| `E6_circ_smc` | Ablación: SMC con capa límite por componente en lugar del STA | Secc. 3.5 |
| `E7_masa_<g>_<fijo\|asta>` | Lemniscata sin viento, 160 s, m_L = 50/100/200/300 g, θ̂(0) = [0.2, 0.5] | Barrido de masa (nuevo) |

## Antes de generar los resultados del paper: verificar en SimQuad1.exe

El código se probó completo en Octave contra `planta_sustituta`. Falta correrlo con el simulador real. En este orden:

1. `run_paper_experiments('escenarios', {'E2_circ_cinematico'}, 'n_rep', 1)`. Debe completar los 100 s. Verificar en `figuras/*_6_estimador.png` que ĥ_p sigue a ḣ verdadero.
2. Lo mismo con `E3_circ_sta_fijo`. Mirar `*_4_lazo_interno.png`: si s oscila con período < 1 s (ciclo límite del STA por la dinámica de actitud), probar `cfg.int.phi_bl = 0.05` y declararlo en el paper.
3. Lo mismo con `E4_circ_asta` y `E7_masa_300_asta`. Verificar en `*_5_theta.png` que θ̂ no se pega a los bordes de Θ.
4. Recién entonces correr todo con 5 repeticiones.

Las ganancias por defecto son K_h/K_dh = 3.0/0.3 (cumplen K_h > (1 + K_dh) K_traj) y K1/K2 = 0.5/0.75. En `controller.m` eran 1.8/0.6 y 1.0/1.5. Si en SimQuad algo no funciona, cambiar un parámetro por vez en `cfg_paper.m` y anotar el valor exacto.

## Requisitos

MATLAB R2016b o posterior (expansión implícita), Windows x64 para `SimQuad1.exe` y los `.mexw64` de `../simulations`. Probado también en GNU Octave 8.4 con el backend `sustituta`.
