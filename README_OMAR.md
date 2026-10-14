# Baseline externo: Omar (2022) vs controlador propuesto

Responde a R1-C7, R2-C1 y R3-C1. El SMC con capa límite (`E6_circ_smc`) queda como **ablación** del lazo interno. Este baseline es la **comparación con un método publicado**, implementado tal como está en el artículo.

> H. M. Omar, "Hardware-In-the-Loop Simulation of Time-Delayed Anti-Swing Controller for Quadrotor with Suspended Load", *Applied Sciences* 12(3):1706, 2022. doi:10.3390/app12031706

## Por qué se puede implementar fielmente en SimQuad

En el artículo, el anti-swing controller (ASC) suma una corrección a la referencia de posición, y esa referencia la sigue el controlador en cascada de PX4 (Fig. 2, Secc. 4.8). `quad1` recibe comandos de velocidad y tiene sus propios lazos de velocidad y actitud. Lo único que falta del lado de PX4 es el lazo de posición. Se implementa como el proporcional del *multicopter position controller* de PX4, con sus valores por defecto: `MPC_XY_P = 0.95`, `MPC_Z_P = 1.0`. El ASC se aplica sin cambios.

| Artículo | Código |
|---|---|
| Ecs. (1)–(2): x_s = K_sx L φ_L(t − t_sx), y_s = K_sy L θ_L(t − t_sy) | `controlador_omar.m`, historia de ángulos con interpolación lineal en t − τ |
| Ec. (3): T_L = 2π/√(g/L) | `parametros_omar.m` |
| Ec. (4): ISH = ∫(φ_L² + θ_L²) dt | `ish_omar.m` |
| Secc. 3.2: K y τ por mínimo ISH | `ajustar_omar.m` |
| Secc. 5.2: maniobra de escalones de 3 m en x y en y | maniobra de ajuste (`referencia_carga`, tipo `'pasos'`) |
| Secc. 3.1: seguimiento PX4 | v_sp = ẋ_d + K_p(p_sp − x), con p_sp = x_d + [0, 0, L] + [x_s, y_s, 0] |

**Elementos comunes a los dos controladores (hay que declararlos en el manuscrito):** la rampa de arranque σ(t) de la Ec. final_reference, la ley de yaw de la Ec. yaw_reference, la saturación u_max, la corrección de la lectura atrasada de la carga y el lazo de simulación (`sim_controlador.m`). Que `sim_controlador(@controlador_paso, cfg)` reproduce exactamente a `sim_payload_asta(cfg)` se verificó con la planta sustituta (diferencia 0).

**Información de sensores.** Omar mide los ángulos del cable con un inclinómetro y no usa tasas. El baseline usa h medido, sin estimador. El propuesto usa h y su estimador de ḣ.

## Signo y retardo

El artículo no fija el signo de sus ángulos y el valor del retardo no es legible en el texto. En el código, φ_x = atan2(−h_x, h_z) y φ_y = atan2(−h_y, h_z) son el desplazamiento de la carga respecto del enganche. El análisis del péndulo con soporte móvil indica que la corrección amortigua cuando s·sin(ωτ_total) > 0, donde τ_total es el retardo agregado más el atraso del lazo de posición y velocidad del UAV. Ese atraso es de unos 0.28 T_L con K_p = 0.95. Por eso `ajustar_omar` busca signo, K y τ juntos.

Prueba en hover con la planta sustituta (ráfaga de 2 s, K = 0.6): con s = +1 y τ = 0–0.1 T_L, el ISH baja 50 % respecto de sin ASC. Con s = +1 y τ ≥ 0.3 T_L, o con s = −1, empeora. El orden de magnitud coincide con las reducciones que reporta Omar (36.7 % en hover y 49.2 % en la maniobra de escalones).

**El ajuste no se hace en el círculo.** Ahí el ISH de la Ec. (4) lo domina la inclinación cuasiestática del cable, no la oscilación. En la planta sustituta, el mínimo en el círculo cae en s = −1, que en hover desestabiliza el péndulo. Se ajusta en la maniobra de escalones del propio artículo y esos parámetros se usan sin cambios en todos los escenarios, igual que las ganancias del propuesto.

## Cómo correr (MATLAB, Windows, `paper_revision/` como carpeta actual)

```matlab
% 1) Ajuste del baseline con su criterio (~65 min en tiempo real). Deja omar_ajuste.mat
A = ajustar_omar();

% 2) Prueba corta
comparar_omar_vs_asta('escenarios', {'E4_circ_asta'}, 'n_rep', 1)

% 3) Corrida del paper: círculo y lemniscata con viento, 5 repeticiones (~25 min)
T = comparar_omar_vs_asta();
```

Para probar el código sin simulador (Octave o MATLAB): `ajustar_omar('backend','sustituta')` y `comparar_omar_vs_asta('backend','sustituta','n_rep',1)`. Esos resultados **no** van al paper: la planta sustituta es determinista y no guarda `omar_ajuste.mat`.

**Revisar la salida de `ajustar_omar` antes del paso 3:**
- *Paso 1:* debe haber una región de τ donde el ISH baja respecto de sin ASC. Si no la hay, el baseline no amortigua en SimQuad y hay que revisar antes de comparar.
- *Paso 2:* si aparece "el óptimo está en el borde de la grilla", ampliar `'K'` o `'tau_TL'` y repetir.
- *Paso 3:* informa el óptimo en el círculo frente al seguidor sin ASC. Es solo informativo, pero conviene comentarlo en el texto.

## Salida (`results/comp_omar_<fecha>/`)

| Archivo | Contenido |
|---|---|
| `tabla_omar.tex` | Una tabla por escenario. Columnas: seguidor PX4 sin ASC, Omar y propuesto (media ± desvío). Δ % = (Omar − propuesto)/Omar, positivo si el propuesto es menor. *p* de Welch, Omar vs propuesto. Fila de corridas completas. |
| `tabla_omar.txt` | Lo mismo en texto (también se imprime en consola). |
| `resumen_omar.csv` | Una fila por corrida y controlador. |
| `raw/*.mat` | Serie completa `R` (con `R.cfg`) y métricas `M`. |
| `figuras/<esc>_omar_vs_asta.png/.pdf` | α, ‖x̃‖ y ‖v⊥‖, media de las repeticiones, con las ráfagas sombreadas. |
| `omar_parametros.txt` | K, τ, T_L, K_p, signo y origen del ajuste, para la tabla de parámetros del manuscrito. |

**Métricas.** Todas salen de la verdad del simulador (`metricas_payload.m`), con t ≥ 5 s: RMSE de x̃ y de dx̃/dt, RMS y máximo de α, ρ y ‖v⊥‖, RMS de ‖h_h − h*_h‖, el ISH de Omar (Ec. 4) y el ISH respecto de h*_h (que separa oscilación de inclinación de equilibrio), y RMS de ‖u̇_h‖ (esfuerzo y chattering del comando).

Con la planta sustituta, el RMS de ‖u̇_h‖ del propuesto es un orden de magnitud mayor que el del baseline, por el término super-twisting. Si en SimQuad pasa lo mismo, conviene mostrarlo en la tabla y discutirlo, no omitirlo.

## Archivos

| Archivo | Nuevo o modificado |
|---|---|
| `controlador_omar.m` | nuevo |
| `parametros_omar.m` | nuevo |
| `sim_controlador.m` | nuevo |
| `ish_omar.m` | nuevo |
| `ajustar_omar.m` | nuevo |
| `comparar_omar_vs_asta.m` | nuevo |
| `referencia_carga.m` | se agrega el caso `'pasos'`, solo para el ajuste; el círculo y la lemniscata no cambian |

`controlador_paso.m`, `cfg_paper.m` y `metricas_payload.m` no se modificaron.
