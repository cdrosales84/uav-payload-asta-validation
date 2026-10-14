# Validación del controlador A-STA para UAV con carga suspendida

Simulación del controlador en cascada del manuscrito *Robust Adaptive Control of Underactuated UAV-Suspended Payload Systems* (`Paper/SN_Payload.tex`): lazo cinemático externo que amortigua el péndulo y compensador interno por super-twisting adaptativo (A-STA). Las corridas usan el simulador de alta fidelidad SimQuad (Windows) y dejan series, métricas y figuras en `results/`.

Requisito para regenerar corridas: MATLAB en Windows, con esta carpeta como directorio actual. El ejecutable `SimQuad1.exe` y los MEX (`quad1`, `rope1`, `simulation1`, `wind1`) ya están en la raíz.

## Estructura

```
.
├── cfg_paper.m                  parámetros y escenarios (E1…E7)
├── controlador_paso.m           un periodo del controlador en cascada
├── sim_payload_asta.m           una corrida; devuelve la serie R
├── metricas_payload.m           métricas M a partir de R
├── run_paper_experiments.m      barrido de escenarios, tablas y figuras
├── graficar_resultados_paper.m  figuras de diagnóstico desde los .mat
├── figuras_paper.m              figuras del manuscrito (media de repeticiones)
├── figuras_escenario.m          seis figuras de control de una corrida
├── plot_uav_payload_frame.m    un instante 3D (UAV, cuerda, carga)
├── planta_simquad.m             interfaz con SimQuad1.exe
├── planta_sustituta.m           planta MATLAB, solo para pruebas
├── referencia_carga.m           círculo y lemniscata
├── SimQuad1.exe, *.mexw64       simulador (Windows)
├── Paper/                       manuscrito (SN_Payload.tex, References.bib)
└── results/<escenario>/         salidas guardadas (ver abajo)
```

Cada `.mat` guarda la serie temporal `R`, las métricas `M` y la configuración usada en `R.cfg`.

Señales útiles de `R` (cada columna es una muestra, el tiempo está en `R.t`):

| Campo | Contenido |
| --- | --- |
| `x`, `v`, `psi` | posición, velocidad y yaw del UAV |
| `xl` | posición de la carga (lectura corregida) |
| `xd`, `vd`, `psid` | referencia de posición, velocidad y yaw |
| `h`, `hp` | vector cuerda `x - xl` y su velocidad estimada |
| `ex`, `epsi` | error de posición de la carga y de yaw |
| `u`, `s`, `nu`, `theta` | comando, superficie, estado integral y parámetros adaptativos |
| `vt` | error de velocidad del lazo interno (marco de yaw, canales x,y) |
| `viento` | viento aplicado `[wx; wy; wz]` |
| `cfg` | copia de `cfg_paper` de esa corrida |

`M` resume la corrida (RMSE, oscilación, saturación, `theta` final). `M.serie` trae la velocidad verdadera de la carga (`vl`) y las señales de oscilación (`alpha`, `rho`, `vperp`, `swing`, `hdot`), que no se guardan dentro de `R`.

## Escenarios

Definidos en `cfg_paper.m`. Los cuatro primeros tienen carpeta en `results/`.

| Carpeta | Qué se simula |
| --- | --- |
| `E1_circ_sin_swing` | círculo, ley cinemática sin compensación de swing (la corrida se corta por divergencia) |
| `E2_circ_cinematico` | círculo, solo lazo cinemático |
| `E4_circ_asta` | círculo, controlador propuesto (cinemático + A-STA) |
| `E5_lemn_viento` | lemniscata, carga de 0,30 kg, dos bloques de viento (20–40 s lateral, 60–80 s oblicuo) |

También están definidos, sin carpeta de resultados en el repositorio: `E3_circ_sta_fijo` (STA con `theta` fija), `E6_circ_smc` (ablación SMC) y `E7_masa_<gramos>_{fijo,asta}` (barrido de masa en lemniscata).

## Carpeta `results`

```
results/<escenario>/
├── raw/<escenario>_rep<k>.mat    serie R y métricas M
├── resumen.csv                    una fila de métricas por repetición
├── tabla_metricas.tex             media y desvío (LaTeX)
├── tabla_parametros.tex           parámetros de cfg_paper
├── figuras/                       PNG y FIG de control (figuras_escenario)
└── paper_figs/                    figuras del manuscrito (PDF; EPS y FIG en local)
```

Los PDF de `paper_figs/`, los CSV y los `.tex` sí están en el repositorio. Los `.mat`, `.png`, `.fig` y `.eps` están en `.gitignore`: quedan en la copia local y no llegan con un clone. Para volver a dibujar las curvas hace falta el `.mat` de `raw/` (o regenerar la corrida).

`run_paper_experiments` escribe por defecto en `results/<yyyymmdd_HHMMSS>/`, con todos los escenarios juntos bajo un único `raw/`. Las carpetas `results/E*` son el recorte por escenario que usa este repositorio.

## Graficar lo guardado

1. Abrir MATLAB en la raíz del repositorio.
2. En `graficar_resultados_paper.m`, apuntar `CarpetaResultados` al `raw/` del escenario. El script recorre todos los `*.mat` de esa carpeta.

```matlab
CarpetaResultados = fullfile(pwd, 'results', 'E4_circ_asta', 'raw');
```

3. Ejecutar el script. Por cada corrida abre estas figuras y, con `ExportarFiguras = true`, las guarda en `<raw>/figuras/` como PNG (y FIG). `ExportarPDF = true` agrega el PDF vectorial.

| Nombre de archivo | Qué muestra |
| --- | --- |
| `error_posicion` | errores de X, Y, Z de la carga y de yaw |
| `error_velocidad` | errores de velocidad (y de yaw) |
| `oscilacion` | ángulo α, desplazamiento horizontal ρ y velocidad perpendicular a la cuerda |
| `seguimiento_xy` | referencia y carga en X e Y |
| `posiciones` | X, Y, Z y yaw de la carga contra la referencia |
| `parametros_theta` | parámetros adaptativos |
| `superficie` | superficie deslizante |
| `ref_dron_carga_pos` | referencia, UAV y carga (posición) |
| `ref_dron_carga_vel` | referencia, UAV y carga (velocidad) |
| `control` | comandos `u` |

En `E5_lemn_viento` el script marca los intervalos de viento con bandas de color. En la ventana de comandos imprime las métricas guardadas en `M` junto al valor recalculado de la serie.

Para un solo archivo:

```matlab
S = load('results/E5_lemn_viento/raw/E5_lemn_viento_rep1.mat');
% S.R  serie,  S.M  métricas,  S.R.cfg  parámetros de esa corrida
```

Un instante en 3D (el índice `k` es la muestra más cercana al tiempo pedido):

```matlab
S = load('results/E4_circ_asta/raw/E4_circ_asta_rep1.mat');
[~, k] = min(abs(S.R.t - 20));
plot_uav_payload_frame(S.R, k);
```

Las figuras ya exportadas se abren sin recalcular: los PDF de `results/<escenario>/paper_figs/` y, si están en la copia local, los PNG y FIG de `results/<escenario>/figuras/`.

Para rehacer las figuras del manuscrito (media entre repeticiones, banda mín–máx, nombres iguales a los `\includegraphics` de `Paper/SN_Payload.tex`):

```matlab
figuras_paper('results/E4_circ_asta');
% escribe results/E4_circ_asta/paper_figs/
```

`figuras_paper` busca `raw/<escenario>_rep*.mat` dentro de la carpeta que se le pasa. Con el armado actual, una llamada por escenario alcanza para las figuras de ese caso. Las figuras que comparan dos leyes (por ejemplo cinemático contra A-STA) necesitan los `.mat` de ambos escenarios bajo el mismo `raw/`.

## Volver a correr un experimento

Desde la raíz del repositorio, con SimQuad:

```matlab
run_paper_experiments('escenarios', {'E4_circ_asta'}, 'n_rep', 1);
```

Sin argumentos recorre todos los escenarios de `cfg_paper` (5 repeticiones) y crea `results/<fecha>/`. La opción `'backend','sustituta'` corre el modelo de MATLAB y no usa `SimQuad1.exe`; esas corridas no son las del paper.
