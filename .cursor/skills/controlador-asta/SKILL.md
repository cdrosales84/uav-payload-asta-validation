---
name: controlador-asta
description: >-
  Implements and tunes the cascaded payload controller (kinematic outer loop
  plus adaptive super-twisting inner loop) of Paper/SN_Payload.tex in the
  MATLAB code under paper_revision. Use when editing controlador_paso.m,
  cfg_paper.m, sim_payload_asta.m, referencia_carga.m, or metricas_payload.m,
  or when the user asks to implementar, corregir, ajustar, sintonizar, or
  validar el controlador, Kh, Kdh, K1, K2, theta, h-star, A-STA, or SimQuad.
---

# Rol: implementación y ajuste del controlador A-STA

Actuar como el ingeniero que mantiene la correspondencia entre el controlador de la Sección 2 y `paper_revision/`. El objetivo es que cada término del código sea una ecuación declarada, y que cada cambio de ganancia se justifique con una señal medida en SimQuad.

La especificación ejecutable es `controlador_paso.m` junto con `paper_revision/EQUATION_MAP.md`. `Paper/SN_Payload.tex` todavía conserva la ley anterior en varios puntos. Esas diferencias están en [reference.md](reference.md). No reescribir el código para que coincida con la ecuación vieja del `.tex`.

## Alcance

Toda la ley vive en `controlador_paso.m`. `sim_payload_asta.m` solo lee la planta, evalúa la referencia y llama al controlador. Los parámetros viven solo en `cfg_paper.m`.

Fuera de esa ley no se agregan: `tanh` sobre el error de posición, término de velocidad de la carga, saturación de `v_comp`, integral con fuga, adaptación de `K1`/`K2`, ni compensador en `z` o yaw.

## Antes de tocar una ecuación

1. Leer el bloque en `controlador_paso.m`, la fila de `EQUATION_MAP.md` y la ecuación del `.tex`.
2. Si el `.tex` y el código discrepan, parar y decir cuál de los dos se va a seguir. La columna "Código" de [reference.md](reference.md) es la que se implementa.
3. Cambiar un solo parámetro o un solo término por vez. Anotar el valor anterior y el nuevo en `cfg_paper.m` con un comentario de una línea (escenario y señal que motivó el cambio).
4. No copiar al manuscrito números obtenidos con `planta_sustituta`.

## Orden de validación en SimQuad

Correr con `paper_revision/` como carpeta actual, una repetición, en este orden. No pasar al siguiente si el anterior no cumple.

```matlab
run_paper_experiments('escenarios', {'E2_circ_cinematico'}, 'n_rep', 1)
run_paper_experiments('escenarios', {'E3_circ_sta_fijo'}, 'n_rep', 1)
run_paper_experiments('escenarios', {'E4_circ_asta'}, 'n_rep', 1)
run_paper_experiments('escenarios', {'E7_masa_300_asta'}, 'n_rep', 1)
```

| Escenario | Figura | Criterio para seguir |
|---|---|---|
| E2 | `*_6_estimador.png` | `hp` sigue a la `hdot` verdadera; la corrida llega a `T` sin abortar (`alpha` < 80°, error < 10 m) |
| E3 | `*_4_lazo_interno.png` | `s` sin ciclo límite de período < 1 s. Si lo hay, probar `cfg.int.phi_bl = 0.05` y dejar constancia de que hay que declararlo en el paper |
| E4 | `*_5_theta.png` | `theta_hat` no queda pegado a `Theta_min` ni a `Theta_max` |
| E7 | `*_5_theta.png` y `nu` | en la lemniscata, `theta1` del canal longitudinal se mueve; el RMS de `nu` baja respecto del caso `fijo` |

Recién después: `run_paper_experiments()` con 5 repeticiones.

## Cómo ajustar

Condición del lazo externo, con el péndulo (tercer orden). Tiene que cumplirse en los canales horizontales:

`K_h > (1 + K_dh) * K_traj`

Con los valores de `cfg_paper.m`, `K_traj_xy = 1.4`, `K_dh = 0.3` exigen `K_h > 1.82`. El nominal `K_h = 3` la cumple. El par viejo `K_h = 1.8`, `K_dh = 0.6` no. No volver a ese par para “calmar” el lazo interno: obligó a subir `K1`/`K2` y el ciclo límite empeora.

Síntoma y el único parámetro que se toca:

| Síntoma | Parámetro | Dirección |
|---|---|---|
| Balanceo creciente o abort por `alpha` en E2 | `Kh`, `Kdh` | subir `Kh` manteniendo la desigualdad; `Kdh` solo para el transitorio |
| Error de posición grande y lento, cable quieto | `Ktraj` | subir, y revalidar la desigualdad |
| `s` en ciclo límite, `RMS(udot)` alto | `K1`, `K2` | bajar juntos desde `0.5` / `0.75`; no subirlos para enmascarar el lazo externo |
| Ciclo límite que no cede al bajar `K1`/`K2` | `phi_bl` | `0.05`, y declararlo |
| `theta_hat` no se mueve y `|s|` grande | no es la ganancia `Gamma` | primero lograr modo deslizante (`|s| <= s_on`); la adaptación está apagada fuera de esa banda y antes de `t_on` |
| `theta_hat` contra el borde de `Theta` | `theta_ini` o excitación | no ampliar `Theta` para ocultarlo. En el círculo `theta1` no es identificable; usar E7 (lemniscata) |
| Comando en `u_max` la mayor parte del tiempo | `theta_ini` | bajarlo hacia `theta0 = [tau; 1]`. `theta_ini = [2; 1.5]` satura y la adaptación no arranca |

`theta_i = [1/kappa_u,i ; kappa_v,i/kappa_u,i]`. Con la inversión de modelo, `theta2_hat > theta2` realimenta velocidad en positivo y `theta1_hat >> theta1` amplifica el retardo.

La adaptación reduce el esfuerzo del término robusto (`nu`). No se espera que baje el error de seguimiento: el STA ya rechaza esa incertidumbre.

## Al informar un cambio

Decir: ecuación (etiqueta), archivo y líneas, valor anterior y nuevo, escenario corrido, y la señal que cambió (`s`, `alpha`, `ex`, `theta`, `nu`, saturación). Si no se pudo correr SimQuad, decirlo y no presentar el cambio como validado.
