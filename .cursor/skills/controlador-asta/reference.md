# Referencia del controlador (no cargar salvo para una ecuación concreta)

## Qué implementa el código

`controlador_paso.m` realiza la ley revisada, no la que todavía está impresa en `Paper/SN_Payload.tex`. Detalle línea a línea: `paper_revision/EQUATION_MAP.md`.

Convención fija: `h = x - x_l`, `z` inercial hacia arriba. El lazo interno actúa solo en `x,y` del marco de yaw. El simulador recibe `[v_xh, v_yh, v_zh, w_zh]` en ese marco.

Parametrización por canal, `i ∈ {x,y}`:

- `theta_i = [1/kappa_u,i ; kappa_v,i/kappa_u,i]`
- `v_tilde = v - v_d`, `s = v_tilde + beta * ∫ v_tilde`
- `a = vdot_d - beta * v_tilde`, `phi_i = [a_i, v_i]`
- `u_i = phi_i * theta_hat_i - w_i`
- `w_i = K1_i * sqrt(|s_i|) * sign(s_i) + nu_i`, `nu_dot_i = K2_i * sign(s_i)`
- En deslizamiento: `nu_i = phi_i * theta_tilde_i + delta_i/kappa_u,i`
- Adaptación, solo si `t > t_on` y `|s_i| <= s_on`:

`theta_hat_dot_i = Proj_Theta( -Gamma * phi_i' * sat(nu_i) - sigma * (theta_hat_i - theta_prior) )`

`z` y yaw copian el comando cinemático. No llevan compensador.

## Discrepancias que no hay que “corregir” hacia el .tex

| Punto | `SN_Payload.tex` (ley vieja) | Código a conservar |
|---|---|---|
| Signo de `h*` | `eq:href` y `as:smallangle`: `-(l/g) xdd_d` | `+(l/g) * r.a(1:2)`. Con `h = x - x_l` el UAV va adelantado a la carga |
| Feedforward | `eq:base_reference` solo tiene `xd_dot + K_traj x_tilde` | suma `[hdot*_h; 0]`, con `hdot* = (l/g) * jerk` analítico |
| `hdot*` | “filtered numerical differentiation” | derivadas analíticas de `referencia_carga.m` |
| Dimensión | `n = 4`, `kappa` de 4×4 | `n = 2`, canales horizontales en el marco de yaw |
| Forma del STA | `K1 * sqrt(\|\|s\|\|) * sign(s)` (norma del vector, `w` dentro de `kappa_u^{-1}`) | STA escalar por canal; `w` queda fuera de `phi*theta` |
| Qué se adapta | `eq:ajuste2`: `kappa_u`, `kappa_v`, gradiente con `s`, fuga `\|\|s\|\| kappa_hat` | `theta`, gradiente con `sat(nu)`, fuga `sigma` hacia `theta_prior`, compuerta `|s| <= s_on` |
| Filtros | no declarados | solo `tau_vdot = 0.05` s. `tau_ex`, `tau_epsi`, `tau_vc` en 0 |
| Arranque | `sigma(t)` sí está | `sigma = 1 - exp(-t/Ts)`, `Ts = 3` |

Si el usuario pide alinear el código con una de esas ecuaciones del `.tex`, mostrar esta tabla y pedir confirmación antes de editar.

## Nominales (`cfg_paper.m`)

| Símbolo | Campo | Valor |
|---|---|---|
| `K_traj` | `cfg.ext.Ktraj` | diag(1.4, 1.4, 0.7) |
| `K_h`, `K_dh` | `cfg.ext.Kh`, `Kdh` | 3.0, 0.3 |
| `T_s`, `k_psi`, `k_psi^t` | `Ts`, `kpsi`, `kpsit` | 3, 1, 0.5 |
| `eps1`, `K1` estimador, `tau_h` | `cfg.est` | 0.14, `3 I`, 0.06 s |
| `beta` | `cfg.int.beta` | 0.08 |
| `K1`, `K2` del STA | `cfg.int.K1`, `K2` | 0.5, 0.75 |
| `phi_bl` | `cfg.int.phi_bl` | 0 (signo puro) |
| `u_max` | `cfg.int.u_max` | 5 m/s, con integración condicional de `nu` |
| `theta0` | `cfg.ad.theta0` | `[0.4; 1]` (`tau_vel = 0.4`, ganancia estática 1) |
| `Gamma`, `sigma`, `nu_bar` | `cfg.ad` | diag(0.1, 0.1), 0.005, 5 |
| `s_on`, `t_on` | `cfg.ad` | 0.05, 5 s |
| `Theta` | `Theta_min`, `Theta_max` | `[0.05; 0.2]` a `[3; 3]` |

`theta1 > 0` mantiene `kappa_u_hat > 0`. No relajar `Theta_min(1)`.

## Identificabilidad

En el círculo la velocidad es constante y el yaw sigue la tangente: en el marco de yaw `v_h ≈ [V, 0]` y `a ≈ 0`. Solo `theta2` longitudinal es identificable. El canal lateral queda sesgado por el término centrípeto. Por eso E7 es una lemniscata de 160 s con `theta_ini = [0.2; 0.5]`, no el círculo y no `[2; 1.5]`.

## Métricas

Usar `metricas_payload.m` (verdad del simulador: diferencia central de la posición de la carga). No reportar velocidades del estimador como si fueran la medición. Validación del estimador: RMS de `hp_h - hdot_h`, retardo y ganancia por correlación.
