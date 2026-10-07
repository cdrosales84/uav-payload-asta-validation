# Correspondencia ecuación ↔ código

Manuscrito revisado (Sección 2) ↔ `paper_revision/`. Las etiquetas son las `\label{}` de `SN_Payload.tex`; las nuevas (`eq:ajuste_theta`, `eq:yaw_frame`, `eq:ref_circ`, `eq:ref_lemn`) hay que crearlas en el manuscrito.

Todas las ecuaciones del controlador están en `controlador_paso.m`. El bucle de simulación (`sim_payload_asta.m`) solo lee la planta, evalúa la referencia y envía el comando.

## Lazo externo (controlador cinemático)

| Ecuación | Expresión | Archivo:línea |
|---|---|---|
| `eq:cable_vector` | h = x − x_l | `controlador_paso.m:23` |
| `eq:h_raw` | h_p,raw = ε₁⁻¹ K₁ (h − ĥ) | `controlador_paso.m:40` |
| `eq:h_p_filter` | τ_h ḣ̂_p + ĥ_p = h_p,raw → ĥ_p(k) = ĥ_p(k−1) + a_h (h_p,raw − ĥ_p(k−1)), a_h = 1 − e^(−Δt/τ_h) | `controlador_paso.m:41–42` |
| `eq:h_hat_dyn` | ĥ(k+1) = ĥ(k) + Δt ĥ_p(k), ĥ(0) = h(0) | `controlador_paso.m:44`, init `:27` |
| `eq:payload_tracking_error` | x̃ = x_d − x_l | `controlador_paso.m:47` |
| `eq:href` | h*_h = **+**(l/g) ẍ_d,h, ḣ*_h = (l/g) x⃛_d,h (analíticas) | `controlador_paso.m:53–54` |
| `eq:base_reference` | v_base = ẋ_d + [ḣ*_h; 0] + K_traj x̃ | `controlador_paso.m:60` |
| `eq:swing_compensation` | v_comp = [−K_h(h_h − h*_h) − K_dh(ĥ_p,h − ḣ*_h); 0] | `controlador_paso.m:61` |
| `eq:final_reference` | v_c = σ(t)(v_base + v_comp), σ = 1 − e^(−t/T_s) | `controlador_paso.m:68–69` |
| `eq:yaw_reference` | ψ̇_c = ψ̇_d + k_ψ tanh(k_ψᵗ e_ψ) | `controlador_paso.m:70` |
| `eq:yaw_frame` (nueva) | v_d = R_ψᵀ v_c,h; v = R_ψᵀ ẋ_h (marco de yaw del modelo de Santana et al.) | `controlador_paso.m:74–78` |

## Lazo interno (compensador dinámico, n = 2: canales x, y en el marco de yaw)

| Ecuación | Expresión | Archivo:línea |
|---|---|---|
| superficie | s = ṽ + β ∫ṽ dt, ṽ = v − v_d | `controlador_paso.m:92–93` |
| regresor | a = v̇_d − β ṽ, φ_i = [a_i, v_i] | `controlador_paso.m:85`, `:95` |
| `eq:AccionControl` | u_i = φ_i θ̂_i − w_i, w_i = K₁ √\|s_i\| sign(s_i) + ν_i, ν̇_i = K₂ sign(s_i) | `controlador_paso.m:96–110` |
| `eq:ajuste_theta` (nueva) | θ̂̇_i = Proj_Θ(−Γ φ_iᵀ sat_ν̄(ν_i) − σ(θ̂_i − θ_prior)), activa si t > t_on y \|s_i\| ≤ s_on | `controlador_paso.m:114–124` |
| z y yaw | u_z = v_c,z, u_ψ = ψ̇_c (sin compensador) | `controlador_paso.m:133` |

Parametrización: θ_i = [1/κ_u,i ; κ_v,i/κ_u,i]. Con u = φθ̂ − w, la dinámica de la superficie es ṡ_i = κ_u,i φ_i θ̃_i − κ_u,i w_i + δ_i, con θ̃ = θ̂ − θ. En modo deslizante, ν_i = φ_i θ̃_i + δ_i/κ_u,i.

## Implementación discreta (sección nueva)

| Elemento | Valor por defecto | Archivo:línea |
|---|---|---|
| Período | Δt = diferencia del tiempo del simulador (nominal 20 ms) | `sim_payload_asta.m:36–44` |
| Filtro de v̇_d | τ = 50 ms | `cfg_paper.m` (`filt.tau_vdot`), `controlador_paso.m:84` |
| Filtros de x̃, e_ψ y u_k | desactivados (τ = 0) | `cfg_paper.m` (`filt.*`) |
| Saturación del comando | \|u_i\| ≤ u_max = 5 m/s; ν no integra hacia fuera de la saturación | `controlador_paso.m:105–110`, `:130` |
| Proyección | recorte de θ̂ a Θ después de cada paso de Euler | `controlador_paso.m:122` |

## Métricas (verdad del simulador)

| Métrica | Definición | Archivo |
|---|---|---|
| ẋ_l | diferencia central de la posición de la carga (última partícula de la cuerda) con el tiempo del simulador | `metricas_payload.m` (`dercentral`) |
| ḣ | ẋ − ẋ_l | ídem |
| α | atan2(√(h_x²+h_y²), h_z) | ídem |
| v_⊥ | v_rel − (v_rel·ĥ)ĥ, v_rel = ẋ_l − ẋ | ídem |
| swing | ‖h_h − h*_h‖ | ídem |
| validación del estimador | RMS(ĥ_p,h − ḣ_h), retardo y ganancia por correlación | ídem (`retardo_ganancia`) |

## Referencias

| Ecuación | Archivo |
|---|---|
| `eq:ref_circ`: x_d = 3 sin(2πt/10), y_d = 3 cos(2πt/10), z_d = 6 + t/10, ψ_d = atan2(ẏ_d, ẋ_d) | `referencia_carga.m` |
| `eq:ref_lemn`: x_d = (a/2) sin(2ct), y_d = b sin(ct), z_d = 6 + d sin(ct) | `referencia_carga.m` |

`test_referencias.m` compara cada derivada analítica con la derivada numérica (error < 1e-4).
