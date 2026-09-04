# Mapeo código ↔ ecuaciones — Crítico #1 (Rev. 1)

**Archivo auditado:** `simulations/CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m`
**Objetivo:** listar, discrepancia por discrepancia, dónde el código diverge de las ecuaciones del manuscrito (`Paper/SN_Payload.tex`, secciones 2.2 "Kinematic Control", 2.3 "Closed-Loop Error Analysis" y 2.4 "Dynamic Compensation"), para poder decidir la reescritura línea por línea. Se asume la decisión ya tomada: **el manuscrito es la versión correcta**; el código debe ajustarse a él.

Convención de líneas: numeración del archivo tal como está en el repo al 2026-09-01 (commit actual de `master`).

---

## A. Lazo cinemático externo (outer loop)

### A.1 Comando base de seguimiento (`v_base`)

- **Manuscrito**, Ec. `eq:base_reference`:
  \[
  \mathbf v_{\text{base}} = \dot{\mathbf x}_d + K_{\text{traj}}\,\tilde{\mathbf x}
  \]
  lineal en el error de payload, sin término de velocidad.

- **Código**, líneas 445-448:
  ```matlab
  v_base = yXidp(:,k) ...
      + Ktraj *tanh(Ktrajt *yExil(:,k))...
      + Kvload*tanh(Kvloadt*yExilp(:,k));
  ```
  Dos discrepancias:
  1. La ganancia proporcional no es lineal: se aplica `tanh(Ktrajt * error)` en vez de `K_traj * error` puro (saturación no declarada en el manuscrito).
  2. Hay un **término adicional en la velocidad del error de payload** (`Kvload*tanh(Kvloadt*yExilp)`) que no existe en `eq:base_reference`. Esto es exactamente lo que el Revisor 1 señala como "additional payload-velocity feedback term absent from Eq. (8)".

  **Acción:** eliminar el término `Kvload*tanh(...)` y reemplazar `tanh(Ktrajt*e)` por `Ktraj*e` lineal, o bien — si se decide mantener la saturación por razones prácticas de vuelo — documentar explícitamente esta modificación en el manuscrito y en el Teorema 1 (ver sección C). Dado que se optó por "el manuscrito es correcto", la opción por defecto es **eliminar ambas desviaciones** y dejar `v_base = yXidp + Ktraj*yExil`.

### A.2 Signo de la referencia cuasiestática (`h_href`)

- **Manuscrito**, Ec. `eq:href`:
  \[
  \mathbf h_h^\star = -\frac{l}{g}\,\ddot{\mathbf x}_{d,h}
  \]
  (signo **negativo**, bajo la convención \(\mathbf h=\mathbf x-\mathbf x_l\)).

- **Código**, líneas 458-459:
  ```matlab
  a_ref_h = yXidpp(1:2,k);
  h_ref_h = (lcuerda/9.81) * a_ref_h;
  ```
  **Sin el signo negativo.** Esto es precisamente la discrepancia de signo que señala el Revisor 1 en el punto 1 ("the sign of the quasi-static cable reference differs from Eq. (12)").

  **Acción:** cambiar a `h_ref_h = -(lcuerda/9.81) * a_ref_h;`. **Importante:** antes de aplicar el cambio hay que verificar la convención de signo de `yH` en el código (`yH(:,k) = yXi(:,k) - yXil(:,k)`, línea ~377, que sí coincide con `h = x - x_l` del manuscrito), así que el cambio de signo aquí es consistente y no requiere tocar la definición de `yH`. Corregir este signo **cambia la dirección del término restaurador** `comp_xy = -Kh*eh_h - Kdh*ehp_h`, por lo tanto **hay que re-simular todo lo que dependa de este script** una vez corregido.

### A.3 Saturación no declarada en `v_comp`

- **Manuscrito**, Ec. `eq:swing_compensation`: sin saturación, solo el par PD sobre `h_h - h_h^\star`.
- **Código**, líneas 474-476:
  ```matlab
  comp_max = 0.9;
  comp_xy = comp_max * tanh(comp_xy/comp_max);
  ```
  Saturación suave adicional no mencionada en el manuscrito.

  **Acción:** quitar la saturación (para que el código sea fiel a la ecuación), o agregarla al manuscrito como una modificación práctica explícita y re-derivar el Teorema 1 con el término saturado (más trabajo). Recomendado: quitarla, dado que el manuscrito no la incluye y el Teorema 1 no la contempla.

### A.4 Rampa de arranque `sigma(t)`

- **Manuscrito**, Ec. `eq:final_reference`: \(\sigma(t)=1-e^{-t/T_s}\). 
- **Código**, línea 484: `sigma = 1 - exp(-yTsim(k)/Tsoft);` con `Tsoft = 3.0`.
  **Coincide.** Sin acción.

### A.5 Estimador de velocidad del cable (Ecs. `h_raw`, `h_hat_dyn`, `h_p_filter`)

- **Manuscrito:** estimador de **dos etapas** — ganancia alta \(\mathbf h_{p,\text{raw}}=\frac1{\varepsilon_1}K_1(\mathbf h-\hat{\mathbf h})\), estado interno \(\dot{\hat{\mathbf h}}=\hat{\mathbf h}_p\), y filtro pasabajos de primer orden \(\tau_h\dot{\hat{\mathbf h}}_p+\hat{\mathbf h}_p=\mathbf h_{p,\text{raw}}\).

- **Código**, líneas 428-434:
  ```matlab
  L1 = 3.0;   % ganancia capa 1
  L2 = 4.5;   % ganancia capa 2
  e_obs = yH(:,k) - yHe(:,k);
  yHe_dot = yHp(:,k) - L1*sqrt(abs(e_obs)).*sign(e_obs);
  yHp_dot = -L2*sign(e_obs);
  yHe(:,k+1) = yHe(:,k) + yHe_dot*deltat;
  yHp(:,k)   = yHp(:,k) + yHp_dot*deltat;
  ```
  Esto **no es** el estimador de dos etapas descrito en el manuscrito: es un **diferenciador tipo super-twisting** (estructura `sqrt|e|·sign(e)` + integral de `sign(e)`), arquitectura distinta a un filtro pasabajos lineal. Además:
  - `yHe(:,k+1)` se propaga correctamente al instante siguiente,
  - pero **`yHp(:,k)` se sobreescribe en el instante actual** (`yHp(:,k) = yHp(:,k) + yHp_dot*deltat`) en lugar de escribirse en `yHp(:,k+1)`. Esto es exactamente el bug que señala el Revisor 1 ("its derivative state is updated at the current sample index rather than propagated to the next sample"), y contamina toda señal derivada de `yHp` (velocidad de payload, RMSE de velocidad, \(v_\perp\), Tablas 2 y 3 — ver tarea #4 del plan de revisión).

  **Acción:** reemplazar por el estimador de dos etapas de las Ecs. `h_raw`-`h_p_filter` (parámetros `epsilon1 = 0.14` ya existe en el código, línea 44, pero no se usa en este bloque — parece un remanente de una versión anterior del estimador). Si se prefiere mantener una estructura tipo diferenciador robusto (super-twisting) en vez del filtro lineal del manuscrito, hay que **reescribir la Sección "Closed-Loop Error Analysis"** del paper para incluir la nueva Asunción 1 (estimación rápida) sobre este estimador no lineal, lo cual es más trabajo. Recomendado: implementar el estimador lineal de dos etapas tal como está en el manuscrito, ya que es el que sustenta el Teorema 1 actual.

---

## B. Compensador dinámico (inner loop, Super-Twisting adaptativo)

### B.1 Arquitectura del control (Ec. `eq:AccionControl`)

- **Manuscrito:**
  \[
  \mathbf u = \hat{\boldsymbol\kappa}_u^{-1}\big(-\boldsymbol\gamma\tilde{\mathbf v}+\dot{\mathbf v}_d+\hat{\boldsymbol\kappa}_v\mathbf v\big) - \mathbf w
  = \boldsymbol\phi\hat{\boldsymbol\theta} - \mathbf w,\qquad
  \mathbf w = \mathbf K_1\sqrt{\|\mathbf s\|}\,\mathrm{sign}(\mathbf s) + \int \mathbf K_2\,\mathrm{sign}(\mathbf s)\,dt
  \]
  Es una **ley de control basada en inversión del modelo del vehículo** (Ec. `eq:ModeloLucas`, \(\ddot{\mathbf x}=\kappa_u\mathbf u-\kappa_v\dot{\mathbf x}+\delta\)): el comando `u` (normalizado, en \([-1,1]\)) se calcula invirtiendo el modelo con las estimaciones \(\hat\kappa_u,\hat\kappa_v\), menos la corrección robusta `w`.

- **Código**, líneas 590-597:
  ```matlab
  sqrt_s_xy = sqrt(abs(s_xy(:,k))) .* sign(s_xy(:,k));
  nu_xy = nu_xy - K2_xy*sign(s_xy(:,k))*deltat;
  u_st_xy = -K1_xy*sqrt_s_xy + nu_xy;

  a_cmd_xy = UcDotFil_xy(:,k) - Lambda_xy*e_v_xy + u_st_xy;

  yULocal(1:2,k) = yUcFil(1:2,k) + tau_d*a_cmd_xy;
  ```
  **No hay inversión del modelo del vehículo en absoluto.** No aparecen `κ̂u`, `κ̂v`, ni el término `φθ̂`. En cambio, el código:
  1. calcula una "pseudo-aceleración" `a_cmd` = derivada filtrada de la referencia − `Λ·e_v` + corrección ST,
  2. la integra con una constante de tiempo heurística `tau_d = 0.05` para obtener una **corrección de velocidad**,
  3. suma esa corrección a la referencia cinemática filtrada `yUcFil` para obtener el comando final `yULocal`, que se envía directo a `quad1` (el simulador).

  Esta es una arquitectura de controlador de velocidad tipo PI+ST **sin modelo**, estructuralmente distinta a la ley con inversión de modelo que sustenta el Teorema 2 (que depende explícitamente de \(\kappa_u\) para el análisis de Lyapunov, ver Ec. `eq:SuperficieFinal`). Es el punto más grave del crítico #1: no es un problema de signos o de un término de más, es que **falta por completo el bloque de inversión de modelo con \(\hat\kappa_u,\hat\kappa_v\)** que la teoría analiza.

  **Acción (la de mayor esfuerzo del plan):** reescribir el bloque del compensador dinámico para que calcule explícitamente:
  ```matlab
  % kappa_u_hat, kappa_v_hat: diag 1x2 (o 1x4 si se extiende a z/yaw), actualizados por adaptación (ver B.3)
  a_vec  = UcDot_xy + gamma_st*e_v_xy;              % "a" del manuscrito: a = v_d_dot + gamma*v_tilde
  w_xy   = K1_xy*sqrt_s_xy - nu_xy;                  % (ver signo, B.2)
  u_xy   = kappa_u_hat_inv * (-gamma_st*e_v_xy + a_vec + kappa_v_hat*Xp(1:2)) - w_xy;  % Ec. AccionControl
  yULocal(1:2,k) = u_xy;   % u es el comando normalizado, no una corrección aditiva sobre yUcFil
  ```
  y luego mapear `u_xy` (comando normalizado en \([-1,1]\), según el modelo de Lucas ICUAS2014) a lo que espera `quad1`. Esto requiere revisar qué interfaz espera `quad1.mexw64`/`SimQuad.h` (¿recibe velocidades o comandos normalizados de actitud?) para no romper la integración con el simulador C++. **Este punto merece confirmación tuya antes de tocarlo**, porque cambia la superficie de acoplamiento con el simulador de bajo nivel.

### B.2 Signo del término robusto `w`

- **Manuscrito:** `u = φθ̂ − w`, con \(w=\mathbf K_1\sqrt{\|s\|}\,\mathrm{sign}(s)+\int K_2\,\mathrm{sign}(s)\,dt\) (ambos términos con signo **positivo** dentro de `w`, que luego se resta).
- **Código:** `nu_xy = nu_xy - K2_xy*sign(s_xy)*deltat` (integra con signo negativo) y `u_st_xy = -K1_xy*sqrt_s_xy + nu_xy` (el término proporcional entra negativo, el integral entra tal cual `nu_xy` que ya es negativo). Hay que verificar con cuidado si el signo neto de `u_st_xy` coincide con `−w` del manuscrito una vez fijada la definición de `s` — **no puede resolverse por inspección aislada**, depende de si `s_xy` en el código tiene el mismo signo que `s` en el manuscrito (ver B.4). Queda pendiente de verificación numérica una vez unificada la definición de `s`.

### B.3 Ley de adaptación — el núcleo del crítico #1

- **Manuscrito**, Ecs. `eq:param_def`, `eq:regressor`, `eq:ajuste2`: se adaptan
  \[
  \theta_{1,i}=\frac1{\kappa_{u,i}},\qquad \theta_{2,i}=\frac{\kappa_{v,i}}{\kappa_{u,i}}
  \]
  (parámetros del **modelo del vehículo**, 2 por canal, 4 canales ⇒ \(\theta\in\mathbb R^8\)), vía
  \[
  \dot{\tilde{\boldsymbol\theta}} = -\boldsymbol\gamma^{-1}\boldsymbol\phi^T\mathbf s - \mathbf A\tilde{\boldsymbol\theta},\qquad \mathbf A=\boldsymbol\gamma\boldsymbol\phi^T\boldsymbol\phi
  \]
  con \(\boldsymbol\phi\in\mathbb R^{4\times8}\) bloque-diagonal, entradas \(a_i=\dot v_{d,i}+\gamma\tilde v_i\) y \(\dot x_i\).

- **Código**, líneas 570-585:
  ```matlab
  Phi = [ (UcDotFil_xy(1,k) - Lambda_xy(1,1)*e_v_xy(1)), Xp(1), 0, 0; ...
          0, 0, (UcDotFil_xy(2,k) - Lambda_xy(2,2)*e_v_xy(2)), Xp(2) ];
  Gamma_st = diag([gamma1, gamma2, gamma1, gamma2]);
  Theta = -Gamma_st * (Phi' * s_xy(:,k));
  k1_vec = [K1_xy(1,1); K1_xy(2,2)] + deltat*[Theta(1); Theta(3)];
  k2_vec = [K2_xy(1,1); K2_xy(2,2)] + deltat*[Theta(2); Theta(4)];
  ```
  Se adaptan **directamente `K1_xy`, `K2_xy`** (las ganancias del super-twisting), no `κ̂u,κ̂v`. Además:
  - Solo 2 canales (x,y), no 4 — consistente con el crítico "the inner compensator acts only on the horizontal channels".
  - No hay término `−A·θ̃` (el de fuga cuadrática); la ley es un integrador puro `Theta = -Gamma*Phi'*s` seguido de *clamping* (`min/max` contra `K1_min/max`, `K2_min/max`) — un mecanismo de anti-windup ad hoc que reemplaza el término `−Aθ̃` que sí tiene justificación de Lyapunov en el manuscrito.
  - `Gamma_st` en el código es una matriz de **ganancias de adaptación** (análoga a `Γ` del manuscrito), pero el regresor `Phi` no tiene la estructura de `eq:regressor` (ahí las columnas son \(a_i\), \(\dot x_i\) por canal con estructura bloque-diagonal 4×8; acá es 2×4 con una estructura distinta, mezclando el rol de `a_i` y sin separar \(\theta_1,\theta_2\) por su significado físico).

  **Acción:** esta es la reescritura central. Reemplazar el bloque completo por:
  ```matlab
  phi_i = [a_i, x_dot_i];     % por canal, 1x2
  % Phi bloque-diagonal 2x4 (para 2 canales) o 4x8 (para 4 canales)
  A_mat = Gamma_st * (Phi' * Phi);
  ThetaTilde_dot = -inv(Gamma_st)*Phi'*s_xy(:,k) - A_mat*ThetaTilde;
  ThetaTilde = ThetaTilde + ThetaTilde_dot*deltat;
  theta1 = theta1_hat0 + ThetaTilde(1:2:end);   % 1/kappa_u_hat
  theta2 = theta2_hat0 + ThetaTilde(2:2:end);   % kappa_v_hat/kappa_u_hat
  kappa_u_hat = 1./theta1;
  kappa_v_hat = theta2 .* kappa_u_hat;
  ```
  con `K1_xy`, `K2_xy` **fijas** (no adaptadas), sujetas únicamente a la condición de la Ec. `eq:gain_condition`/`K2≥‖κu⁻¹δ̇‖` del Teorema 2. Esto requiere decidir valores iniciales razonables de \(\hat\kappa_u,\hat\kappa_v\) (el código ya tiene `Ku_inicial = diag([0.8 0.8 1.0 0.4])`, `Kv_inicial = diag([0.2 0.2 0.3 0.1])`, líneas 379-395 — parecen ser justamente para esto, posiblemente remanentes de una versión anterior del script que sí hacía inversión de modelo, y no están conectados al bloque de adaptación actual).

### B.4 Definición de la superficie deslizante `s`

- **Manuscrito:** \(\mathbf s=\tilde{\mathbf v}+\boldsymbol\gamma\int\tilde{\mathbf v}\,dt\), con \(\tilde{\mathbf v}=\mathbf v-\mathbf v_d\) (o el error de velocidad correspondiente).
- **Código**, líneas 562-566:
  ```matlab
  ErrorAcum_xy = (1 - lambda_i*deltat)*ErrorAcum_xy + e_v_xy*deltat;
  s_xy(:,k) = e_v_xy + Lambda_xy*ErrorAcum_xy;
  ```
  Estructuralmente análogo (`e_v` en vez de `v_tilde`, `Lambda_xy` en vez de `γ`), **pero el integrador tiene fuga** (`(1-lambda_i*deltat)`), lo cual no está en la definición de `s` del manuscrito. Esta fuga es otro elemento no declarado en la teoría (similar a A.3): o se quita del código, o se agrega al manuscrito y se re-verifica que el Teorema 2 sigue siendo válido con esta modificación (la fuga generalmente ayuda a la robustez práctica pero cambia la prueba de Lyapunov).

---

## C. Resumen de decisiones pendientes antes de editar

| # | Punto | Opción A (ajustar código a teoría, por defecto) | Opción B (ajustar teoría al código) |
|---|-------|--------------------------------------------------|--------------------------------------|
| A.1 | `tanh` + término `Kvload` en `v_base` | Quitar ambos, dejar lineal | Agregar saturación y término de velocidad a `eq:base_reference`, re-verificar Teorema 1 |
| A.2 | Signo de `h_ref_h` | Corregir signo en código | (no aplica, es un bug objetivo) |
| A.3 | Saturación `tanh` en `comp_xy` | Quitar del código | Agregar al manuscrito, re-verificar Teorema 1 |
| A.5 | Estimador STA vs. filtro lineal 2 etapas | Reemplazar por filtro lineal del manuscrito | Documentar el estimador STA como nueva Asunción, re-verificar Teorema 1 |
| B.1 | Falta inversión de modelo `κ̂u,κ̂v` | Reescribir compensador con inversión de modelo (mayor esfuerzo, afecta interfaz con `quad1`) | Reescribir Teorema 2 para el compensador tipo PI+ST sin modelo (pierde el rol de `κu,κv` como parámetros adaptados, hay que reescribir toda la Sec. 2.4) |
| B.3 | Adaptación de `K1,K2` vs. `κ̂u,κ̂v` | Adaptar `κ̂u,κ̂v` según `eq:ajuste2`, fijar `K1,K2` | Redefinir formalmente `θ` como `[K1;K2]` y rehacer Teorema 2 en esos términos |
| B.4 | Fuga en el integrador de `s` | Quitar fuga | Agregar a la definición de `s` y re-verificar Teorema 2 |

Dado que ya se decidió "el manuscrito es correcto", la recomendación por defecto es la **Opción A en todas las filas**. B.1 es, con diferencia, el cambio de mayor riesgo técnico porque toca la interfaz con el simulador C++ (`quad1`/`SimQuad.h`) y no puede validarse sin correr MATLAB — algo que solo vos podés hacer en tu máquina.

---

## D. Próximo paso sugerido

1. Confirmar fila por fila de la tabla en la Sección C (o aceptar todas en bloque).
2. Empezar por A.2 y A.5 (bugs de signo/timing, bajo riesgo, alto impacto en Tablas 2-3) antes de tocar B.1-B.4 (cambio arquitectónico del compensador, mayor riesgo).
3. Reescribir el compensador dinámico (B.1-B.4) en una copia del script (o rama) para poder comparar resultados antes/después.
4. Vos corrés la simulación corregida en tu máquina (MATLAB + `quad1.mexw64`/`SimQuad1.exe`) y compartís los `.mat`/figuras resultantes para regenerar Tablas 2-3 y decidir si el Teorema 2 tal como está redactado sigue siendo válido con el nuevo estimador y compensador.
