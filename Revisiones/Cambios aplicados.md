# Cambios aplicados — versión fiel al manuscrito

**Archivo nuevo:** `simulations/CompensacionConsiderandoCarga_STMejorado_FielManuscrito.m`**Archivo base (sin modificar):** `simulations/CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m`**Referencia de discrepancias:** `Revisiones/Mapeo_Codigo_Ecuaciones_Compensador.md`

Se generó una copia nueva (no se sobrescribió el script original) con todos los cambios de la tabla de decisiones (Opción A en todas las filas: el código se ajusta al manuscrito). Resumen de lo aplicado:

## A. Lazo cinemático externo

- **A.1** `v_base`: ahora lineal, `v_base = ẋ_d + K_traj·x̃` (Ec. `base_reference`). Se quitó el `tanh` de saturación y el término extra de velocidad de payload (`Kvload`).
- **A.2** Signo de `h_ref_h`: corregido a `-(l/g)·ẍ_d,h` (Ec. `href`).
- **A.3** Se quitó la saturación `tanh` no declarada sobre `comp_xy` (Ec. `swing_compensation`).
- **A.5** El diferenciador tipo super-twisting (L1,L2) fue reemplazado por el estimador de dos etapas del manuscrito (Ecs. `h_raw`, `h_hat_dyn`, `h_p_filter`): ganancia alta `K1_est/epsilon1` + filtro pasabajos de primer orden con `tau_h = filtro_hp`. Se corrigió además el bug de timing: `yHp(:,k)` ya no se sobreescribe en el instante actual, se calcula limpio en cada paso y `yHe` se propaga a `k+1` como corresponde.

## B. Compensador dinámico (inner loop)

- **B.1** Se reemplazó por completo el bloque de "pseudo-aceleración + corrección aditiva" por la ley de control con **inversión de modelo** de la Ec. `AccionControl`: `u = φθ̂ − w`, ahora para los **4 canales** (x, y, z, yaw), no solo x,y.
- **B.2/B.4** La superficie deslizante `s` ya no tiene integrador con fuga (antes `lambda_i`); ahora es la definición literal `s = ṽ + Λ∫ṽ dt` (Ec. sliding surface), también para los 4 canales.
- **B.3** La ley de adaptación ya no ajusta `K1,K2` (ganancias del super-twisting). Ahora adapta `θ̂ = [1/κ̂u,ᵢ ; κ̂v,ᵢ/κ̂u,ᵢ]` por canal (Ec. `param_def`), con el regresor `φᵢ=[aᵢ, ẋᵢ]` (Ec. `regressor`) y la estructura de la Ec. `ajuste2`. `K1`,`K2` del super-twisting quedaron **fijas** (`K1_4`, `K2_4`), sujetas a la condición del Teorema 2.
- Las variables muertas `Ku`, `Kv`, `Ku_inicial`, `Kv_inicial` (que existían en el archivo original pero nunca se usaban realmente) ahora sí están conectadas: `Ku_hat`/`Kv_hat` son las estimaciones de `κ̂u`, `κ̂v` que evolucionan en cada paso de la simulación. Esto también reactivó de forma coherente la función `CargarGananciasPrevias`/`GuardarGananciasFinales` (antes guardaba/cargaba valores que no se usaban en el compensador).

## Advertencia importante (pendiente, no resuelta acá)

La Ec. `ajuste2` del manuscrito está escrita en términos del **error de parámetro** `θ̃` (que depende del parámetro verdadero `θ`, desconocido), por lo tanto **no es implementable tal cual** — esto es exactamente el crítico #3 del Revisor 1 (tarea "Rehacer Teorema 2" del plan de revisión, todavía no resuelta). Para que el código pudiera ejecutarse, se implementó un análogo con la misma estructura pero aplicado sobre la **estimación** `θ̂` (tipo *sigma-modification*, ver comentario en el código, bloque `if UsarAdaptacionST`). Esto significa:

- El código ahora es fiel a la **estructura algebraica** de las ecuaciones (regresor, parametrización, ley con término de fuga cuadrática `A(t)=Γφᵀφ`), pero **no** a la ley literal de la Ec. `ajuste2` tal como está escrita hoy en el manuscrito, porque esa ley no es ejecutable sin conocer `θ` verdadero.
- Cuando se resuelva la tarea "Rehacer Teorema 2" (reescribir la ley en cantidades medibles), hay que volver a este bloque y actualizarlo para que coincida exactamente con la nueva ecuación del paper.

## Otras notas técnicas

- Se verificó la interfaz con el simulador C++ (`SimQuad.h`/`quad1.cpp`): `quad1(1, u)` espera un vector `[v_xh, v_yh, v_zh, w_zh]` en **m/s** (velocidades en marco local), no comandos normalizados `[-1,1]` como describe literalmente el manuscrito para el modelo de Lucas (ICUAS 2014). El código ya enviaba directamente la referencia cinemática (en m/s) como comando, así que la nueva ley `u = φθ̂ − w` es dimensionalmente compatible con lo que el simulador espera — pero esto es una discrepancia adicional entre la descripción textual del manuscrito ("normalized commands... $u\\in[-1,1]$") y el simulador realmente usado, que podría valer la pena aclarar en el texto (ítem menor, no crítico).
- El archivo nuevo quedó con saltos de línea `LF` (Unix) en vez de `CRLF` (Windows) por el proceso de edición. MATLAB en Windows lee `LF` sin problemas; si Git te muestra el archivo entero como "modificado" por esto, es solo el fin de línea, no el contenido — avisame si preferís que lo vuelva a convertir a `CRLF` para que el diff sea más limpio.
- **No se ejecutó la simulación** (no hay MATLAB disponible en este entorno). El archivo no fue validado numéricamente. Antes de usarlo para regenerar resultados del
paper:
1. Corré el script en tu máquina y revisá errores de sintaxis/dimensiones.
2. Los valores de `K1_4`, `K2_4`, `gamma_ad`, `Ku_inicial`, `Kv_inicial` son estimaciones de partida razonables pero **probablemente necesiten resintonizarse**, porque la dinámica cerrada cambió completamente respecto al compensador anterior (ya no hay adaptación directa de K1/K2, así que el comportamiento transitorio va a ser distinto).
3. Comparar contra el script original (`..._8_Graficas_7_4_2026.m`) corriendo ambos con la misma trayectoria/semilla para ver qué tan distinto es el comportamiento.

## Alcance NO cubierto en este cambio

- `simulations/CompensacionConsiderandoCarga_STMejorado_Claude.m` (script de viento/lemniscata, §5.3 del README) **todavía usa la arquitectura vieja** (probablemente comparte el mismo problema del crítico #1). No se tocó.
- `simulations/Comparativa_SMC_Kusznir_vs_STA_Propuesto.m` y `Comparativa_Klausen_Backstepping_vs_STA_Propuesto.m` tampoco se actualizaron; como usan "STA Propuesto" como referencia, probablemente haya que sincronizarlos con la nueva arquitectura del compensador para que la comparación siga siendo justa.
- El crítico #2 (Teorema 1, dinámica del lazo externo) y el crítico #3 (Teorema 2, ley de adaptación implementable) son tareas de **matemática del paper**, no de código — siguen pendientes en el plan de revisión (tareas #2 y #3).

 
