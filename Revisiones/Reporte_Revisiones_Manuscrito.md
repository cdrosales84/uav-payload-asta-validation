# Reporte de Revisiones — Plan de Acción para Revisión

**Manuscrito:** "Robust Adaptive Control of Underactuated UAV-Suspended Payload Systems: A Symmetry Approach for Last-Mile Delivery"
**Revista:** Scientific Reports
**ID de sumisión:** 8f58d8bc-94ff-4c7a-b6d0-29e1897b5988
**Editor responsable:** Dr. Mouquan Shen
**Recomendación:** Request Revision
**Fecha límite sugerida:** 11 sep 2026

---

## Resumen ejecutivo

El Revisor 1 identifica problemas **estructurales y de fondo**: el controlador simulado no coincide con el analizado matemáticamente, dos teoremas centrales no están correctamente demostrados, y varias métricas reportadas dependen de un estimador con errores. Esto significa que, en su forma actual, **el manuscrito no valida lo que afirma validar**. Los Revisores 2 y 3 son mucho más favorables y piden sobre todo mejoras de claridad, consistencia terminológica y rigor en pruebas secundarias. La revisión necesaria es sustancial (no cosmética), pero está bien delimitada: corregir la coherencia código–teoría, arreglar las dos demostraciones, corregir el estimador y regenerar resultados, y eliminar las afirmaciones no sustentadas (incluyendo, posiblemente, el propio marco de "simetría" del título).

---

## Nivel 1 — CRÍTICO (bloquean la aceptación; invalidan resultados centrales)

Todos provienen del **Revisor 1**. Sin resolver estos puntos, ni la revisión de forma ni el ajuste de redacción bastarían.

| # | Problema | Acción requerida |
|---|----------|-------------------|
| 1 | El controlador en el código público difiere del analizado en el manuscrito (retroalimentación saturada + término de velocidad del payload no presente en Ec. 8; signo distinto en Ec. 12; filtros/saturación/integral adicionales; compensador interno solo en canales horizontales; se adaptan κᵤ, κᵥ en el texto pero K₁, K₂ en el código) | Unificar teoría e implementación: o se rehace el análisis para el controlador realmente simulado, o se reimplementa exactamente el controlador de las Ecs. (8)–(40) y se vuelve a simular. Las Ecs. (29)–(40) y el Teorema 2 deben corresponder al código. |
| 2 | Las dinámicas del lazo externo (Ecs. 16–17) no se derivan del sistema UAV–payload; la ecuación de error del cable es circular (ξ_h depende de la derivada del propio error); el Teorema 1 no prueba estabilidad del lazo cerrado real | Derivar explícitamente las dinámicas de error a partir del modelo físico del péndulo/cable. Redefinir ξ_h sin dependencia circular. Reescribir el Teorema 1 y su prueba. |
| 3 | El Teorema 2 tiene incoherencias de signo (Ecs. 26–34); la ley de adaptación usa el error de parámetro θ̃ (que depende del parámetro real desconocido → no implementable); la prueba de Lyapunov no incluye el término integral super-twisting como estado aumentado; la condición sobre K₂ es insuficiente; el argumento de Barbalat y la convergencia de parámetros carecen de las hipótesis necesarias | Reescribir la ley de adaptación en términos de cantidades medibles (no de θ̃). Aumentar el estado de Lyapunov para incluir el término integral. Corregir signos. Establecer condiciones suficientes reales sobre K₂ y las hipótesis de excitación persistente para la convergencia de parámetros. |
| 4 | El estimador de velocidad del cable (Ecs. 9–11) no corresponde al código: signos de corrección inconsistentes, la derivada se actualiza en el instante actual en vez de propagarse al siguiente; esta misma señal alimenta velocidad del payload, RMSE de velocidad y v⊥, contaminando Tablas 2 y 3 | Corregir el estimador, validarlo contra la verdad de referencia del simulador, y **regenerar todos los resultados afectados** (Tablas 2 y 3 y las figuras derivadas). |
| 5 | "Restauración de simetría de desempeño" no está definida matemáticamente ni probada cuantitativamente; no hay transformación de simetría, medida de equivarianza/invarianza, ni comparación bajo perturbaciones rotadas | Definir rigurosamente qué se entiende por "simetría" (p. ej. equivarianza ante rotación de la perturbación de viento) y aportar una prueba cuantitativa (comparación pareada bajo disturbios rotados). Si no se puede sustentar, **retirar "simetría" del título y de las afirmaciones de novedad**. |
| 6 | La comparación con Kusznir y Smoczek es una variante interna (SMC de capa límite) inspirada en ese trabajo, no una implementación fiel; además difiere en la normalización | No presentarla como "validación externa contra el controlador publicado". Reescribir como ablation study propio, o implementar fielmente el controlador original citado. |
| 7 | Lista de afirmaciones que deben eliminarse o reformularse tal como están: "rigorosamente demostrado" (estabilidad del lazo cerrado completo); "restauración de simetría de desempeño"; "invariancia de la trayectoria del payload" ante viento/estado del cable; "validación experimental"; "aptitud para implementación embebida/bajo consumo" sin evidencia de tiempo de cómputo | Revisar todo el manuscrito (resumen, introducción, conclusiones) y suavizar/eliminar cada una de estas afirmaciones para que coincidan con lo efectivamente demostrado. |

---

## Nivel 2 — MAYOR (necesarios para rigor y reproducibilidad; no bloquean per se pero son obligatorios para publicación)

| # | Fuente | Problema | Acción requerida |
|---|--------|----------|-------------------|
| 8 | Rev. 1 (#5) | Ec. (44) con signo de eje vertical inconsistente con la definición de h y con el código; Ec. (47) no coincide con el periodo de trayectoria usado en simulación; la segunda derivada implementada de la lemniscata es incorrecta; las matrices de parámetros nominales reportadas no son las usadas por el controlador interno liberado; faltan scripts exactos que reproduzcan cada figura/tabla | Auditar y corregir cada ecuación contra el código real. Publicar scripts de reproducción exactos (uno por figura/tabla), sin reconfiguración manual. |
| 9 | Rev. 1 (#8) | Validación limitada a corridas deterministas únicas: sin barrido de masa de payload, sin experimentos estocásticos/repetidos de viento, sin evaluación de ruido de sensor, retardo, saturación de actuador ni costo computacional; sin experimento físico | Añadir: (a) barrido de masa del payload demostrando adaptación real; (b) réplicas estocásticas de viento con estadística (media/varianza); (c) análisis de sensibilidad a ruido/retardo/saturación; (d) reporte de costo computacional (tiempo por paso). Ajustar las afirmaciones de robustez/eficiencia/"listo para last-mile" a lo efectivamente probado. |
| 10 | Rev. 1 (#9) | Repositorio excluye archivos `.mat` y demás salidas; datos subyacentes a figuras/tablas no disponibles | Publicar datos crudos y procesados, configuraciones exactas, versiones de software, y una release/DOI persistente. Incluir declaraciones **separadas** de Data Availability y Code Availability. |
| 11 | Rev. 2 (#8) | La "convergencia en tiempo finito" del super-twisting solo se prueba vía V̇≤0 + Barbalat, lo cual da convergencia asintótica, no en tiempo finito | Aportar una prueba de estabilidad en tiempo finito propiamente dicha (p. ej. función de Lyapunov no suave / desigualdad diferencial tipo V̇ ≤ -cV^α), o reformular la afirmación como "convergencia asintótica". |
| 12 | Rev. 2 (#9) | Posible inconsistencia dimensional/notacional: φ∈ℝ^{4×8}, θ∈ℝ^8, pero las expresiones de Lyapunov y la ley de adaptación deben revisarse en transposiciones y dimensiones | Verificar y declarar explícitamente las dimensiones de todos los términos en las Ecs. (31), (34) y (38)–(40). |

---

## Nivel 3 — MODERADO (organización, consistencia y claridad expositiva)

| Fuente | Punto |
|--------|-------|
| Rev. 2 (#3) | Unificar terminología: "adaptive robust control", "adaptive super-twisting", "A-STA", "dynamic compensator" deben definirse una vez y usarse consistentemente. |
| Rev. 2 (#4) | Reorganizar la Sección 2: presentar supuestos, modelo del sistema, diseño del controlador y análisis de estabilidad en secuencia lógica (posiblemente en subsecciones nuevas). |
| Rev. 2 (#5) | Aclarar explícitamente qué objetivo cubre el controlador cinemático y cuál el compensador dinámico, y cómo interactúan; consolidar esta explicación (actualmente repartida en varios lugares). |
| Rev. 2 (#6) | Vincular directamente cada afirmación cualitativa de mejora de desempeño con la figura/índice cuantitativo correspondiente; evitar repetición. |
| Rev. 3 (#2) | Aclarar mejor la estructura de la ley adaptativa. |
| Rev. 3 (#3) | Justificar por qué el regresor debe ser persistentemente excitante para que el error de estimación de parámetros converja a cero (relacionado también con el punto crítico #3). |

---

## Nivel 4 — MENOR (presentación / edición)

- **Rev. 1 (#5, subset editorial):** títulos duplicados de Figs. 4 y 6; Figs. 11 y 12 invertidas en la discusión; frase "basic pneumatic controller" (probable error, no aplica a UAV); uso inconsistente de "super-twisting", "supertorsion" y "time-based sliding mode"; instrucciones de plantilla de la revista sin resolver; DOIs erróneos o duplicados en referencias.
- **Rev. 2 (#1):** eliminar abreviaturas innecesarias del resumen y palabras clave (LMD, STA).
- **Rev. 2 (#2):** acortar párrafos extensos de la introducción.
- **Rev. 2 (#7):** revisar y mejorar los pies de figura.
- **Rev. 3 (#1):** ampliar la introducción con comparación frente a métodos existentes.
- **Rev. 3 (#4):** incluir figuras de convergencia del error bajo distintas ganancias, para ilustrar el efecto de los parámetros.
- **Rev. 3 (#5):** citar trabajo reciente relacionado sugerido por el revisor (control adaptativo por aprendizaje iterativo basado en factor de equilibrio; control adaptativo por redes neuronales para sistemas con retardo desconocido en feedback estricto).

---

## Hoja de ruta sugerida (orden de trabajo)

1. **Reconciliar código y teoría** (crítico #1) — esto condiciona todo lo demás, incluidas las pruebas de los Teoremas 1 y 2.
2. **Corregir el estimador de velocidad del cable y regenerar Tablas 2–3** (crítico #4).
3. **Rehacer las demostraciones de los Teoremas 1 y 2** con las dinámicas correctas y la ley de adaptación implementable (críticos #2 y #3).
4. **Decidir el destino del marco de "simetría"**: sustentarlo rigurosamente o retirarlo del título/abstract/conclusiones (crítico #5).
5. **Reencuadrar la comparación con Kusznir y Smoczek** como ablation propio (crítico #6).
6. **Auditar ecuaciones vs. código** (Ecs. 44, 47, segunda derivada de la lemniscata, matrices nominales) y publicar scripts de reproducción (mayor #8).
7. **Ampliar la validación**: barrido de masa, réplicas estocásticas de viento, sensibilidad a ruido/retardo/saturación, costo computacional (mayor #9).
8. **Completar disponibilidad de datos y código** con DOI persistente (mayor #10).
9. **Suavizar todas las afirmaciones sobreestimadas** listadas en el crítico #7, alineándolas con lo que quedó efectivamente demostrado tras los pasos anteriores.
10. **Resolver puntos de rigor secundario**: prueba de tiempo finito (mayor #11) y consistencia dimensional (mayor #12).
11. **Reorganizar y unificar terminología** (moderados de Revisor 2 y 3).
12. **Limpieza editorial final**: figuras, pies de figura, DOIs, abreviaturas, párrafos largos (menores).

Al enviar la revisión, acompañar con una **respuesta punto por punto** que referencie cada comentario numerado de los tres revisores y del editor, indicando la página/ecuación modificada.

---

*Nota: dado que los puntos críticos 1–7 (Revisor 1) implican rehacer el análisis de estabilidad y regenerar simulaciones, esta revisión requiere trabajo técnico sustancial, no solo edición de texto. Conviene contactar a la editora asistente (Pooja Gade, srep@nature.com) si se necesita una extensión del plazo del 11 de septiembre de 2026, indicando el ID de sumisión.*
