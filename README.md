# Robust Adaptive Control of UAV–Suspended Payload Systems  
### Simulation Code for Controller Validation

This repository contains the MATLAB/C++ simulation package used to validate the cascade control architecture presented in:

> Kamel, S.; Rosales, C.; Boubaker, S.; Alsubaei, F. S.; Freire Oliveira, E.; Rossomando, F.  
> *Robust Adaptive Control of Underactuated UAV-Suspended Payload Systems: A Symmetry Approach for Last-Mile Delivery.*

The software reproduces the high-fidelity numerical experiments reported in the manuscript: payload-oriented kinematic control, adaptive Super-Twisting (A-STA) dynamic compensation, wind-disturbance robustness tests, and the sliding-mode baseline comparison.

---

## 1. Control architecture (overview)

The closed-loop scheme is hierarchical:

1. **Outer loop (payload-oriented kinematics).**  
   A reference generator shapes the UAV velocity command from payload tracking error and cable geometry. Quasi-static swing compensation employs the horizontal cable reference  
   \(\mathbf{h}_h^\star \propto (l/g)\,\ddot{\mathbf{x}}_{d,h}\)  
   with gains \(K_h\) and \(K_{dh}\).

2. **Inner loop (adaptive Super-Twisting).**  
   A velocity-tracking compensator based on the Super-Twisting algorithm, with online adaptation of the parameterized vehicle gains \(\hat{\boldsymbol{\kappa}}_u\), \(\hat{\boldsymbol{\kappa}}_v\), enforces the kinematic velocity commands under parametric mismatch and external disturbances.

3. **Plant.**  
   The quadrotor–cable–payload dynamics are integrated in a C++ simulator with a discretized mass–spring–damper cable and a cubic payload subject to aerodynamic drag (Salinas et al. unified simulator interface).

---

## 2. Repository layout

```
uav-payload-asta-validation/
├── README.md
├── .gitignore
└── simulations/          # MATLAB scripts, MEX gateways, and physics sources
    ├── CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026.m
    ├── CompensacionConsiderandoCarga_STMejorado_Claude.m
    ├── Comparativa_SMC_Kusznir_vs_STA_Propuesto.m
    ├── Comparativa_Klausen_Backstepping_vs_STA_Propuesto.m
    ├── ReferenciasCarga_ST.m
    ├── ExportarFigurasComparativa.m
    ├── plot_uav_payload_*.m
    ├── export_uav_simulation_video.m
    ├── cube_plot.m
    ├── *.cpp / SimQuad.h
    ├── *.mexw64              # prebuilt MEX (Windows)
    └── SimQuad1.exe          # physics executable (Windows)
```

All runnable scripts assume that MATLAB’s current directory is `simulations/`.

---

## 3. Requirements

| Component | Requirement |
|-----------|-------------|
| MATLAB | R2016b or newer (R2020a+ recommended for high-resolution figure export) |
| OS (prebuilt binaries) | Windows ×64 (`*.mexw64`, `SimQuad1.exe`) |
| Compiler (rebuild only) | A MATLAB-supported C/C++ compiler (`mex -setup`) |

No System Identification Toolbox or Simulink model is required for the scripts included here.

---

## 4. Build instructions (optional)

Precompiled Windows MEX files and `SimQuad1.exe` are shipped for convenience. To rebuild the MEX gateways from source, open MATLAB in `simulations/` and run:

```matlab
mex load1.cpp
mex laser1.cpp
mex fforce1.cpp
mex rope1.cpp
mex simulation1.cpp
mex force1.cpp
mex quad1.cpp
mex wind1.cpp
```

Ensure `SimQuad1.exe` remains on the MATLAB path or in the working directory (the scripts launch it automatically).

---

## 5. Reproduction of manuscript experiments

Nominal physical parameters match the simulation table of the paper unless a scenario explicitly overrides them:

- UAV / cable / payload: \(l = 2\,\mathrm{m}\), \(m_L = 0.10\,\mathrm{kg}\) (wind case: \(0.30\,\mathrm{kg}\)), cube edge \(0.10\,\mathrm{m}\)
- Control sampling: \(T_c = 0.02\,\mathrm{s}\) (\(50\,\mathrm{Hz}\))

### 5.1 Kinematic swing compensation only

Corresponds to the ascending-circular payload trajectory under the outer-loop law (inner A-STA disabled).

```matlab
cd simulations
CompensacionConsiderandoCarga_STMejorado_8_Graficas_7_4_2026
```

Default flags: `UsarKinematicoII = true`, `UsarCompensadorDinamico = false`, `TrayectoriaDefault = 1`.

### 5.2 Cascaded kinematic + adaptive Super-Twisting controller

Same script as §5.1; enable the inner loop before execution:

```matlab
UsarCompensadorDinamico = true;
UsarAdaptacionST        = true;
```

(or edit the corresponding assignments near the top of the file). This configuration underlies the quantitative Kin vs. Kin+A-STA comparison reported in the manuscript.

### 5.3 Robustness under scheduled wind disturbances

Lemniscate (figure-eight) reference with two wind intervals:

```matlab
% In CompensacionConsiderandoCarga_STMejorado_Claude.m, set:
TrayectoriaDefault     = 5;      % inclined lemniscate (a=10, b=7.5)
payload                = 0.3;    % kg
UsarVientoProgramado   = true;
UsarDosBloquesViento   = true;
UsarKinematicoII       = true;
UsarCompensadorDinamico = true;
UsarAdaptacionST       = true;
```

Then run:

```matlab
CompensacionConsiderandoCarga_STMejorado_Claude
```

Wind blocks (default magnitudes in the script): lateral `[0 6 0]` m/s and oblique `[3√2  3√2  0]` m/s over disjoint time windows.

### 5.4 Baseline comparison: first-order SMC (Kusznir, 2020)

Fair comparison with a shared payload-aware kinematic outer loop; only the inner velocity law differs (A-STA vs. boundary-layer first-order SMC):

```matlab
Comparativa_SMC_Kusznir_vs_STA_Propuesto
% optional figure post-processing:
ExportarFigurasComparativa
```

### 5.5 Additional benchmark (not required for the submit manuscript)

```matlab
Comparativa_Klausen_Backstepping_vs_STA_Propuesto
```

compares the proposed scheme against a backstepping baseline under nominal, wind, and parametric-load scenarios.

---

## 6. Reference trajectories

`ReferenciasCarga_ST.m` generates differentiable payload references. Indices used in the paper:

| Index | Trajectory | Role in the manuscript |
|------:|------------|------------------------|
| 1 | Ascending circle / ellipse | Nominal tracking and SMC benchmark |
| 5 | Inclined lemniscate (`a=10`, `b=7.5`, `c=0.025·2π`) | Wind-disturbance study |

---

## 7. Outputs

When `ExportarResultadosPaper = true` (or `ExportarResultados = true` in the comparative scripts), results are written under `simulations/ResultadosPaper/`:

- figures (`.fig` / `.png`, depending on the script),
- time-series archives (`.mat`) with tracking errors, oscillation metrics \(\alpha\), \(\rho\), \(\|v_\perp\|\), sliding variables, and adaptive gains.

Generated result folders are ignored by Git (see `.gitignore`).

---

## 8. File roles (quick reference)

| File | Role |
|------|------|
| `..._STMejorado_8_Graficas_7_4_2026.m` | Canonical nominal validation (Kin / Kin+A-STA) |
| `..._STMejorado_Claude.m` | Wind / lemniscate robustness study |
| `Comparativa_SMC_Kusznir_vs_STA_Propuesto.m` | Manuscript SMC baseline |
| `Comparativa_Klausen_....m` | Supplementary backstepping comparison |
| `ReferenciasCarga_ST.m` | Reference generators |
| `*.cpp`, `SimQuad.h`, `SimQuad1.exe` | High-fidelity plant and shared-memory interface |

---

## 9. Reproducibility notes

- Simulations are deterministic for fixed references and prescribed wind profiles.
- Controller parameters are initialized without plant identification; nominal \(\hat{\boldsymbol{\kappa}}_u\) and \(\hat{\boldsymbol{\kappa}}_v\) intentionally mismatch the true vehicle maps.
- Prebuilt binaries target **Windows ×64**. On other platforms, rebuild the MEX files and provide a compatible `SimQuad1` executable.
- Filenames retain the historical Spanish/project naming used during manuscript preparation to preserve script–figure provenance.

---

## 10. Citation

If you use this code, please cite the associated article (full bibliographic details upon publication) and, for the simulator backbone, the unified cable-suspended transport simulator referenced therein.

---

## 11. License and contact

Research source code accompanying the manuscript. Redistribution and reuse should acknowledge the original authors and the cited simulator. For scientific correspondence related to the control design, contact the corresponding author listed in the paper.
