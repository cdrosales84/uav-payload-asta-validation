# Numerical validation of an adaptive super-twisting controller for a UAV with a suspended payload

This repository is the numerical companion to the manuscript *Robust Adaptive Control of Underactuated UAV-Suspended Payload Systems: A Symmetry Approach for Last-Mile Delivery* (Kamel, Rosales, Boubaker, Alsubaei, Oliveira Freire, and Rossomando). It implements the cascade controller of the revised manuscript, co-simulates it with the high-fidelity simulator SimQuad, and writes the time series, metrics, and figures used in the paper.

The closed-loop law evaluated at each sample is `controlador_paso.m`. Every numerical value used in a run is taken from `cfg_paper.m` and stored with the result, so a figure or a table can be traced to the configuration that produced it. Equation labels in the source comments (`cable_vector`, `base_reference`, `swing_compensation`, `AccionControl`, `ajuste2`, and the others) refer to the revised manuscript source `SN_Payload.tex`. The file `VesionOriginaldelPaper.pdf` is the original manuscript.

## Control problem

A single quadrotor carries a payload on a cable. The payload must track a prescribed trajectory, while the cable tilt is not actuated. The quadrotor is the actuation platform: the controller commands its velocity, and the onboard velocity and attitude loops remain inside the simulator.

The design is a cascade.

1. An outer kinematic loop builds a velocity reference for the vehicle. It combines feedforward from the payload reference, proportional action on the payload tracking error, and a swing-compensation term that drives the horizontal cable vector toward the quasi-static tilt implied by the reference acceleration.
2. A cable-velocity estimator reconstructs \(\dot{\mathbf{h}}\) from the measured cable vector \(\mathbf{h} = \mathbf{x} - \mathbf{x}_L\), because the payload rate is not a sensor output.
3. An inner dynamic compensator acts on the two horizontal channels, expressed in the vehicle yaw frame. It is an adaptive super-twisting law (A-STA) with a model inversion of the velocity loop. The vertical velocity and the yaw rate pass the kinematic command through.

The command sent to the simulator is \(u = [v_{xh},\, v_{yh},\, v_{zh},\, \dot{\psi}_z]^\top\), in metres per second and radians per second, in the yaw frame.

### Outer loop

With \(l\) the cable length and \(g\) gravity, the quasi-static horizontal cable reference and its derivative are

\[
\mathbf{h}_s^\star = \frac{l}{g}\,\mathbf{a}_{d,h}, \qquad \dot{\mathbf{h}}_s^\star = \frac{l}{g}\,\mathbf{j}_{d,h}.
\]

The base velocity and the swing compensation, before the startup ramp \(\sigma(t) = 1 - \exp(-t/T_s)\), are

\[
\mathbf{v}_{\mathrm{base}} = \mathbf{v}_d + \begin{bmatrix}\dot{\mathbf{h}}_s^\star \\ 0\end{bmatrix} + K_{\mathrm{traj}}\,\mathbf{e}_x,
\]

\[
\mathbf{v}_{\mathrm{comp}} = \begin{bmatrix} -K_h(\mathbf{h}_h - \mathbf{h}_s^\star) - K_{dh}(\hat{\dot{\mathbf{h}}}_h - \dot{\mathbf{h}}_s^\star) \\ 0 \end{bmatrix},
\]

and the kinematic command is \(\mathbf{v}_c = \sigma(t)\,(\mathbf{v}_{\mathrm{base}} + \mathbf{v}_{\mathrm{comp}})\). The yaw rate is \(\dot{\psi}_c = \dot{\psi}_d + k_\psi \tanh(k_\psi^t e_\psi)\). The case `sin_swing` retains only \(\mathbf{v}_{\mathrm{base}}\) and disables the ramp; that law is the divergent reference case of the manuscript.

Nominal gains in `cfg_paper.m`: \(K_{\mathrm{traj}} = \mathrm{diag}(1.8, 1.8, 0.7)\), \(K_h = \mathrm{diag}(4, 4)\), \(K_{dh} = \mathrm{diag}(0.3, 0.3)\), \(T_s = 5\,\mathrm{s}\), \(k_\psi = 1\), \(k_\psi^t = 0.5\).

### Cable-velocity estimator

The raw rate \(\dot{\mathbf{h}}_{\mathrm{raw}} = \varepsilon_1^{-1} K_1 (\mathbf{h} - \hat{\mathbf{h}})\) is passed through a first-order filter of time constant \(\tau_h\), discretized as \(y_k = y_{k-1} + \bigl(1 - e^{-\Delta t/\tau_h}\bigr)(x_k - y_{k-1})\), and \(\hat{\mathbf{h}}\) is integrated by Euler. Nominal values: \(\varepsilon_1 = 0.14\), \(K_1 = 3 I_3\), \(\tau_h = 0.06\,\mathrm{s}\). Metrics compare this estimate with the central difference of the simulator payload position.

### Inner loop

On the horizontal channels, \(\tilde{\mathbf{v}} = \mathbf{v}_h - \mathbf{v}_{d,h}\) and the sliding variable is

\[
\mathbf{s} = \tilde{\mathbf{v}} + \beta \int \tilde{\mathbf{v}}\,dt.
\]

The super-twisting injection and the model inversion are

\[
\mathbf{w} = K_1 \sqrt{\lVert\mathbf{s}\rVert}\,\mathrm{sign}(\mathbf{s}) + \boldsymbol{\nu}, \qquad \dot{\boldsymbol{\nu}} = K_2\,\mathrm{sign}(\mathbf{s}),
\]

\[
\mathbf{u}_{xy} = \kappa_u^{-1}\bigl(\dot{\mathbf{v}}_{d,h} - \beta\tilde{\mathbf{v}} + \kappa_v \mathbf{v}_h - \mathbf{w}\bigr).
\]

When the inner law is `asta`, the matrices \(\kappa_u\) and \(\kappa_v\) adapt by Euler integration of

\[
\dot{\kappa}_u^\top = \gamma^{-1} \mathbf{u}_{xy}\mathbf{s}^\top - \lVert\mathbf{s}\rVert\,\kappa_u^\top, \qquad \dot{\kappa}_v^\top = -\gamma^{-1} \mathbf{v}_h\mathbf{s}^\top - \lVert\mathbf{s}\rVert\,\kappa_v^\top,
\]

with \(\gamma = 1\) unless `cfg.ad.gamma` is set. The law `sta_fijo` freezes the matrices at their initial values. Both start at \(\kappa_u(0) = 0.5\,I_2\) and \(\kappa_v(0) = 0.75\,I_2\). The logged parameter \(\boldsymbol{\theta}\) is not itself the adapted state: for each horizontal channel it stores \((1/\kappa_u,\, \kappa_v/\kappa_u)\).

Nominal inner-loop gains: \(\beta = 0.08\), \(K_1 = [0.3,\, 0.3]^\top\), \(K_2 = [0.45,\, 0.45]^\top\). The ablation `smc` replaces the super-twisting law by the boundary-layer sliding-mode law \(u = -K_{\mathrm{SMC}}\,\mathbf{s}/(\lVert\mathbf{s}\rVert + \phi)\), with \(K_{\mathrm{SMC}} = [1.2,\, 1.2]^\top\) and \(\phi = 0.05\). The law `ninguno` forwards the kinematic command and leaves the inner compensator out.

`cfg_paper.m` also records filter time constants, a boundary-layer width `phi_bl`, a washout, a payload-velocity gain \(K_v\), and a projected adaptation law in \(\boldsymbol{\theta}\) (\(\Gamma\), \(\sigma\), \(\Theta\)). Those fields are saved with each run and are written into the parameter table. The sample law in `controlador_paso.m` does not read them.

## Numerical platform

Paper runs use SimQuad (`SimQuad1.exe` together with the MEX interfaces `quad1`, `rope1`, `simulation1`, and `wind1`) through `planta_simquad.m`. The interface is real-time co-simulation on Windows. The control period is \(T_c = 0.02\,\mathrm{s}\) (50 Hz). The integrator time is the simulator clock, so the sample interval is not exactly uniform. Because the co-simulation is not deterministic, each scenario is repeated; tables report the mean and the standard deviation over the repetitions that run to completion.

The cable is a chain of particles. The payload position is the last particle returned by `rope1`. The UAV state (position, inertial velocity, yaw, yaw rate, and simulator time) comes from `quad1`. Wind is applied simultaneously to the vehicle, the cable, and the payload.

`quad1` and `rope1` are read from separate shared-memory buffers. On the order of one sample in five hundred, the payload buffer is one frame behind the vehicle. `corregir_lectura_carga.m` detects that sample by linear extrapolation of the last two accepted cable vectors and replaces \(\mathbf{x}_L\) before it enters the controller. The corrected position is what the controller and the logged field `xl` use. `xl_crudo` keeps the original reading, and `corr_xl` marks the corrected samples. A run is stopped if the cable tilt exceeds \(80^\circ\) or the payload position error exceeds \(10\,\mathrm{m}\).

`planta_sustituta.m` is a MATLAB point-mass model with the same call interface (first-order velocity loop, spring-damper cable, linear drag, one sample of computation delay). It exists so that the scripts can be exercised without SimQuad, including under Octave. Results obtained with `backend` equal to `sustituta` are code checks. The manuscript figures and tables are the SimQuad runs.

The nominal plant is a cable of length \(l = 2\,\mathrm{m}\) and a payload of mass \(m_L = 0.10\,\mathrm{kg}\). The wind scenario raises the payload mass to \(0.30\,\mathrm{kg}\).

## Repository layout

```
cfg_paper.m                 parameter record and scenario definitions
controlador_paso.m          one sample of the cascade controller
sim_payload_asta.m          one closed-loop run; returns the series R
sim_controlador.m           same loop, for an arbitrary controller handle
metricas_payload.m          metrics M from simulator ground truth
run_paper_experiments.m     scenario sweep, tables, and manuscript figures
referencia_carga.m          circle, lemniscate, and the step maneuver
planta_simquad.m            SimQuad interface
planta_sustituta.m          surrogate plant for code checks
corregir_lectura_carga.m    one-frame payload-reading correction
figuras_escenario.m         diagnostic figures of one run
figuras_paper.m             manuscript figures (mean and min–max band)
graficar_resultados_paper.m diagnostic figures from saved .mat files
plot_uav_payload_frame.m    one 3-D frame (vehicle, cable, payload)
tabla_parametros_latex.m    LaTeX parameter table from cfg_paper
controlador_omar.m          Omar (2022) delayed anti-swing baseline
parametros_omar.m           baseline gains, delay, and PX4 position gains
ajustar_omar.m              ISH tuning of the baseline
ish_omar.m                  integral swing index of Omar, Eq. (4)
comparar_omar_vs_asta.m     PX4 tracker, Omar, and the proposed law
figuras_omar.m              figures for the baseline comparison
barrido_ganancias.m         one-at-a-time gain sensitivity
SimQuad1.exe, *.mexw64      Windows simulator
quad1.cpp, rope1.cpp,       MEX sources
  simulation1.cpp, wind1.cpp
VesionOriginaldelPaper.pdf  original manuscript
results/<scenario>/         archival outputs
```

## Scenarios

Scenarios are the `switch` in `cfg_paper.m`. A call `cfg_paper('E4_circ_asta')` returns the nominal plant with that scenario’s reference, wind, payload mass, and choice of outer and inner law.

| Identifier | Condition |
| --- | --- |
| `E1_circ_sin_swing` | Circle. Kinematic law without swing compensation and without the inner loop. The run is cut by the divergence test; it is the reference failure case. |
| `E2_circ_cinematico` | Circle. Outer kinematic loop only. |
| `E3_circ_sta_fijo` | Circle. Cascade with the super-twisting matrices held at their initial values. |
| `E4_circ_asta` | Circle. Proposed controller (kinematic loop and A-STA). |
| `E5_lemn_viento` | Lemniscate, \(m_L = 0.30\,\mathrm{kg}\), A-STA. Lateral wind \(6\,\mathrm{m\,s}^{-1}\) on \([20, 40]\,\mathrm{s}\) and oblique wind \(3\sqrt{2}\,\mathrm{m\,s}^{-1}\) on each horizontal axis on \([60, 80]\,\mathrm{s}\). |
| `E6_circ_smc` | Circle. Inner-loop ablation: boundary-layer sliding mode in place of A-STA. |
| `E7_masa_<grams>_{fijo,asta}` | Lemniscate, no wind, duration \(160\,\mathrm{s}\), payload mass in grams. Compares frozen and adaptive inner loops from the mismatched initial condition \(\boldsymbol{\theta}(0)\) recorded in the scenario. The mass sweep is part of `run_paper_experiments` (50, 100, 200, and 300 g). |

The circle (manuscript reference) has radius \(3\,\mathrm{m}\), period \(10\,\mathrm{s}\), and a slow climb \(z = 6 + 0.1\,t\). The lemniscate is the closed-form curve in `referencia_carga.m`, with horizontal scales \(a = 10\,\mathrm{m}\) and \(b = 7.5\,\mathrm{m}\) and a vertical oscillation of amplitude \(1\,\mathrm{m}\) about \(z = 6\,\mathrm{m}\). Both references return position, velocity, acceleration, and jerk; the jerk enters \(\dot{\mathbf{h}}_s^\star\). Yaw follows the horizontal velocity heading.

## Recorded signals and metrics

Each file `raw/<scenario>_rep<k>.mat` contains the series `R` and the metrics `M`. `R.cfg` is the `cfg_paper` structure of that run. Columns of `R` are samples; time is `R.t`, in seconds from the first simulator sample.

| Field | Content |
| --- | --- |
| `x`, `v`, `psi` | UAV position, inertial velocity, and yaw |
| `xl`, `xl_crudo`, `corr_xl` | Corrected payload position, raw reading, and correction flag |
| `xd`, `vd`, `ad`, `jd`, `psid` | Payload reference and its derivatives, and reference yaw |
| `h`, `hp` | Cable vector and estimated cable velocity |
| `hs`, `hsd` | Quasi-static cable reference and its derivative |
| `ex`, `epsi` | Payload position error and yaw error |
| `vt`, `s`, `nu`, `w` | Inner-loop velocity error, sliding variable, integral state, and injection |
| `theta` | Logged parameter \([1/\kappa_{u,x},\, \kappa_{v,x}/\kappa_{u,x},\, 1/\kappa_{u,y},\, \kappa_{v,y}/\kappa_{u,y}]^\top\) |
| `u` | Command \([v_{xh},\, v_{yh},\, v_{zh},\, \dot{\psi}]^\top\) |
| `viento` | Wind \([w_x,\, w_y,\, w_z]^\top\) applied at that sample |
| `motivo`, `completo` | Termination cause (`fin`, `divergencia`, or `simulador_detenido`) and completion flag |

`metricas_payload.m` differentiates the simulator payload position with central differences on the simulator clock. It does not use the cable-velocity estimate for the tracking scores. The window is \(t \ge\) `cfg.met.t_inicio` (\(1.5\,\mathrm{s}\) in the paper scenarios; the Omar comparison uses \(5\,\mathrm{s}\), after the startup ramp). Reported quantities are the RMSE of the payload position and velocity errors; the RMS and the maximum of the cable angle \(\alpha\), the horizontal displacement \(\rho\), the velocity orthogonal to the cable, and the deviation from the quasi-static tilt; the RMS error, delay, and gain of the cable-velocity estimate against ground truth; the RMS of \(\tilde{\mathbf{v}}\), \(\boldsymbol{\nu}\), and \(\dot{u}\); the fraction of saturated samples; and the final \(\boldsymbol{\theta}\). The series behind the swing figures (`vl`, `hdot`, `alpha`, `rho`, `vperp`, `swing`) are returned in `M.serie` and are not fields of `R`.

## Reproducing a paper run

Requirements: MATLAB on Windows, with this directory as the current folder. `SimQuad1.exe` and the MEX files are in the repository root. The experiment driver also adds a sibling directory `../simulations` to the path when that directory exists.

One scenario, one repetition:

```matlab
run_paper_experiments('escenarios', {'E4_circ_asta'}, 'n_rep', 1);
```

A call without arguments runs `E1` through `E6` and the four-mass `E7` sweep, five repetitions each, and writes `results/<yyyymmdd_HHMMSS>/`. That directory contains `raw/`, `resumen.csv`, `tabla_metricas.tex`, `tabla_parametros.tex`, `figuras/`, and `paper_figs/`. The folders `results/E*` in the tree are the per-scenario extracts used by this repository. A full SimQuad campaign is a real-time co-simulation and takes on the order of the simulated horizon times the number of runs.

```matlab
run_paper_experiments('backend', 'sustituta', 'n_rep', 1, 'escenarios', {'E4_circ_asta'});
```

exercises the same driver on the surrogate plant.

## Figures

Manuscript figures are the mean across repetitions, interpolated onto a common time grid, with a min–max band. `figuras_paper` names the files as the `\includegraphics` keys of `SN_Payload.tex`.

```matlab
figuras_paper('results/E4_circ_asta');   % writes results/E4_circ_asta/paper_figs/
```

The function looks for `raw/<scenario>_rep*.mat` inside the folder it is given. Figures that overlay two laws need both scenarios’ files in the same `raw/` directory.

Diagnostic figures from a saved run:

```matlab
% In graficar_resultados_paper.m, point CarpetaResultados at a raw/ folder, then run the script.
CarpetaResultados = fullfile(pwd, 'results', 'E4_circ_asta', 'raw');
```

With `ExportarFiguras = true` the script writes PNG and FIG files under `<raw>/figuras/`. The series are payload position and velocity errors, cable angle and orthogonal velocity, planar tracking, adaptive parameters, the sliding variable, vehicle against payload, and the command. On `E5_lemn_viento` the wind intervals are shaded.

A single stored run, and one three-dimensional frame at the sample nearest a requested time:

```matlab
S = load('results/E4_circ_asta/raw/E4_circ_asta_rep1.mat');
[~, k] = min(abs(S.R.t - 20));
plot_uav_payload_frame(S.R, k);
```

Vector PDFs under `results/<scenario>/paper_figs/`, together with `resumen.csv` and the `.tex` tables, are the archival products. Raw `.mat` files and the PNG, FIG, and EPS renders are excluded by `.gitignore` and remain on the machine that ran the experiment. Redrawing a curve requires the `.mat` file or a new run.

## External baseline

The published comparison is the delayed-feedback anti-swing controller of Omar, *Applied Sciences* 12(3):1706, 2022 (doi:10.3390/app12031706), against the proposed cascade and against the same position tracker with the anti-swing term removed. The inner-loop sliding-mode ablation (`E6_circ_smc`) is a separate comparison: it changes only the compensator of this paper. The Omar law is an independent published method.

In that paper the anti-swing correction is added to the position setpoint, and a PX4 cascade tracks the setpoint. SimQuad already closes velocity and attitude, which is the role of the PX4 inner loops, and it accepts a yaw-frame velocity command. The missing position loop is the proportional law of the PX4 multicopter position controller at its default gains, \(\mathrm{MPC\_XY\_P} = 0.95\) and \(\mathrm{MPC\_Z\_P} = 1.0\):

\[
\mathbf{v}_{\mathrm{sp}} = \dot{\mathbf{x}}_d + K_p\bigl(\mathbf{p}_{\mathrm{sp}} - \mathbf{x}\bigr), \qquad \mathbf{p}_{\mathrm{sp}} = \mathbf{x}_d + \begin{bmatrix}0\\0\\L\end{bmatrix} + \begin{bmatrix}x_s\\ y_s\\ 0\end{bmatrix}.
\]

The corrections \(x_s\) and \(y_s\) are Omar’s equations (1)–(2), evaluated on a linearly interpolated history of the cable angles at \(t - \tau\). The pendulum period \(T_L = 2\pi/\sqrt{g/L}\) is equation (3). The tuning index is the integral swing \(\mathrm{ISH} = \int(\phi_L^2 + \theta_L^2)\,dt\), equation (4). `ajustar_omar.m` minimizes that index over the sign, the gain, and the delay on the step maneuver of Omar’s Section 5.2, and writes `omar_ajuste.mat`. The same numerical values are then frozen for every scenario, as the proposed-controller gains are.

Both laws share the startup ramp, the yaw law, the command bound, the delayed-reading correction, and the loop in `sim_controlador.m`. Omar’s sensor is an inclinometer: the baseline uses the measured cable vector and does not estimate \(\dot{\mathbf{h}}\).

```matlab
A = ajustar_omar();                                              % writes omar_ajuste.mat
comparar_omar_vs_asta('escenarios', {'E4_circ_asta'}, 'n_rep', 1)
T = comparar_omar_vs_asta();                                     % circle and wind lemniscate, 5 repetitions
```

If `omar_ajuste.mat` is absent, the comparison uses \(K = 0.6\) and \(\tau = T_L/4\) and records that fact in the parameter file. Output goes to `results/comp_omar_<date>/`: a LaTeX table per scenario (PX4 tracker, Omar, proposed law; mean \(\pm\) standard deviation; relative difference; Welch \(p\)-value), `resumen_omar.csv`, the raw series, and overlay figures of \(\alpha\), \(\lVert\tilde{\mathbf{x}}\rVert\), and \(\lVert v_\perp\rVert\). An interrupted campaign resumes from the last unfinished folder with the same arguments. A simulator stall retries the run; a divergence does not.

## Gain sensitivity

`barrido_ganancias.m` repeats the nominal closed loop while changing one recorded gain at a time, and writes `results/barrido_<date>/` with a sensitivity table and convergence figures. The linear pendulum model used to draw the Routh boundary lives in that script. As with the parameter table, a gain affects the SimQuad trajectory only when `controlador_paso.m` reads the corresponding field.

## Citation

If you use this code, please cite the manuscript:

S. Kamel, C. Rosales, S. Boubaker, F. S. Alsubaei, E. Oliveira Freire, and F. G. Rossomando, “Robust adaptive control of underactuated UAV-suspended payload systems: a symmetry approach for last-mile delivery.”

The external baseline is:

H. M. Omar, “Hardware-in-the-loop simulation of a time-delayed anti-swing controller for a quadrotor with a suspended load,” *Applied Sciences*, vol. 12, no. 3, 1706, 2022. doi:10.3390/app12031706.
