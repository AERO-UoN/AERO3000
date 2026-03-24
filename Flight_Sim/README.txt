========================================================================
  PC-9 FLIGHT DYNAMICS SIMULATOR
  AERO3000 Flight Dynamics — Assignment 3
  The University of Newcastle
========================================================================

------------------------------------------------------------------------
OVERVIEW
------------------------------------------------------------------------
This package implements a full nonlinear six-degrees-of-freedom (6-DOF)
flight dynamics simulator for the PC-9 trainer aircraft.  Given a trim
flight condition and a sequence of control inputs, it integrates the
equations of motion forward in time and plots the aircraft response.

The simulator is built around three core ideas taught in AERO3000:

  1. TRIM — find the control inputs and attitude that produce steady,
     unaccelerated flight at a specified speed and altitude.

  2. EQUATIONS OF MOTION — the nonlinear Newton-Euler equations that
     govern how forces and moments change the aircraft state.

  3. NUMERICAL INTEGRATION — the Runge-Kutta 4 (RK4) method that
     steps the equations of motion forward through time.

------------------------------------------------------------------------
PREREQUISITES
------------------------------------------------------------------------
- MATLAB R2018b or later (uses rad2deg, quat2euler, convlength)
- Aerospace Toolbox is NOT required (all transforms are coded from
  scratch in the Functions/ folder)

------------------------------------------------------------------------
QUICK START
------------------------------------------------------------------------
1. Open MATLAB and set the working directory to the folder containing
   main.m and this README.

2. Open main.m.

3. In the "STUDENT CONFIGURATION" section near the top, set:
     SPEED        — flight condition ('100' or '180')
     CONTROL_FILE — the manoeuvre to simulate (see list below)
     T_END        — simulation duration in seconds
     DT           — RK4 time step in seconds

4. Press Run (F5).  The console will print:
     • Trim verification results (state rates should be ~0)
     • Trimmed flight condition summary
     • Simulation progress
   Eight figures will appear showing the full aircraft response.

Control input files available (all use T_END = 150 s):
   control_input_E.mat       — elevator doublet      (longitudinal modes)
   control_input_A.mat       — aileron  doublet      (lateral-roll modes)
   control_input_R.mat       — rudder   doublet      (lateral-yaw modes)
   control_input_Phugoid.mat — phugoid excitation    (slow long-period mode)

To regenerate any of these files (or inspect how they were constructed),
run GenerateControlInputs.m.

------------------------------------------------------------------------
FILE STRUCTURE
------------------------------------------------------------------------
Assignment_3/
│
├── main.m                        Main script — start here
├── GenerateControlInputs.m       Regenerates the control input .mat files
├── LoadFlightDataPC9.m           PC-9 aircraft parameters
├── README.txt                    This file
│
├── InitialCondition1_100Kn.mat   Initial state at 100 kn / 20 m
├── InitialCondition2_180Kn.mat   Initial state at 180 kn / 1000 ft
│
├── control_input_E.mat           Elevator doublet       (T_END = 150 s)
├── control_input_A.mat           Aileron  doublet       (T_END = 150 s)
├── control_input_R.mat           Rudder   doublet       (T_END = 150 s)
├── control_input_Phugoid.mat     Phugoid excitation     (T_END = 150 s)
│
└── Functions/
    │
    │  — Simulation pipeline —
    ├── getstaterates.m     Top-level state-rate solver (iterates alpha/beta dot)
    ├── staterates.m        Computes full Xdot from X, U, and angle rates
    ├── calculateForces.m   Assembles all forces and moments on the aircraft
    ├── rungeKutta4.m       4th-order Runge-Kutta integrator
    ├── trim.m              Newton-Raphson trim solver
    │
    │  — Aerodynamics —
    ├── windforces.m        Lift and drag coefficients (wind-axis)
    ├── bodyforces.m        Side force and moment coefficients (body-axis)
    ├── propforce.m         Thrust from throttle and altitude
    ├── aeroangles.m        V, alpha, beta from body velocities
    ├── angularRates.m      alpha_dot, beta_dot from state rates
    ├── flowproperties.m    ISA atmosphere: rho and dynamic pressure Q
    ├── gravity.m           Weight vector resolved into body axes
    │
    │  — Attitude and rotation —
    ├── rotate321quat.m     DCM from quaternion (3-2-1 sequence)
    ├── rotatex/y/z.m       Elementary rotation matrices
    ├── euler2quat.m        Euler angles -> quaternion
    ├── quat2euler.m        Quaternion -> Euler angles
    │
    │  — Post-processing —
    ├── plotData.m          Generates all 8 result figures
    └── controls.m          Legacy stub — not called in this version

------------------------------------------------------------------------
THE MATHEMATICAL MODEL
------------------------------------------------------------------------

STATE VECTOR (13 elements)
   X = [u, v, w, p, q, r, q0, q1, q2, q3, x, y, z]'

   u, v, w   — body-axis velocity components (m/s)
                 u: forward,  v: right,  w: downward
   p, q, r   — body-axis angular rates (rad/s)
                 p: roll,     q: pitch,  r: yaw
   q0,q1,q2,q3 — quaternion attitude representation (dimensionless)
                 avoids the gimbal-lock singularity of Euler angles
   x, y, z   — inertial position (m)
                 x: North, y: East, z: positive downward
                 Altitude = -z

CONTROL VECTOR (4 elements)
   U = [delta_t, delta_e, delta_a, delta_r]'

   delta_t   — throttle (0 = idle, 1 = full power)
   delta_e   — elevator deflection (rad, positive = trailing-edge down)
   delta_a   — aileron  deflection (rad)
   delta_r   — rudder   deflection (rad)
   All surface limits: ±25 degrees

AERODYNAMIC ANGLES
   V     = sqrt(u² + v² + w²)         total airspeed (m/s)
   alpha = atan(w/u)                   angle of attack (rad)
   beta  = asin(v/V)                   sideslip angle (rad)

EQUATIONS OF MOTION — translational (Newton's 2nd law, body frame)
   u_dot = rv - qw + (Fgx + Fa_x + T) / m
   v_dot = pw - ru + (Fgy + Fa_y) / m
   w_dot = qu - pv + (Fgz + Fa_z) / m

   where Fg = weight components in body axes
         Fa = aerodynamic force components
         T  = thrust (acts along body x-axis)

EQUATIONS OF MOTION — rotational (Euler's equations, body frame)
   p_dot = c3*p*q + c4*q*r + c1*La + c2*Na
   q_dot = c7*p*r - c6*(p²-r²) + c5*Ma
   r_dot = c9*p*q - c3*q*r + c2*La + c8*Na

   where La, Ma, Na are the aerodynamic roll, pitch, yaw moments
   and c1…c9 are inertial constants derived from Ixx, Iyy, Izz, Ixz
   (see staterates.m for the full derivation)

QUATERNION KINEMATICS
   q0_dot = -0.5*(q1*p + q2*q + q3*r)
   q1_dot =  0.5*(q0*p - q3*q + q2*r)
   q2_dot =  0.5*(q3*p + q0*q - q1*r)
   q3_dot = -0.5*(q2*p - q1*q - q0*r)

   The quaternion norm is enforced (||q|| = 1) at every RK4 sub-step.

AERODYNAMIC MODEL
   Lift coefficient (longitudinal):
     CL = CL0 + CLa*alpha - CLad*alpha_dot_hat + CLq*q_hat + CLde*delta_e

   Drag (parabolic polar):
     CD = CD0 + k*CL²

   Side force, roll, pitch, yaw moments are similarly expressed as
   linear combinations of the stability and control derivatives stored
   in LoadFlightDataPC9.m (reference CG: 22% MAC).

   Non-dimensional rates:
     Longitudinal (reference = mean chord c):  q_hat     = q*c/(2V)
     Lateral      (reference = wing span  b):  p_hat     = p*b/(2V)
                                               r_hat     = r*b/(2V)

TRIM ALGORITHM (Newton-Raphson)
   Trim seeks alpha, delta_t, and delta_e such that
   u_dot = w_dot = q_dot = 0 (symmetric, unaccelerated flight).

   A numerical Jacobian is computed by finite-difference perturbation,
   then the Newton step  x_new = x - J\f(x)  is applied iteratively
   until the update is smaller than 1e-9.

NUMERICAL INTEGRATION (RK4)
   X(k+1) = X(k) + (1/6)*(A + 2B + 2C + D) * dt

   where A, B, C, D are the four Runge-Kutta slope estimates.
   The quaternion is renormalised after each sub-step.

------------------------------------------------------------------------
UNDERSTANDING THE PLOTS
------------------------------------------------------------------------
Figures are ordered in cause-effect sequence so you can trace the full
chain from pilot input through to aircraft response:

Figure 1 — Control inputs
   Throttle and all three surface deflections vs time.
   Always read this first — every feature in the response plots has a
   corresponding event here.

Figure 2 — Aerodynamic angles
   Angle of attack alpha and sideslip beta (deg).
   The immediate aerodynamic consequence of the control input.
   An elevator input changes alpha; a rudder or aileron input changes beta.

Figure 3 — Body angular rates
   Roll rate p, pitch rate q, yaw rate r (deg/s).
   The rotational response driven by the aerodynamic moments.
   Elevator doublet excites q; aileron excites p; rudder excites r and p.

Figure 4 — Euler attitude angles
   Bank phi, pitch theta, yaw psi (deg) — one subplot each.
   Separating them prevents a large phi from hiding a small theta, or
   vice versa, which happens with a shared axis and legend.

Figure 5 — Body-axis velocities
   Forward u, lateral v, and heave w components (m/s).
   Shows how the translational state evolves.  For symmetric manoeuvres
   (elevator only), v remains near zero throughout.

Figure 6 — Position and altitude (SI units)
   Altitude (-z) on its own subplot so small changes are visible.
   North (x) and East (y) share the lower subplot.

Figure 7 — Flight condition in aviation units
   Airspeed in knots and altitude in feet.
   Useful for cross-checking against the PC-9 Flight Manual data.

Figure 8 — 3-D flight path
   Spatial trajectory with start (green circle) and end (red square)
   markers.  Use View -> Rotate 3D in MATLAB to examine the path from
   any angle.  The path is nearly straight for small-amplitude inputs.

------------------------------------------------------------------------
RUNNING DIFFERENT SCENARIOS
------------------------------------------------------------------------
To simulate a different manoeuvre, change CONTROL_FILE and T_END in the
Student Configuration section of main.m.  The table below summarises
each file and its purpose:

All files use T_END = 150 s (already set in main.m).  Just change CONTROL_FILE:

   CONTROL_FILE = 'control_input_E.mat';
     Elevator doublet — shows short-period and phugoid longitudinal modes.

   CONTROL_FILE = 'control_input_A.mat';
     Aileron doublet — shows roll subsidence and Dutch-roll lateral modes.

   CONTROL_FILE = 'control_input_R.mat';
     Rudder doublet — shows Dutch-roll (yaw-driven) and roll coupling.

   CONTROL_FILE = 'control_input_Phugoid.mat';
     Brief elevator pulse — isolates the phugoid; ~6 cycles are visible
     in the airspeed and altitude plots (Figure 7).


To run at the higher-speed condition:

   SPEED = '180';

To compare two manoeuvres on the same axes, run the script twice with
different settings, then use "hold on" in the MATLAB command window
before the second run (or modify plotData.m to overlay traces).

------------------------------------------------------------------------
COMMON ISSUES
------------------------------------------------------------------------
Q: I get "Undefined function or variable" errors.
A: Make sure MATLAB's current folder is the Assignment_3/2023 directory
   (where main.m lives), not a subdirectory.

Q: The trim verification shows state rates that are not near zero.
A: Values below ~1e-6 are acceptable (numerical precision of the solver).
   Larger values suggest a problem with the aircraft data or initial state.

Q: The simulation takes a long time.
A: Try reducing T_END or increasing DT.  Note that DT > 0.02 s may cause
   numerical instability for the higher-speed (180 kn) condition.

Q: Figures are overwritten when I run the script a second time.
A: This is by design (fixed figure numbers 1-8).  If you want to keep
   previous plots, type "figure" in the command window before re-running,
   or save figures with File > Save As before the next run.

------------------------------------------------------------------------
REFERENCES
------------------------------------------------------------------------
Nelson, R.C., "Flight Stability and Automatic Control", 2nd ed.,
  McGraw-Hill, 1998.

Etkin, B. and Reid, L.D., "Dynamics of Flight: Stability and Control",
  3rd ed., Wiley, 1996.

Stevens, B.L., Lewis, F.L. and Johnson, E.N., "Aircraft Simulation
  and Control", 3rd ed., Wiley, 2016.

Anderson, J.D., "Introduction to Flight", 8th ed., McGraw-Hill, 2015.

========================================================================
