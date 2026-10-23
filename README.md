# Panda Task 7 gravity release experiments

This package uses the original TA `panda_multibody.slx` and its mesh files. It prepares a new model, runs three initial arm poses with zero internal damping and repeats them with 1 N m s/rad of internal damping. The source model is preserved.

The setup code has been checked against the supplied model's stored parameters. It has not been executed in MATLAB or Simscape in the authoring environment. Generated experiment results must come from running the package on your computer.

## Run

1. Extract the complete ZIP.
2. Open MATLAB and make this folder the Current Folder.
3. Run `run_task7` in the Command Window. No previous lab workspace is needed.

MATLAB, Simulink, Simscape and Simscape Multibody are required. To build and inspect before running, use `run_task7(false)`. To repeat all cases over a longer interval, use `run_task7(true,40)`. Every invocation creates a new timestamped output folder.

## Setup

- Gravity is [0 0 -9.81] m/s^2. Seven arm joints have no applied motor torque, computed motion, zero internal springs and zero initial velocity targets.
- Three initial arm poses are saved as columns in `initial_poses_columns_rad.csv`. The first is the ready pose; the other two are stated experiment assumptions within the supplied limits.
- The copied model retains the supplied inertias, finger joints, mechanical connections and limit stiffness, damping and transition widths. Existing stop damping can dissipate energy even when internal damping is zero.
- Mesh paths are repaired to this package's local files.
- Velocity and both limit-torque sensors are added, one at a time, using the new unconnected physical signal port. Existing position sensing is preserved.
- Solver: variable-step ode15s, maximum step 0.001 s, relative tolerance 1e-5, absolute tolerance 1e-7, default stop time 20 s. These are starting experiment settings, not specified lab requirements. Task 6's 0.02 s fixed step does not apply here.
- Damping units kg*m^2/(rad*s) are equivalent to N*m*s/rad.

## Results

`task7_results/<timestamp>/` contains the copied configured SLX, experiment settings, an aggregate `task7_summary.csv` and a `task7_joint_results.csv` with per-joint limit times and final states. Each successful `pose1_B0` etc. subfolder contains the raw simulation MAT, trajectory CSV, summary, joint results and joint position/velocity plots. A failed case has `error.txt`; it is not reported as a successful measurement. The six cases are attempted sequentially and completed results are saved immediately.

A limit hit is the first recorded magnitude of stop torque above 1e-4 N m. Its reported time has the logged time resolution. NaN means no detected hit. Stops are compliant and may allow penetration, so exact equality with a position bound is not required.

Settled means every recorded arm joint speed stays below 0.01 rad/s throughout the final second. A false result should be reported and investigated; extend the run if needed. B = 1 does not guarantee settling within 20 s. Check temporal convergence before accepting close timing comparisons.

The last successful simulation is available in Mechanics Explorer / Multibody Explorer. Inspect its final frame and describe the actual posture and active stops. Other cases can be rerun by opening the saved configured model, setting the position targets and damping to the recorded case and pressing Run. The package does not invent final posture descriptions or add ground contact or self-collision.

Upload the resulting summary CSV files and any error.txt here so the measured Task 7 report can be completed.

## GitHub references

After completing the experiments, keep the scripts, model, meshes, selected results and README together. Cite the repository URL plus the tested commit SHA or release tag in the report. Prefer commit-specific file links for reproducible references. The report already reserves these fields. No repository is created or published by this script.

## References

- Lab Sheet 1, Selected Topics in Robotics and AI, Ain Shams University, Fall 2026.
- TA source: panda_multibody.slx and panda_meshes from the supplied Lab_1.zip.
- https://www.mathworks.com/help/sm/ref/revolutejoint.html
- https://www.mathworks.com/help/simulink/slref/add_line.html
