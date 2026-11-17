# DeepMechDyn

**Open-source simulation and deep-learning tools for machine design.**

DeepMechDyn couples the mechanics of hydraulic actuators, valves and machine components with rigid and flexible multibody system dynamics. It provides data-driven surrogate models that make these simulations fast enough for design, virtual prototyping and digital twins. The code is written in MATLAB and Python, using the open-source multibody code [Exudyn](https://github.com/jgerstmayr/EXUDYN).

---

## Contents

| Folder | Language | What it contains |
|---|---|---|
| `modules/` | MATLAB | Reusable libraries: `hydraulicsLib` (valves, effective bulk modulus, friction, control signals), `multibodyLib` (rigid bodies, constraints, formulations), `integratorsLib` (RK4 and other solvers), `visualization` |
| `rigidMBD-PVG32/` | MATLAB | Semi-recursive rigid multibody model of a 2-DOF hydraulic crane with a new pressure-compensated valve model (PVG32) and several effective bulk modulus models for aerated fluid |
| `tutorials/` | MATLAB | Teaching examples: hydraulic cylinder lifting a mass, parameter variation, kinematic and dynamic analysis tasks |
| `tests/` | MATLAB | Simple verification models (e.g. single pendulum) |

---

## Getting started

### MATLAB
1. Clone the repository:
   ```bash
   git clone https://github.com/qkhadim22/DeepMechDyn.git
   ```
2. In MATLAB, add the libraries to the path:
   ```matlab
   addpath(genpath('modules'));
   ```
3. Run a tutorial, e.g. `tutorials/tutorial_hydraulic_cylinder_lifting_mass.m`, or the crane model `rigidMBD-PVG32/MATLABCode/run_PATU_model.m`.

### Python
1. Python 3.10 or newer.
2. Install the dependencies:
   ```bash
   pip install exudyn numpy scipy matplotlib scikit-learn torch
   ```
Large datasets and trained models are not stored in this repository. Measurement data from the laboratory crane are needed to run some examples.


---

## Methods

- **Pressure-compensated valve model.** Accounts for compensator saturation at the spool-stroke ends and for low-pressure, partially aerated conditions when no counterbalance valve is used. Validated against measurements from a forestry crane with PVG32 valves.
- **Effective bulk modulus.** Several models for aerated hydraulic fluid, selectable in `hydraulicsLib`.
- **SLIDE.** Physics-inspired deep learning for estimating the dynamic response and structural deflection of hydraulically actuated flexible multibody systems (see the related repositories below).

---

## Publication

The valve model and crane simulations in `rigidMBD-PVG32/` belong to:

- Q. Khadim, A. Garcia-Agundez Blanco, A. Mikkola, M. R. Hansen. A novel model for pressure-compensated valves under low-pressure, aerated fluid conditions: experimental validation in a forestry crane. *IEEE/ASME Transactions on Mechatronics*, under revision.

If you use DeepMechDyn, please cite this paper once it is published.

---

## Related work and repositories

Related methods by the author and collaborators are available in separate repositories:

- **SLIDE method:** P. Manzl, A. Humer, Q. Khadim, J. Gerstmayr (2026), *Mechanics Based Design of Structures and Machines*, https://doi.org/10.1080/15397734.2025.2559325. Code: https://github.com/peter-manzl/SurrogateModelDynamics2024
- **SLIDE for structural deflection:** Q. Khadim et al. (2025), *Mechanical Systems and Signal Processing*, https://doi.org/10.1016/j.ymssp.2025.113220. Code: https://github.com/qkhadim22/SLIDE-EstimatingStructuralDeflection
- **Exudyn** multibody code: https://github.com/jgerstmayr/EXUDYN

---

## Acknowledgements

Development of the open-source code was funded by the **Fuugin säätiö (Fuug Foundation)**. The research was supported by Business Finland projects (SIMKAT, VIIMA) at the University of Oulu. Collaborators on the valve model: LUT University (Prof. A. Mikkola, Dr. A. Garcia-Agundez Blanco) and University of Agder (Prof. M. R. Hansen).

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).

## Contact

Qasim Khadim, University of Oulu: qasim.khadim@oulu.fi
