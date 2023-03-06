# A New Method for Control Law Design in Disturbance Observer-Based Control

This repository is associated with the paper **[1]**. It includes the source files that generated the simulations of Section 7 and, also, a novel application to the classical magnetic levitation benchmark control problem introduced in **[2]**.

---

The simulation results indicate that the new control-law design for Disturbance Observer Based Controllers (DOBCs) proposed in **[1]** may offer notable advantages in terms of control tuning simplicity and closed-loop performance.

This repository aims to encourage other researchers to test and apply these results to their individual control problems. The repository provides sample code to build and compute the observer (DOB), to compute the DOB gain, as well as to compute the control-law matrices **[1]**.

---

The content is organized as follows:
- `./paper_simulations`: Contains the source code generating the simulations of Section 7 in **[1]**.
- `./magnetic_levitation_benchmark`: Contains a comparison between different DOBC designs by using the benchmark control problem reported in **[2]**. Here you can find sample code for building different control-laws for DOBCs as well as a new technique for optimizing the DOB gain.
- `./yalmip_and_sedumi`: You should add this folder to the MatLab path in order to execute the code in `./magnetic_levitation_benchmark`.

---

## References

[1] Castillo et al. (2023), "A New Method for Control Law Design in Disturbance Observer-Based Control," *ISA Transactions*.

[2] J. Yang, A. Zolotas, W. H. Chen, K. Michail, and S. Li, "Robust control of nonlinear MAGLEV suspension system with mismatched uncertainties via DOBC approach," *ISA Transactions*, vol. 50, no. 3, pp. 389-396, 2011.
