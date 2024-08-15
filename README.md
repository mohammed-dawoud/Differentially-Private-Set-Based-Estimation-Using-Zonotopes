# Privacy-Preserving Set-Based Estimation Using Differential Privacy and Zonotopes
This repo is the code and data used to evaluate our [conference paper](https://ieeexplore.ieee.org/document/10178269) and [journal paper]() <br />
Mohammed M. Dawoud, Changxin Liu, Amr Alanwar, and Karl Henrik Johansson "Differentially-Private-Set-Based-Estimation-Using-Zonotopes" and "Privacy-Preserving Set-Based Estimation Using Differential Privacy and Zonotopes", respectively<br />
<br />
For large-scale cyber-physical systems, the collaboration of spatially distributed sensors is often needed to perform the state estimation process. Privacy concerns arise from disclosing sensitive measurements to a cloud estimator. To solve this issue, we propose a differentially private set-based estimation protocol that guarantees true state containment in the estimated set and differential privacy for the sensitive measurements throughout the set-based state estimation process within the central and local differential privacy models. Zonotopes are employed in the proposed differentially private set-based estimator, offering computational advantages in set operations. We consider a plant of a non-linear discrete-time dynamical system with bounded modeling uncertainties, sensors that provide sensitive measurements with bounded measurement uncertainties, and a cloud estimator that predicts the system’s state. The privacy-preserving noise perturbs the centers of measurement zonotopes, thereby concealing the precise position of these zonotopes, i.e., ensuring privacy preservation for the sets containing sensitive measurements. Compared to existing research, our approach achieves less privacy loss and utility loss through the central and local differential privacy models by leveraging a numerically optimized truncated noise distribution. The proposed estimator is perturbed by weaker noise than the analytical approaches in the literature to guarantee the same level of privacy, therefore improving the estimation utility. Numerical and comparison experiments with truncated Laplace noise are presented to support our approach.<br />

