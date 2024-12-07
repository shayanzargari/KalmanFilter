# KalmanFilter
A beginner-friendly tutorial on the Kalman Filter, explaining its principles and mathematical foundations. Perfect for those looking to understand state estimation in dynamic systems with noise. ic systems with noise. Includes both theoretical explanations and code examples for easy application.

## Introduction to Kalman Filters
The Kalman filter is an optimal recursive estimation algorithm designed to estimate the states of a linear dynamical system in the presence of noise. Its ability to predict and correct based on noisy measurements makes it invaluable in fields such as:
- **Control Systems**
- **Navigation**
- **Signal Processing**
- **Robotics**

### Core Steps
1. **Prediction**: Predict the next state and its uncertainty.
2. **Correction (Update)**: Incorporate the measurement to refine the estimate.

---

## System Model
A linear dynamical system is represented as:
\[
\mathbf{x}_k = \mathbf{A} \mathbf{x}_{k-1} + \mathbf{B} \mathbf{u}_k + \mathbf{w}_k,
\]
\[
\mathbf{y}_k = \mathbf{C} \mathbf{x}_k + \mathbf{v}_k,
\]
where:
- \( \mathbf{x}_k \): State vector at time \( k \).
- \( \mathbf{A} \): State transition matrix.
- \( \mathbf{B} \): Control input matrix.
- \( \mathbf{u}_k \): Control input.
- \( \mathbf{y}_k \): Measurement vector.
- \( \mathbf{C} \): Measurement matrix.
- \( \mathbf{w}_k \): Process noise (\( \mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q}) \)).
- \( \mathbf{v}_k \): Measurement noise (\( \mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R}) \)).

---

## Kalman Filter Steps

### 1. Prediction
Predict the state and covariance for the next time step:
\[
\hat{\mathbf{x}}_k^- = \mathbf{A} \hat{\mathbf{x}}_{k-1} + \mathbf{B} \mathbf{u}_k,
\]
\[
\mathbf{P}_k^- = \mathbf{A} \mathbf{P}_{k-1} \mathbf{A}^\top + \mathbf{Q}.
\]

### 2. Update
Refine the state estimate and covariance using the measurement \( \mathbf{y}_k \):
\[
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{C}^\top \left( \mathbf{C} \mathbf{P}_k^- \mathbf{C}^\top + \mathbf{R} \right)^{-1},
\]
\[
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \left( \mathbf{y}_k - \mathbf{C} \hat{\mathbf{x}}_k^- \right),
\]
\[
\mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{C} \right) \mathbf{P}_k^-.
\]

---

## Extended Kalman Filter (EKF) for Nonlinear Systems

For systems with nonlinear dynamics, the EKF approximates the system by linearizing it around the current state estimate.

### Nonlinear Model
\[
\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k,
\]
\[
\mathbf{y}_k = h(\mathbf{x}_k) + \mathbf{v}_k.
\]

### Steps in EKF
1. **Prediction**:
   \[
   \hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_k),
   \]
   \[
   \mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^\top + \mathbf{Q}.
   \]

2. **Update**:
   \[
   \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^\top \left( \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^\top + \mathbf{R} \right)^{-1},
   \]
   \[
   \hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \left( \mathbf{y}_k - h(\hat{\mathbf{x}}_k^-) \right),
   \]
   \[
   \mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{H}_k \right) \mathbf{P}_k^-.
   \]

### Jacobian Matrices
The system and measurement functions are linearized using Jacobians:
\[
\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}}, \quad \mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}}.
\]

---

## Advantages and Limitations

### Advantages
- Works for nonlinear systems (EKF).
- Provides an optimal estimate in noisy environments.

### Limitations
- EKF linearization may not handle highly nonlinear systems well.
- Computational cost due to Jacobian calculations.

---

## References
1. [Understanding Kalman Filters](https://www.mathworks.com/videos/series/understanding-kalman-filters.html)
2. [Statistical signal processing: estimation theory](http://lib.ysu.am/disciplines_bk/0c6460162880d19be573a6df4c75db33.pdf)

---

## License
This project is licensed under the MIT License.

---

### Contributors
- Shayan Zargari

