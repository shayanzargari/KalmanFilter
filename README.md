# KalmanFilter

A beginner-friendly tutorial on the Kalman Filter, explaining its principles and mathematical foundations. Perfect for those looking to understand state estimation in dynamic systems with noise. Includes theoretical explanations and code examples for practical applications.

---

## Introduction to Kalman Filters

The Kalman Filter is an optimal recursive estimation algorithm used to estimate the states of a linear dynamical system in the presence of noise. It combines predictions and noisy measurements to provide optimal state estimates, making it invaluable in fields such as:

- **Control Systems**
- **Navigation**
- **Signal Processing**
- **Robotics**

### Core Steps

1. **Prediction**: Estimate the next state and its uncertainty.
2. **Correction (Update)**: Incorporate new measurements to refine the estimate.

---

## System Model

A linear dynamical system is described as:

- **State update equation**:  
  $$ \mathbf{x}_k = \mathbf{A} \mathbf{x}_{k-1} + \mathbf{B} \mathbf{u}_k + \mathbf{w}_k $$

- **Measurement equation**:  
  $$ \mathbf{y}_k = \mathbf{C} \mathbf{x}_k + \mathbf{v}_k $$

Where:
- \( \mathbf{x}_k \): State vector at time \( k \).
- \( \mathbf{A} \): State transition matrix.
- \( \mathbf{B} \): Control input matrix.
- \( \mathbf{u}_k \): Control input vector.
- \( \mathbf{y}_k \): Measurement vector.
- \( \mathbf{C} \): Measurement matrix.
- \( \mathbf{w}_k \): Process noise, \( \mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q}) \).
- \( \mathbf{v}_k \): Measurement noise, \( \mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R}) \).

---

## Kalman Filter Steps

### 1. Prediction

Predict the state and covariance for the next time step:

- Predicted state:  
  $$ \mathbf{x}_k^- = \mathbf{A} \mathbf{x}_{k-1} + \mathbf{B} \mathbf{u}_k $$

- Predicted error covariance:  
  $$ \mathbf{P}_k^- = \mathbf{A} \mathbf{P}_{k-1} \mathbf{A}^\top + \mathbf{Q} $$

### 2. Update

Refine the state estimate and covariance using the measurement:

- Kalman Gain:  
  $$ \mathbf{K}_k = \mathbf{P}_k^- \mathbf{C}^\top \left( \mathbf{C} \mathbf{P}_k^- \mathbf{C}^\top + \mathbf{R} \right)^{-1} $$

- Updated state:  
  $$ \mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k \left( \mathbf{y}_k - \mathbf{C} \mathbf{x}_k^- \right) $$

- Updated error covariance:  
  $$ \mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{C} \right) \mathbf{P}_k^- $$

---

## Extended Kalman Filter (EKF)

The EKF extends the Kalman Filter to handle nonlinear systems by linearizing the state transition and measurement equations around the current estimate.

### Nonlinear System Model

- **State update equation**:  
  $$ \mathbf{x}_k = \mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k $$

- **Measurement equation**:  
  $$ \mathbf{y}_k = \mathbf{h}(\mathbf{x}_k) + \mathbf{v}_k $$

Where:
- \( \mathbf{f} \): Nonlinear state transition function.
- \( \mathbf{h} \): Nonlinear measurement function.

### Steps in EKF

1. **Prediction**:
   - Predicted state:  
     $$ \mathbf{x}_k^- = \mathbf{f}(\mathbf{x}_{k-1}, \mathbf{u}_k) $$
   - Predicted error covariance:  
     $$ \mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^\top + \mathbf{Q} $$
   - \( \mathbf{F}_k \): Jacobian of the state transition function \( \mathbf{f} \).

2. **Update**:
   - Kalman Gain:  
     $$ \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^\top \left( \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^\top + \mathbf{R} \right)^{-1} $$
   - Updated state:  
     $$ \mathbf{x}_k = \mathbf{x}_k^- + \mathbf{K}_k \left( \mathbf{y}_k - \mathbf{h}(\mathbf{x}_k^-) \right) $$
   - Updated error covariance:  
     $$ \mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{H}_k \right) \mathbf{P}_k^- $$
   - \( \mathbf{H}_k \): Jacobian of the measurement function \( \mathbf{h} \).

---

## Advantages and Limitations

### Advantages
- Handles both linear (KF) and nonlinear (EKF) systems.
- Provides optimal estimates in noisy environments.

### Limitations
- Linearization in EKF may fail for highly nonlinear systems.
- Computationally expensive due to Jacobian calculations.

---

## References

1. [Understanding Kalman Filters - MathWorks](https://www.mathworks.com/videos/series/understanding-kalman-filters.html)
2. [Statistical Signal Processing: Estimation Theory](http://lib.ysu.am/disciplines_bk/0c6460162880d19be573a6df4c75db33.pdf)

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contributors

- **Shayan Zargari**
