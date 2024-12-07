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
  `x_k = A * x_(k-1) + B * u_k + w_k`

- **Measurement equation**:  
  `y_k = C * x_k + v_k`

Where:
- `x_k`: State vector at time `k`.
- `A`: State transition matrix.
- `B`: Control input matrix.
- `u_k`: Control input.
- `y_k`: Measurement vector.
- `C`: Measurement matrix.
- `w_k`: Process noise, `w_k ~ N(0, Q)`.
- `v_k`: Measurement noise, `v_k ~ N(0, R)`.

---

## Kalman Filter Steps

### 1. Prediction

Predict the state and covariance for the next time step:

- Predicted state:  
  `x_k^- = A * x_(k-1) + B * u_k`

- Predicted error covariance:  
  `P_k^- = A * P_(k-1) * A^T + Q`

### 2. Update

Refine the state estimate and covariance using the measurement:

- Kalman Gain:  
  `K_k = P_k^- * C^T * (C * P_k^- * C^T + R)^-1`

- Updated state:  
  `x_k = x_k^- + K_k * (y_k - C * x_k^-)`

- Updated error covariance:  
  `P_k = (I - K_k * C) * P_k^-`

---

## Extended Kalman Filter (EKF)

The EKF extends the Kalman Filter to handle nonlinear systems by linearizing the state transition and measurement equations around the current estimate.

### Nonlinear System Model

- **State update equation**:  
  `x_k = f(x_(k-1), u_k) + w_k`

- **Measurement equation**:  
  `y_k = h(x_k) + v_k`

Where:
- `f`: Nonlinear state transition function.
- `h`: Nonlinear measurement function.

### Steps in EKF

1. **Prediction**:
   - Predicted state:  
     `x_k^- = f(x_(k-1), u_k)`

   - Predicted error covariance:  
     `P_k^- = F_k * P_(k-1) * F_k^T + Q`

   - `F_k`: Jacobian of the state transition function `f`.

2. **Update**:
   - Kalman Gain:  
     `K_k = P_k^- * H_k^T * (H_k * P_k^- * H_k^T + R)^-1`

   - Updated state:  
     `x_k = x_k^- + K_k * (y_k - h(x_k^-))`

   - Updated error covariance:  
     `P_k = (I - K_k * H_k) * P_k^-`

   - `H_k`: Jacobian of the measurement function `h`.

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
