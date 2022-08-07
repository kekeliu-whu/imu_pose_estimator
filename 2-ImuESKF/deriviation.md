# Deriviation on imu_eskf
## imu_eskf
This demo estimates imu orientation and gyro bias by ESKF.


## IMU Noise Model
$$
\hat{w} = w+b+n_w \\
\hat{a} = R_w^t g^w + n_a
$$
In the equation above, acceleration has been ignored (or considered as a part of $n_{a_t}$) because acceleration is impossible to determine without odometry. Acceleration bias are ignored for acc-bias component is too small.  
$R_w^t$ and $b$ are varibles to be estimated.


## Prediction
From eskf, we get state of true/nominal/error denoted as $x_t=x+\delta{x}$.
### True state
$$
\begin{equation}
\begin{split}
\dot{q}_t &= q_t \otimes \begin{bmatrix} 1\\ \frac12(w_m-b_t-n_w) \end{bmatrix} \\
\dot{b}_t &= n_b
\end{split}
\end{equation}
$$

### Nominal state
$$
\begin{equation}
\begin{split}
\dot{q} &= q \otimes \begin{bmatrix} 1\\ \frac12(w_m-b) \end{bmatrix} \\
\dot{b} &= 0
\end{split}
\end{equation}
$$

### Error state
$$
\begin{equation}
\begin{split}
\dot{\delta\theta} &= -(w_m-b)^\wedge \delta\theta - \delta{b} - n_w \\
\dot{\delta b} &= n_b
\end{split}
\end{equation}
$$
