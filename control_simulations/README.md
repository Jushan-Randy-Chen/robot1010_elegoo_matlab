# Control simulations

Simulations of feedback control for the car. They do not need the car.

| File | What it does |
|---|---|
| `sim_distance_control.mlx` | Live Script: P-control of the distance to a wall, with sensor noise and a variable sampling period. The text explains the model and the control law. |
| `sim_distance_control_script.m` | The same simulation as a plain script. It has a different name because a `.mlx` file hides an `.m` file with the same name. |

The model is `x(k+1) = x(k) - u(k) t_s(k)`, the measurement is `y(k) = x(k) + n_s(k)`, and the control law is `u(k) = k_p (y(k) - x*)`. Change `k_p`, `t_s_star` and the noise bounds to see the effect on the steady-state error and on stability.
