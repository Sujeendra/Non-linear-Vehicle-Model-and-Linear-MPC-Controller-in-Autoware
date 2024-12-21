# Non-linear Vehicle Model and Linear MPC Controller in Autoware

## Abstract

This report presents the development and analysis of a non-linear vehicle model and a Model Predictive Control (MPC) framework implemented within the **Autoware** autonomous driving platform.

## Dynamic Modeling

### Slip Angles

The slip angles for the front and rear tires are defined as:

$$
\alpha_f = \tan^{-1} \left( \frac{v_y + \ell_f r}{v_x} \right) - \delta, \quad
\alpha_r = \tan^{-1} \left( \frac{v_y - \ell_r r}{v_x} \right).
$$

### Equations of Motion

The lateral force balance is expressed as:

$$
F_{yf} \cos(\delta) - F_{xf} \sin(\delta) + F_{yr} = m (\dot{v}_y + v_x r).
$$

The yaw moment balance is given by:

$$
\ell_f (F_{yf} \cos(\delta)) - \ell_r (F_{yr} - F_{xf} \sin(\delta)) = I_z \dot{r}.
$$

### Dynamic Model

Below is the dynamic model for the vehicle:

![Dynamic Model](dynamic_model.png)

This model is used to describe the behavior of the vehicle in response to lateral forces and moments.

### Path Coordinates

The dynamic model in path coordinates is shown below:

![Path Coordinates](path-coordinates.png)

This representation helps in analyzing the vehicle's motion along a predefined trajectory.

## Conclusion

This report demonstrates the importance of accurate dynamic modeling and the interplay between non-linear and linearized models in achieving robust autonomous vehicle control.
