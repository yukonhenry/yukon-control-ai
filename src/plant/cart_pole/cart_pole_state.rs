use std::ops::{Add, Mul};
use crate::plant::cart_pole::cart_pole_params::CartPoleParams;
use crate::plant::kinodynamic_base::KinodynamicBase;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CartPoleState {
    pub x: f64,
    pub x_dot: f64,
    pub theta: f64,
    pub theta_dot: f64,
}

/// The 1-DOF Control Input Vector: [f], the horizontal force applied to the cart
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CartPoleControl {
    pub force: f64, // The horizontal force applied to the cart
}

// Implement Addition for state mixing inside RK4 engine
impl Add for CartPoleState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            x_dot: self.x_dot + other.x_dot,
            theta: self.theta + other.theta,
            theta_dot: self.theta_dot + other.theta_dot,
        }
    }
}

// Implement Scalar Multiplication for integration scaling steps
impl Mul<f64> for CartPoleState {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            x_dot: self.x_dot * scalar,
            theta: self.theta * scalar,
            theta_dot: self.theta_dot * scalar,
        }
    }
}

/// Implementation of the Kinodynamic Model trait for our cart-pole parameters
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SecondOrderCartPole {
    pub params: CartPoleParams,
}

impl KinodynamicBase for SecondOrderCartPole {
    type State = CartPoleState;
    type Control = CartPoleControl;

    /// Reference https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
    /// Equations 56 and 57
    ///
    /// `theta` is zero with the pole straight up and positive when it leans to the right
    /// (+x) of the pivot.
    fn dynamics(&self, state: &Self::State, control: &Self::Control) -> Self::State {
        let CartPoleParams {
            mass_pole: m,
            length: l,
            gravity: g,
            friction_cart: mu_c,
            friction_pole: mu_p,
            ..
        } = self.params;
        let total_mass = self.params.total_mass();
        let (sin_theta, cos_theta) = state.theta.sin_cos();

        // Equation 56
        let x_ddot = (m * g * sin_theta * cos_theta
            - (7.0 / 3.0)
                * (control.force + m * l * state.theta_dot.powi(2) * sin_theta
                    - mu_c * state.x_dot)
            - mu_p * state.theta_dot * cos_theta / l)
            / (m * cos_theta.powi(2) - (7.0 / 3.0) * total_mass);

        // Equation 57
        let theta_ddot = 3.0 / (7.0 * l)
            * (g * sin_theta - x_ddot * cos_theta - mu_p * state.theta_dot / (m * l));

        Self::State {
            x: state.x_dot,
            x_dot: x_ddot,
            theta: state.theta_dot,
            theta_dot: theta_ddot,
        }
    }

    /// Enforces the actuator saturation limits (Control Box Constraints)
    fn clamp_inputs(&self, control: &Self::Control) -> Self::Control {
        Self::Control {
            force: self.params.saturate_force(control.force),
        }
    }

    /// The cart-pole has no state box constraints. `theta` is deliberately left unwrapped so
    /// the state stays continuous and keeps track of full revolutions; consumers that need a
    /// single representation per pose (rewards, observations, distances) should use
    /// `wrap_angle` or `sin`/`cos` of `theta` themselves.
    fn clamp_states(&self, state: &Self::State) -> Self::State {
        *state
    }
}

impl SecondOrderCartPole {
    /// Inverts Equation 56: the cart force that produces cart acceleration `x_ddot` from
    /// `state` (collocated partial feedback linearization). Unsaturated.
    pub fn force_for_cart_acceleration(&self, state: &CartPoleState, x_ddot: f64) -> f64 {
        let CartPoleParams {
            mass_pole: m,
            length: l,
            gravity: g,
            friction_cart: mu_c,
            friction_pole: mu_p,
            ..
        } = self.params;
        let (sin_theta, cos_theta) = state.theta.sin_cos();
        let denominator = m * cos_theta.powi(2) - (7.0 / 3.0) * self.params.total_mass();

        (3.0 / 7.0)
            * (m * g * sin_theta * cos_theta
                - mu_p * state.theta_dot * cos_theta / l
                - x_ddot * denominator)
            - m * l * state.theta_dot.powi(2) * sin_theta
            + mu_c * state.x_dot
    }

    /// Mechanical energy of the pole relative to the cart, zero when upright and at rest and
    /// `-2 m g l` when hanging at rest. Equation 57 gives the pole an effective inertia of
    /// `(7/3) m l^2` about the pivot, so this is the quantity that model conserves on a
    /// stationary cart (friction aside); its rate of change is `-m l x_ddot theta_dot cos(theta)`.
    pub fn pole_energy(&self, state: &CartPoleState) -> f64 {
        let CartPoleParams { mass_pole: m, length: l, gravity: g, .. } = self.params;
        0.5 * (7.0 / 3.0) * m * l * l * state.theta_dot.powi(2) + m * g * l * (state.theta.cos() - 1.0)
    }
}

/// Wraps an angle into [-pi, pi). `rem_euclid` rather than `%`, since Rust's float `%` takes
/// the sign of the dividend and would leave negative angles unwrapped.
pub fn wrap_angle(theta: f64) -> f64 {
    use std::f64::consts::PI;
    (theta + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::integrator::Integrator;

    fn cart_pole() -> SecondOrderCartPole {
        SecondOrderCartPole::default()
    }

    fn at_rest(theta: f64) -> CartPoleState {
        CartPoleState {
            x: 0.0,
            x_dot: 0.0,
            theta,
            theta_dot: 0.0,
        }
    }

    const NO_FORCE: CartPoleControl = CartPoleControl { force: 0.0 };

    /// Upright and motionless with no force applied is an (unstable) equilibrium.
    #[test]
    fn upright_at_rest_is_an_equilibrium() {
        let d = cart_pole().dynamics(&at_rest(0.0), &NO_FORCE);
        assert_eq!(d, CartPoleState { x: 0.0, x_dot: 0.0, theta: 0.0, theta_dot: 0.0 });
    }

    /// Pushing the cart right accelerates it right and tips the upright pole back to the left.
    #[test]
    fn pushing_right_moves_the_cart_right_and_tips_the_pole_left() {
        let d = cart_pole().dynamics(&at_rest(0.0), &CartPoleControl { force: 5.0 });
        assert!(d.x_dot > 0.0, "cart acceleration {}", d.x_dot);
        assert!(d.theta_dot < 0.0, "pole angular acceleration {}", d.theta_dot);
    }

    /// A pole leaning right falls further right, and its reaction pushes the cart left.
    /// Getting the sign of either equation wrong turns a falling pole into a self-righting one.
    #[test]
    fn a_leaning_pole_falls_the_way_it_leans() {
        for theta in [0.1, -0.1] {
            let d = cart_pole().dynamics(&at_rest(theta), &NO_FORCE);
            assert!(d.theta_dot * theta > 0.0, "theta {theta} gave theta_ddot {}", d.theta_dot);
            assert!(d.x_dot * theta < 0.0, "theta {theta} gave x_ddot {}", d.x_dot);
        }
    }

    /// Hanging straight down is a stable equilibrium: displaced slightly, the pole swings back.
    #[test]
    fn a_hanging_pole_swings_back_toward_the_bottom() {
        let pi = std::f64::consts::PI;
        let d = cart_pole().dynamics(&at_rest(pi - 0.1), &NO_FORCE);
        assert!(d.theta_dot > 0.0, "pole should accelerate back toward pi, got {}", d.theta_dot);
    }

    /// Friction opposes motion: a cart coasting right with the pole hanging still decelerates.
    #[test]
    fn cart_friction_opposes_velocity() {
        let state = CartPoleState { x_dot: 1.0, ..at_rest(0.0) };
        let d = cart_pole().dynamics(&state, &NO_FORCE);
        assert!(d.x_dot < 0.0, "a coasting cart should slow down, got {}", d.x_dot);
    }

    /// The force from `force_for_cart_acceleration` reproduces the requested acceleration.
    #[test]
    fn force_for_cart_acceleration_inverts_the_dynamics() {
        let model = cart_pole();
        let state = CartPoleState { x: 0.3, x_dot: -1.2, theta: 2.0, theta_dot: 3.5 };
        for x_ddot in [-4.0, 0.0, 2.5] {
            let force = model.force_for_cart_acceleration(&state, x_ddot);
            let d = model.dynamics(&state, &CartPoleControl { force });
            assert!((d.x_dot - x_ddot).abs() < 1e-9, "asked {x_ddot}, got {}", d.x_dot);
        }
    }

    /// Without friction or force, a swinging pole on a free cart conserves `pole_energy` only
    /// if the cart doesn't accelerate, so pin the cart with the inverse dynamics and check.
    #[test]
    fn pole_energy_is_conserved_on_a_stationary_cart() {
        let model = SecondOrderCartPole {
            params: CartPoleParams { friction_cart: 0.0, friction_pole: 0.0, ..Default::default() },
        };
        let mut state = at_rest(2.5);
        let initial = model.pole_energy(&state);
        for _ in 0..500 {
            let force = model.force_for_cart_acceleration(&state, 0.0);
            // Force is held across the RK4 step, so it's only exactly right at its start.
            let model = SecondOrderCartPole {
                params: CartPoleParams { max_force: 1e9, min_force: -1e9, ..model.params },
            };
            state = Integrator::step(&model, &state, &CartPoleControl { force }, 0.001);
        }
        let drift = (model.pole_energy(&state) - initial).abs();
        assert!(drift < 1e-3, "energy drifted by {drift}");
        assert!((model.pole_energy(&at_rest(std::f64::consts::PI)) + 2.0 * 0.1 * 9.8 * 0.5).abs() < 1e-12);
    }

    #[test]
    fn force_is_saturated_at_the_actuator_limits() {
        let model = cart_pole();
        assert_eq!(model.clamp_inputs(&CartPoleControl { force: 50.0 }).force, 10.0);
        assert_eq!(model.clamp_inputs(&CartPoleControl { force: -50.0 }).force, -10.0);
        assert_eq!(model.clamp_inputs(&CartPoleControl { force: 3.0 }).force, 3.0);
    }

    #[test]
    fn wrap_angle_folds_into_a_half_open_turn_from_either_side() {
        let pi = std::f64::consts::PI;
        let wrapped = wrap_angle;

        for (raw, expected) in [
            (0.5, 0.5),
            (-0.5, -0.5),
            (3.5, 3.5 - 2.0 * pi),
            (-3.5, 2.0 * pi - 3.5),
            (7.0, 7.0 - 2.0 * pi),
        ] {
            assert!((wrapped(raw) - expected).abs() < 1e-12, "{raw} wrapped to {}", wrapped(raw));
        }
        assert!((wrapped(pi) + pi).abs() < 1e-12, "pi should fold onto -pi");
    }

    /// A spinning pole's angle keeps counting revolutions rather than jumping back by 2*pi as
    /// it passes the bottom.
    #[test]
    fn a_spinning_pole_keeps_a_continuous_raw_angle() {
        let model = cart_pole();
        let mut state = CartPoleState { theta_dot: 15.0, ..at_rest(0.0) };
        for _ in 0..100 {
            let next = Integrator::step(&model, &state, &NO_FORCE, 0.01);
            assert!(
                (next.theta - state.theta).abs() < 0.5,
                "theta jumped from {} to {}",
                state.theta,
                next.theta,
            );
            state = next;
        }
        assert!(
            state.theta > 2.0 * std::f64::consts::PI,
            "pole should have completed a revolution, theta = {}",
            state.theta,
        );
    }

    /// An unbalanced pole falls over within a couple of seconds of simulated time.
    #[test]
    fn an_uncontrolled_pole_falls_over() {
        let model = cart_pole();
        let mut state = at_rest(0.01);
        for _ in 0..200 {
            state = Integrator::step(&model, &state, &NO_FORCE, 0.01);
        }
        assert!(state.theta.abs() > 0.5, "pole only reached {} rad after 2 s", state.theta);
        assert!(state.theta.is_finite() && state.x.is_finite());
    }
}
