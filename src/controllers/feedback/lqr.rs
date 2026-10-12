use nalgebra::{Matrix1, Matrix4, Matrix4x1, SMatrix, Vector4};

use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::{
    wrap_angle, CartPoleControl, CartPoleState, SecondOrderCartPole,
};
use crate::plant::kinodynamic_base::KinodynamicBase;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LQRParams {
    pub k: [f64; 4], // LQR gain vector over [x, x_dot, theta, theta_dot]; force = -k . state
}

impl LQRParams {
    /// Designs a discrete-time LQR about the upright equilibrium for a controller that holds
    /// its force for `dt` seconds. `q` is the diagonal state weight over
    /// [x, x_dot, theta, theta_dot] and `r` the force weight.
    pub fn design(model: &SecondOrderCartPole, q: [f64; 4], r: f64, dt: f64) -> Self {
        let (a, b) = linearize_upright(model);
        let (a_d, b_d) = discretize(&a, &b, dt);
        Self { k: solve_dare_gain(&a_d, &b_d, &Matrix4::from_diagonal(&Vector4::from(q)), r) }
    }
}

impl Default for LQRParams {
    fn default() -> Self {
        Self::design(&SecondOrderCartPole::default(), [1.0, 1.0, 10.0, 1.0], 0.1, 0.01)
    }
}

pub struct LQRController {
    pub params: LQRParams,
    pub model: SecondOrderCartPole,
    pub control_output: f64,
}

impl LQRController {
    pub fn new(params: LQRParams, model: SecondOrderCartPole) -> Self {
        Self { params, model, control_output: 0.0 }
    }

    /// The state relative to the upright equilibrium, with the pole angle folded so that
    /// every full revolution maps back onto the same target.
    pub fn error(state: &CartPoleState) -> Vector4<f64> {
        Vector4::new(state.x, state.x_dot, wrap_angle(state.theta), state.theta_dot)
    }
}

impl ControlOutput for LQRController {
    type State = CartPoleState;
    type Control = CartPoleControl;

    fn compute_control(&mut self, state: &Self::State) -> Self::Control {
        let force = -Vector4::from(self.params.k).dot(&Self::error(state));
        self.control_output = self.model.params.saturate_force(force);
        CartPoleControl { force: self.control_output }
    }
}

/// Central-difference Jacobians of the dynamics at upright, at rest, with no force.
fn linearize_upright(model: &SecondOrderCartPole) -> (Matrix4<f64>, Matrix4x1<f64>) {
    let as_vector = |s: CartPoleState| Vector4::new(s.x, s.x_dot, s.theta, s.theta_dot);
    let f = |x: Vector4<f64>, force: f64| {
        let state = CartPoleState { x: x[0], x_dot: x[1], theta: x[2], theta_dot: x[3] };
        as_vector(model.dynamics(&state, &CartPoleControl { force }))
    };
    let h = 1e-6;

    let mut a = Matrix4::zeros();
    for i in 0..4 {
        let mut step = Vector4::zeros();
        step[i] = h;
        a.set_column(i, &((f(step, 0.0) - f(-step, 0.0)) / (2.0 * h)));
    }
    let b = (f(Vector4::zeros(), h) - f(Vector4::zeros(), -h)) / (2.0 * h);
    (a, b)
}

/// Zero-order-hold discretization via the exponential of the augmented matrix [[A, B], [0, 0]].
fn discretize(a: &Matrix4<f64>, b: &Matrix4x1<f64>, dt: f64) -> (Matrix4<f64>, Matrix4x1<f64>) {
    let mut augmented = SMatrix::<f64, 5, 5>::zeros();
    augmented.fixed_view_mut::<4, 4>(0, 0).copy_from(&(a * dt));
    augmented.fixed_view_mut::<4, 1>(0, 4).copy_from(&(b * dt));
    let phi = augmented.exp();
    (phi.fixed_view::<4, 4>(0, 0).into_owned(), phi.fixed_view::<4, 1>(0, 4).into_owned())
}

/// Iterates the discrete algebraic Riccati equation to its fixed point and returns the gain
/// `K = (R + B'PB)^-1 B'PA`.
fn solve_dare_gain(a: &Matrix4<f64>, b: &Matrix4x1<f64>, q: &Matrix4<f64>, r: f64) -> [f64; 4] {
    let gain = |p: &Matrix4<f64>| {
        let s = Matrix1::new(r) + b.transpose() * p * b;
        (b.transpose() * p * a) / s[0]
    };

    let mut p = *q;
    for _ in 0..100_000 {
        let k = gain(&p);
        let next = q + a.transpose() * p * a - a.transpose() * p * b * k;
        let converged = (next - p).norm() <= 1e-10 * next.norm();
        p = next;
        if converged {
            let k = gain(&p);
            return [k[0], k[1], k[2], k[3]];
        }
    }
    panic!("discrete Riccati iteration did not converge");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::integrator::Integrator;

    /// Every eigenvalue of the sampled closed loop A_d - B_d K lies inside the unit circle.
    #[test]
    fn designed_gains_stabilize_the_linearized_plant() {
        let model = SecondOrderCartPole::default();
        let params = LQRParams::default();
        let (a, b) = linearize_upright(&model);
        let (a_d, b_d) = discretize(&a, &b, 0.01);
        let closed_loop = a_d - b_d * Vector4::from(params.k).transpose();
        for eigenvalue in closed_loop.complex_eigenvalues().iter() {
            assert!(eigenvalue.norm() < 1.0, "unstable closed-loop eigenvalue {eigenvalue}");
        }
    }

    /// Pushing right tips the pole left, so a pole leaning right needs a push right.
    #[test]
    fn a_pole_leaning_right_is_caught_by_pushing_right() {
        let mut controller = LQRController::new(LQRParams::default(), SecondOrderCartPole::default());
        let state = CartPoleState { x: 0.0, x_dot: 0.0, theta: 0.05, theta_dot: 0.0 };
        assert!(controller.compute_control(&state).force > 0.0);
    }

    /// The nonlinear plant, started off balance and a full revolution away in raw angle,
    /// settles back to upright over the origin.
    #[test]
    fn balances_the_nonlinear_plant() {
        let model = SecondOrderCartPole::default();
        let mut controller = LQRController::new(LQRParams::default(), model);
        let mut state = CartPoleState {
            x: 0.5,
            x_dot: 0.0,
            theta: 2.0 * std::f64::consts::PI + 0.2,
            theta_dot: -0.3,
        };
        for _ in 0..1000 {
            let control = controller.compute_control(&state);
            state = Integrator::step(&model, &state, &control, 0.01);
        }
        let error = LQRController::error(&state);
        assert!(error.norm() < 1e-3, "did not settle, error {error}");
    }
}
