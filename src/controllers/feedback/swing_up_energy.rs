use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwingUpEnergyParams {
    pub k: f64,              // Gain for the energy difference
    pub desired_energy: f64, // Target `pole_energy` (0 = upright at rest)
    pub max_accel: f64,      // Saturation on the commanded cart acceleration, m/s^2
    pub k_x: f64,            // Cart centering gain on position
    pub k_x_dot: f64,        // Cart centering gain on velocity
    pub kick_accel: f64,     // Acceleration used to start a pole resting at the bottom
}

impl Default for SwingUpEnergyParams {
    fn default() -> Self {
        Self {
            k: 20.0,
            desired_energy: 0.0,
            max_accel: 6.0,
            k_x: 1.0,
            k_x_dot: 1.5,
            kick_accel: 3.0,
        }
    }
}

/// Astrom-Furuta energy-pumping swing-up. Commands the cart acceleration
/// `x_ddot = sat(k (E - E_d) theta_dot cos(theta)) - k_x x - k_x_dot x_dot`, which makes
/// `dE/dt = -m l k (E - E_d) (theta_dot cos(theta))^2` (ignoring the centering terms and
/// saturation), and converts it to a force through the inverse cart dynamics.
pub struct SwingUpEnergyController {
    pub params: SwingUpEnergyParams,
    pub model: SecondOrderCartPole,
    pub control_output: f64,
}

impl SwingUpEnergyController {
    pub fn new(params: SwingUpEnergyParams, model: SecondOrderCartPole) -> Self {
        Self { params, model, control_output: 0.0 }
    }

    pub fn energy_error(&self, state: &CartPoleState) -> f64 {
        self.model.pole_energy(state) - self.params.desired_energy
    }

    fn cart_acceleration(&self, state: &CartPoleState) -> f64 {
        let p = &self.params;
        let e_tilde = self.energy_error(state);
        let pump = e_tilde * state.theta_dot * state.theta.cos();

        // The pumping term vanishes for a pole at rest at the bottom, which would never start.
        let resting_at_bottom = state.theta.cos() < -0.99 && state.theta_dot.abs() < 1e-3;
        let energy = if resting_at_bottom && e_tilde < 0.0 {
            p.kick_accel
        } else {
            (p.k * pump).clamp(-p.max_accel, p.max_accel)
        };
        energy - p.k_x * state.x - p.k_x_dot * state.x_dot
    }
}

impl ControlOutput for SwingUpEnergyController {
    type State = CartPoleState;
    type Control = CartPoleControl;

    fn compute_control(&mut self, state: &Self::State) -> Self::Control {
        let force = self.model.force_for_cart_acceleration(state, self.cart_acceleration(state));
        self.control_output = self.model.params.saturate_force(force);
        CartPoleControl { force: self.control_output }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::integrator::Integrator;

    /// Pumping alone, starting from rest at the bottom, brings the pole's energy up to the
    /// upright level.
    #[test]
    fn pumps_a_hanging_pole_up_to_the_upright_energy() {
        let model = SecondOrderCartPole::default();
        let mut controller = SwingUpEnergyController::new(SwingUpEnergyParams::default(), model);
        let mut state = CartPoleState { x: 0.0, x_dot: 0.0, theta: std::f64::consts::PI, theta_dot: 0.0 };
        let start = controller.energy_error(&state).abs();

        let mut best = f64::INFINITY;
        for _ in 0..1000 {
            let control = controller.compute_control(&state);
            state = Integrator::step(&model, &state, &control, 0.01);
            best = best.min(controller.energy_error(&state).abs());
        }
        assert!(best < 0.05 * start, "energy error only fell from {start} to {best}");
        assert!(state.x.abs() < 2.0, "cart wandered to {}", state.x);
    }
}
