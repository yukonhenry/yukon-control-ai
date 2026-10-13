use crate::controllers::control_output::ControlOutput;
use crate::controllers::feedback::lqr::{LQRController, LQRParams};
use crate::controllers::feedback::swing_up_energy::{SwingUpEnergyController, SwingUpEnergyParams};
use crate::plant::cart_pole::cart_pole_state::{
    wrap_angle, CartPoleControl, CartPoleState, SecondOrderCartPole,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwitchMode {
    SwingUp,
    Balance,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwitchParams {
    /// Hand over to LQR once |wrapped theta| and |theta_dot| are both inside these bounds.
    pub engage_angle: f64,
    pub engage_rate: f64,
    /// Fall back to swing-up if |wrapped theta| leaves this (wider) bound while balancing.
    pub release_angle: f64,
}

impl Default for SwitchParams {
    fn default() -> Self {
        Self { engage_angle: 0.35, engage_rate: 3.0, release_angle: 0.8 }
    }
}

/// Swings the pole up with energy pumping, then balances it with LQR once it enters a
/// neighborhood of upright. The gap between `engage_angle` and `release_angle` is hysteresis,
/// so the mode doesn't chatter on the boundary.
pub struct SwingUpEnergySwitchLQR {
    pub swing_up: SwingUpEnergyController,
    pub lqr: LQRController,
    pub switch: SwitchParams,
    pub mode: SwitchMode,
    pub control_output: f64,
}

impl SwingUpEnergySwitchLQR {
    pub fn new(
        swing_up: SwingUpEnergyParams,
        lqr: LQRParams,
        switch: SwitchParams,
        model: SecondOrderCartPole,
    ) -> Self {
        Self {
            swing_up: SwingUpEnergyController::new(swing_up, model),
            lqr: LQRController::new(lqr, model),
            switch,
            mode: SwitchMode::SwingUp,
            control_output: 0.0,
        }
    }

    fn update_mode(&mut self, state: &CartPoleState) {
        let angle = wrap_angle(state.theta).abs();
        self.mode = match self.mode {
            SwitchMode::SwingUp
                if angle < self.switch.engage_angle
                    && state.theta_dot.abs() < self.switch.engage_rate =>
            {
                SwitchMode::Balance
            }
            SwitchMode::Balance if angle > self.switch.release_angle => SwitchMode::SwingUp,
            mode => mode,
        };
    }
}

impl ControlOutput for SwingUpEnergySwitchLQR {
    type State = CartPoleState;
    type Control = CartPoleControl;

    fn compute_control(&mut self, state: &Self::State) -> Self::Control {
        self.update_mode(state);
        let control = match self.mode {
            SwitchMode::SwingUp => self.swing_up.compute_control(state),
            SwitchMode::Balance => self.lqr.compute_control(state),
        };
        self.control_output = control.force;
        control
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulator::integrator::Integrator;

    fn controller(model: SecondOrderCartPole) -> SwingUpEnergySwitchLQR {
        SwingUpEnergySwitchLQR::new(
            SwingUpEnergyParams::default(),
            LQRParams::design(&model, [1.0, 1.0, 10.0, 1.0], 0.1, 0.01),
            SwitchParams::default(),
            model,
        )
    }

    /// From hanging (at rest, and with a few perturbations), the pole is swung up and ends up
    /// balanced over the origin under LQR.
    #[test]
    fn swings_up_from_hanging_and_balances() {
        let model = SecondOrderCartPole::default();
        let pi = std::f64::consts::PI;
        for (theta, theta_dot, x) in [(pi, 0.0, 0.0), (pi - 0.3, 0.0, 0.0), (pi, -1.0, 0.5), (-pi + 0.1, 0.5, -0.5)] {
            let mut controller = controller(model);
            let mut state = CartPoleState { x, x_dot: 0.0, theta, theta_dot };
            let mut max_x: f64 = 0.0;
            for _ in 0..2000 {
                let control = controller.compute_control(&state);
                state = Integrator::step(&model, &state, &control, 0.01);
                max_x = max_x.max(state.x.abs());
            }
            assert_eq!(controller.mode, SwitchMode::Balance, "start ({theta}, {theta_dot}, {x})");
            let error = LQRController::error(&state);
            assert!(error.norm() < 1e-2, "start ({theta}, {theta_dot}, {x}) ended with error {error}");
            assert!(max_x < 2.5, "start ({theta}, {theta_dot}, {x}) let the cart reach {max_x}");
        }
    }
}
