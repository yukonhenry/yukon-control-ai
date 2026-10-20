use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};
use crate::simulator::integrator::Integrator;

/// A cart-pole plant that a closed loop can drive: a stateful simulation that is reset to a
/// starting state, read, and advanced with a control held for each step.
pub trait CartPoleSim {
    fn reset(&mut self, initial: CartPoleState);
    fn state(&self) -> CartPoleState;
    /// Advances the plant by `dt` seconds with `control` held for the whole step. The plant
    /// enforces its own actuator limits.
    fn step(&mut self, control: &CartPoleControl, dt: f64);
}

/// The analytic cart-pole equations, advanced with the RK4 `Integrator`.
pub struct AnalyticCartPole {
    pub model: SecondOrderCartPole,
    state: CartPoleState,
}

impl AnalyticCartPole {
    pub fn new(model: SecondOrderCartPole) -> Self {
        Self { model, state: CartPoleState { x: 0.0, x_dot: 0.0, theta: 0.0, theta_dot: 0.0 } }
    }
}

impl CartPoleSim for AnalyticCartPole {
    fn reset(&mut self, initial: CartPoleState) {
        self.state = initial;
    }

    fn state(&self) -> CartPoleState {
        self.state
    }

    fn step(&mut self, control: &CartPoleControl, dt: f64) {
        self.state = Integrator::step(&self.model, &self.state, control, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stepping the wrapper is exactly one `Integrator::step`.
    #[test]
    fn steps_match_the_integrator() {
        let model = SecondOrderCartPole::default();
        let initial = CartPoleState { x: 0.1, x_dot: -0.2, theta: 0.3, theta_dot: 0.4 };
        let control = CartPoleControl { force: 2.0 };
        let mut plant = AnalyticCartPole::new(model);
        plant.reset(initial);
        plant.step(&control, 0.01);
        assert_eq!(plant.state(), Integrator::step(&model, &initial, &control, 0.01));
    }
}
