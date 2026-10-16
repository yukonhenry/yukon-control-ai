use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RandomControlParams {
    pub control_min: f64,
    pub control_max: f64,
    pub seed: u64, // Seeds the generator so a run's force sequence is reproducible
}

impl Default for RandomControlParams {
    fn default() -> Self {
        Self { control_min: -10.0, control_max: 10.0, seed: 0 }
    }
}

pub struct RandomControl {
    pub params: RandomControlParams,
    pub model: SecondOrderCartPole,
    pub control_output: f64,
    rng: StdRng,
}

impl RandomControl {
    pub fn new(params: RandomControlParams, model: SecondOrderCartPole) -> Self {
        Self { params, model, control_output: 0.0, rng: StdRng::seed_from_u64(params.seed) }
    }

    pub fn random_control(&mut self) -> f64 {
        let force = self.rng.random::<f64>() * (self.params.control_max - self.params.control_min)
            + self.params.control_min;
        self.control_output = self.model.params.saturate_force(force);
        self.control_output
    }
}

impl ControlOutput for RandomControl {
    type State = CartPoleState;
    type Control = CartPoleControl;

    fn compute_control(&mut self, _state: &Self::State) -> Self::Control {
        CartPoleControl { force: self.random_control() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_random_control_params_default() {
        let params = RandomControlParams::default();
        assert_eq!(params.control_min, -10.0);
        assert_eq!(params.control_max, 10.0);
    }

    #[test]
    fn test_random_control() {
        let params = RandomControlParams::default();
        let model = SecondOrderCartPole::default();
        let mut controller = RandomControl::new(params, model);
        let control = controller.random_control();
        assert!(control >= params.control_min && control <= params.control_max);
    }

    /// Two controllers built with the same seed command the same force sequence.
    #[test]
    fn the_same_seed_reproduces_the_same_forces() {
        let model = SecondOrderCartPole::default();
        let mut a = RandomControl::new(RandomControlParams::default(), model);
        let mut b = RandomControl::new(RandomControlParams::default(), model);
        for _ in 0..10 {
            assert_eq!(a.random_control(), b.random_control());
        }
    }
}
