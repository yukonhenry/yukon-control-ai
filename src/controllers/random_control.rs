use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::SecondOrderCartPole;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RandomControlParams {
    pub control_min: f64,
    pub control_max: f64,
}

impl Default for RandomControlParams {
    fn default() -> Self {
        Self { control_min: -10.0, control_max: 10.0 }
    }
}

pub struct RandomControl {
    pub params: RandomControlParams,
    pub model: SecondOrderCartPole,
    pub control_output: f64,
}

impl RandomControl {
    pub fn new(params: RandomControlParams, model: SecondOrderCartPole) -> Self {
        Self { params, model, control_output: 0.0 }
    }

    pub fn random_control(&mut self) -> f64 {
        let force = rand::random::<f64>() * (self.params.control_max - self.params.control_min)
            + self.params.control_min;
        self.control_output = self.model.params.saturate_force(force);
        self.control_output
    }
}

impl ControlOutput for RandomControl {
    type State = ();
    type Control = f64;

    fn compute_control(&mut self, _state: &Self::State) -> Self::Control {
        self.random_control()
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
}