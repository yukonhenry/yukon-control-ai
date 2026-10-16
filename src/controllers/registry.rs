use crate::controllers::control_output::ControlOutput;
use crate::controllers::feedback::lqr::{LQRController, LQRParams};
use crate::controllers::feedback::swing_up_energy::{SwingUpEnergyController, SwingUpEnergyParams};
use crate::controllers::feedback::swing_up_energy_switch_lqr::{SwingUpEnergySwitchLQR, SwitchParams};
use crate::controllers::random_control::{RandomControl, RandomControlParams};
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};

pub type CartPoleController = Box<dyn ControlOutput<State = CartPoleState, Control = CartPoleControl>>;

/// Every controller under comparison, freshly constructed so no run inherits another's state.
/// `dt` is the period the controllers will be sampled at, which the LQR design depends on.
pub fn cart_pole_controllers(model: SecondOrderCartPole, dt: f64) -> Vec<(&'static str, CartPoleController)> {
    let lqr = LQRParams::design(&model, [1.0, 1.0, 10.0, 1.0], 0.1, dt);
    vec![
        ("Random", Box::new(RandomControl::new(RandomControlParams::default(), model))),
        ("Swing-up energy", Box::new(SwingUpEnergyController::new(SwingUpEnergyParams::default(), model))),
        ("LQR", Box::new(LQRController::new(lqr, model))),
        (
            "Swing-up + LQR",
            Box::new(SwingUpEnergySwitchLQR::new(SwingUpEnergyParams::default(), lqr, SwitchParams::default(), model)),
        ),
    ]
}
