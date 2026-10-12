mod plant;
mod simulator;
mod controllers;

use controllers::control_output::ControlOutput;
use controllers::feedback::lqr::LQRParams;
use controllers::feedback::swing_up_energy::SwingUpEnergyParams;
use controllers::feedback::swing_up_energy_switch_lqr::{SwingUpEnergySwitchLQR, SwitchParams};
use plant::cart_pole::cart_pole_state::{wrap_angle, CartPoleState, SecondOrderCartPole};
use simulator::integrator::Integrator;

/// Swings the cart-pole up from hanging at rest and balances it, printing the trajectory.
fn main() {
    let dt = 0.01;
    let model = SecondOrderCartPole::default();
    let mut controller = SwingUpEnergySwitchLQR::new(
        SwingUpEnergyParams::default(),
        LQRParams::design(&model, [1.0, 1.0, 10.0, 1.0], 0.1, dt),
        SwitchParams::default(),
        model,
    );
    let mut state = CartPoleState { x: 0.0, x_dot: 0.0, theta: std::f64::consts::PI, theta_dot: 0.0 };

    println!("LQR gains: {:?}", controller.lqr.params.k);
    println!("{:>6} {:>8} {:>8} {:>8} {:>8} {:>8}  mode", "t", "x", "theta", "theta_d", "energy", "force");
    let mut mode = controller.mode;
    for step in 0..=1500 {
        let control = controller.compute_control(&state);
        if step % 25 == 0 || controller.mode != mode {
            println!(
                "{:>6.2} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>8.3}  {:?}",
                step as f64 * dt,
                state.x,
                wrap_angle(state.theta),
                state.theta_dot,
                model.pole_energy(&state),
                control.force,
                controller.mode,
            );
            mode = controller.mode;
        }
        state = Integrator::step(&model, &state, &control, dt);
    }
}
