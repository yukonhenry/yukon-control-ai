mod plant;
mod simulator;
mod controllers;

use controllers::registry::cart_pole_controllers;
use plant::cart_pole::cart_pole_state::{CartPoleState, SecondOrderCartPole};
use simulator::closed_loop::simulate;
use simulator::metrics::MetricsParams;

/// Runs every controller from each starting state in turn and prints a comparison table.
fn main() {
    let dt = 0.01;
    let steps = 1500;
    let model = SecondOrderCartPole::default();
    let scenarios = [
        ("hanging at rest", CartPoleState { x: 0.0, x_dot: 0.0, theta: std::f64::consts::PI, theta_dot: 0.0 }),
        ("near upright", CartPoleState { x: 0.5, x_dot: 0.0, theta: 0.2, theta_dot: -0.3 }),
    ];

    let metrics = MetricsParams::default();
    println!(
        "{steps} steps of {dt} s; converged = every error component within {:?} from then on; \
         steady-state error = mean upright error over the last {} s",
        metrics.settle_tolerance, metrics.steady_state_seconds,
    );
    for (scenario, initial) in scenarios {
        println!("\n{scenario}: {initial:?}");
        println!("{:<16} {:>12} {:>16} {:>14}", "controller", "steady err", "converged step", "int u^2 dt");
        for (name, mut controller) in cart_pole_controllers(model, dt) {
            let m = simulate(controller.as_mut(), &model, initial, dt, steps, metrics);
            let converged = m.steps_to_converge.map_or("never".to_string(), |s| s.to_string());
            println!("{name:<16} {:>12.4} {converged:>16} {:>14.1}", m.steady_state_error, m.input_power);
        }
    }
}
