mod plant;
mod simulator;
mod controllers;

use controllers::registry::cart_pole_controllers;
use plant::cart_pole::cart_pole_state::{CartPoleState, SecondOrderCartPole};
use simulator::cart_pole_sim::{AnalyticCartPole, CartPoleSim};
use simulator::closed_loop::simulate;
use simulator::metrics::MetricsParams;
use simulator::mujoco_cart_pole::MujocoCartPole;
use simulator::realtime::watch;

const DT: f64 = 0.01;
const STEPS: usize = 1500;
const MUJOCO_TIMESTEP: f64 = 0.002;

const USAGE: &str = "usage: yukon_control_ai [view [CONTROLLER] [SCENARIO]]\n\
    \n  (no arguments)  run every controller on both plants and print a comparison table\
    \n  view            play one controller on the MuJoCo plant in a viewer window, in real time\
    \n                  (defaults: \"Swing-up + LQR\", \"hanging at rest\"; names are case-insensitive)";

fn scenarios() -> [(&'static str, CartPoleState); 2] {
    [
        ("hanging at rest", CartPoleState { x: 0.0, x_dot: 0.0, theta: std::f64::consts::PI, theta_dot: 0.0 }),
        ("near upright", CartPoleState { x: 0.5, x_dot: 0.0, theta: 0.2, theta_dot: -0.3 }),
    ]
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    match args.as_slice() {
        [] => compare(),
        ["view", rest @ ..] if rest.len() <= 2 => {
            let controller = rest.first().copied().unwrap_or("Swing-up + LQR");
            let scenario = rest.get(1).copied().unwrap_or("hanging at rest");
            if let Err(message) = view(controller, scenario) {
                eprintln!("{message}\n\n{USAGE}");
                std::process::exit(2);
            }
        }
        _ => {
            eprintln!("{USAGE}");
            std::process::exit(2);
        }
    }
}

/// Runs every controller from each starting state on each plant in turn and prints a
/// comparison table. The controllers always design against the analytic model.
fn compare() {
    let model = SecondOrderCartPole::default();
    let mut plants: [(&str, Box<dyn CartPoleSim>); 2] = [
        ("analytic RK4", Box::new(AnalyticCartPole::new(model))),
        ("MuJoCo", Box::new(MujocoCartPole::new(&model.params, MUJOCO_TIMESTEP))),
    ];

    let metrics = MetricsParams::default();
    println!(
        "{STEPS} steps of {DT} s; converged = every error component within {:?} from then on; \
         steady-state error = mean upright error over the last {} s",
        metrics.settle_tolerance, metrics.steady_state_seconds,
    );
    for (scenario, initial) in scenarios() {
        println!("\n{scenario}: {initial:?}");
        println!(
            "{:<16} {:<14} {:>12} {:>16} {:>14}",
            "controller", "plant", "steady err", "converged step", "int u^2 dt",
        );
        for (plant_name, plant) in &mut plants {
            for (name, mut controller) in cart_pole_controllers(model, DT) {
                let m = simulate(controller.as_mut(), plant.as_mut(), initial, DT, STEPS, metrics);
                let converged = m.steps_to_converge.map_or("never".to_string(), |s| s.to_string());
                println!(
                    "{name:<16} {plant_name:<14} {:>12.4} {converged:>16} {:>14.1}",
                    m.steady_state_error, m.input_power,
                );
            }
        }
    }
}

/// Plays the named controller from the named scenario on the MuJoCo plant in a viewer window,
/// restarting each run, until the window is closed.
fn view(controller: &str, scenario: &str) -> Result<(), String> {
    let model = SecondOrderCartPole::default();
    let controller_names: Vec<&str> = cart_pole_controllers(model, DT).iter().map(|(n, _)| *n).collect();
    let controller = *controller_names
        .iter()
        .find(|n| n.eq_ignore_ascii_case(controller))
        .ok_or_else(|| format!("unknown controller {controller:?}; choose from {controller_names:?}"))?;
    let (scenario, initial) = scenarios()
        .into_iter()
        .find(|(n, _)| n.eq_ignore_ascii_case(scenario))
        .ok_or_else(|| {
            let names: Vec<&str> = scenarios().iter().map(|(n, _)| *n).collect();
            format!("unknown scenario {scenario:?}; choose from {names:?}")
        })?;

    println!("{controller} from {scenario} on MuJoCo; close the window to quit");
    let make_controller = || {
        cart_pole_controllers(model, DT)
            .into_iter()
            .find_map(|(n, c)| (n == controller).then_some(c))
            .expect("the controller was found by name above")
    };
    let mut plant = MujocoCartPole::new(&model.params, MUJOCO_TIMESTEP);
    watch(make_controller, &mut plant, initial, DT, STEPS).map_err(|e| format!("viewer failed: {e}"))
}
