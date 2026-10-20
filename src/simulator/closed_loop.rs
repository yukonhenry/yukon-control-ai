use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState};
use crate::simulator::cart_pole_sim::CartPoleSim;
use crate::simulator::metrics::{ControllerMetrics, MetricsParams, MetricsRecorder};

/// Runs `controller` on `plant` from `initial` for `steps` steps of `dt`, holding each control
/// for its step, and scores the run.
pub fn simulate(
    controller: &mut dyn ControlOutput<State = CartPoleState, Control = CartPoleControl>,
    plant: &mut dyn CartPoleSim,
    initial: CartPoleState,
    dt: f64,
    steps: usize,
    metrics: MetricsParams,
) -> ControllerMetrics {
    let mut recorder = MetricsRecorder::new(metrics, dt);
    plant.reset(initial);
    for _ in 0..steps {
        let state = plant.state();
        let control = controller.compute_control(&state);
        recorder.record(&state, &control);
        plant.step(&control, dt);
    }
    recorder.summarize()
}
