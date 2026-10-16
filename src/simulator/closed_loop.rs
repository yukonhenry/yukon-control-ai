use crate::controllers::control_output::ControlOutput;
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};
use crate::simulator::integrator::Integrator;
use crate::simulator::metrics::{ControllerMetrics, MetricsParams, MetricsRecorder};

/// Runs `controller` on `model` from `initial` for `steps` steps of `dt`, holding each control
/// for its step, and scores the run.
pub fn simulate(
    controller: &mut dyn ControlOutput<State = CartPoleState, Control = CartPoleControl>,
    model: &SecondOrderCartPole,
    initial: CartPoleState,
    dt: f64,
    steps: usize,
    metrics: MetricsParams,
) -> ControllerMetrics {
    let mut recorder = MetricsRecorder::new(metrics, dt);
    let mut state = initial;
    for _ in 0..steps {
        let control = controller.compute_control(&state);
        recorder.record(&state, &control);
        state = Integrator::step(model, &state, &control, dt);
    }
    recorder.summarize()
}
