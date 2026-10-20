use std::thread::sleep;
use std::time::{Duration, Instant};

use mujoco_rs::prelude::*;

use crate::controllers::registry::CartPoleController;
use crate::plant::cart_pole::cart_pole_state::CartPoleState;
use crate::simulator::cart_pole_sim::CartPoleSim;
use crate::simulator::mujoco_cart_pole::MujocoCartPole;

/// How long the final frame of a run stays on screen before the run restarts.
const PAUSE_BETWEEN_RUNS: Duration = Duration::from_secs(1);

/// Plays the same closed loop as `simulate` in a MuJoCo viewer window at wall-clock speed,
/// restarting from `initial` with a fresh controller from `make_controller` after every
/// `steps` steps, until the window is closed.
pub fn watch(
    make_controller: impl Fn() -> CartPoleController,
    plant: &mut MujocoCartPole,
    initial: CartPoleState,
    dt: f64,
    steps: usize,
) -> Result<(), MjViewerError> {
    let mut viewer = plant.launch_viewer()?;
    let tick = Duration::from_secs_f64(dt);
    while viewer.running() {
        let mut controller = make_controller();
        plant.reset(initial);
        let mut deadline = Instant::now();
        for _ in 0..steps {
            if !viewer.running() {
                return Ok(());
            }
            let control = controller.compute_control(&plant.state());
            plant.step(&control, dt);
            plant.show(&mut viewer)?;
            // Pace against a running deadline rather than sleeping `dt` per step, so time
            // spent simulating and rendering doesn't accumulate as drift.
            deadline += tick;
            sleep(deadline.saturating_duration_since(Instant::now()));
        }
        let resume = Instant::now() + PAUSE_BETWEEN_RUNS;
        while viewer.running() && Instant::now() < resume {
            plant.show(&mut viewer)?;
            sleep(tick);
        }
    }
    Ok(())
}
