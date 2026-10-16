use crate::plant::cart_pole::cart_pole_state::{wrap_angle, CartPoleControl, CartPoleState};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MetricsParams {
    /// Per-component bounds on the error from upright: a run has converged once |x|, |x_dot|,
    /// |wrapped theta| and |theta_dot| all stay within these for the rest of it.
    pub settle_tolerance: CartPoleState,
    /// Length of the final stretch of the run, in seconds, averaged into the steady-state error.
    pub steady_state_seconds: f64,
}

impl Default for MetricsParams {
    fn default() -> Self {
        Self {
            settle_tolerance: CartPoleState { x: 0.05, x_dot: 0.05, theta: 0.02, theta_dot: 0.05 },
            steady_state_seconds: 2.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerMetrics {
    /// Mean upright error over the final `steady_state_seconds` of the run.
    pub steady_state_error: f64,
    /// First step from which every error component stays within `settle_tolerance`, or `None`
    /// if the run never settles.
    pub steps_to_converge: Option<usize>,
    /// Integral of the squared applied force over the run, in N^2 s: the sum of u^2 dt over
    /// every step, with each force held for its step.
    pub input_power: f64,
}

/// The state relative to upright, at rest, over the origin, with the pole angle folded so
/// that every full revolution maps back onto the same target.
fn upright_offset(state: &CartPoleState) -> CartPoleState {
    CartPoleState { theta: wrap_angle(state.theta), ..*state }
}

/// Euclidean distance of the state from upright, at rest, over the origin.
pub fn upright_error(state: &CartPoleState) -> f64 {
    let e = upright_offset(state);
    (e.x.powi(2) + e.x_dot.powi(2) + e.theta.powi(2) + e.theta_dot.powi(2)).sqrt()
}

/// Records a closed-loop run one step at a time and summarizes it into `ControllerMetrics`.
pub struct MetricsRecorder {
    pub params: MetricsParams,
    dt: f64,
    offsets: Vec<CartPoleState>,
    input_power: f64,
}

impl MetricsRecorder {
    pub fn new(params: MetricsParams, dt: f64) -> Self {
        Self { params, dt, offsets: Vec::new(), input_power: 0.0 }
    }

    /// Records the state a step started from and the control applied over it.
    pub fn record(&mut self, state: &CartPoleState, control: &CartPoleControl) {
        self.offsets.push(upright_offset(state));
        self.input_power += control.force.powi(2) * self.dt;
    }

    pub fn summarize(&self) -> ControllerMetrics {
        let window = (self.params.steady_state_seconds / self.dt).round() as usize;
        let tail = &self.offsets[self.offsets.len().saturating_sub(window)..];
        let steady_state_error = tail.iter().map(upright_error).sum::<f64>() / tail.len() as f64;

        // Written as `!(|e| <= tol)` so a diverged (NaN) state counts as unsettled.
        let tol = &self.params.settle_tolerance;
        let unsettled = |e: &CartPoleState| {
            !(e.x.abs() <= tol.x
                && e.x_dot.abs() <= tol.x_dot
                && e.theta.abs() <= tol.theta
                && e.theta_dot.abs() <= tol.theta_dot)
        };
        let steps_to_converge = match self.offsets.iter().rposition(unsettled) {
            None => Some(0),
            Some(last) if last + 1 < self.offsets.len() => Some(last + 1),
            Some(_) => None,
        };

        ControllerMetrics { steady_state_error, steps_to_converge, input_power: self.input_power }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leaning(theta: f64) -> CartPoleState {
        CartPoleState { x: 0.0, x_dot: 0.0, theta, theta_dot: 0.0 }
    }

    /// Tolerance 0.1 on every component; dt 0.5 s, so the 1 s steady-state window is 2 steps.
    fn run_states(states: &[CartPoleState], force: f64) -> ControllerMetrics {
        let params = MetricsParams {
            settle_tolerance: CartPoleState { x: 0.1, x_dot: 0.1, theta: 0.1, theta_dot: 0.1 },
            steady_state_seconds: 1.0,
        };
        let mut recorder = MetricsRecorder::new(params, 0.5);
        for state in states {
            recorder.record(state, &CartPoleControl { force });
        }
        recorder.summarize()
    }

    fn run(thetas: &[f64], force: f64) -> ControllerMetrics {
        run_states(&thetas.iter().map(|&theta| leaning(theta)).collect::<Vec<_>>(), force)
    }

    #[test]
    fn a_run_that_stays_upright_is_converged_from_the_start() {
        let metrics = run(&[0.0, 0.0, 0.0], 0.0);
        assert_eq!(metrics, ControllerMetrics { steady_state_error: 0.0, steps_to_converge: Some(0), input_power: 0.0 });
    }

    #[test]
    fn converges_at_the_step_after_the_last_excursion() {
        let metrics = run(&[1.0, 0.05, 0.5, 0.02, 0.04], 2.0);
        assert_eq!(metrics.steps_to_converge, Some(3));
        assert!((metrics.steady_state_error - 0.03).abs() < 1e-12);
        assert_eq!(metrics.input_power, 5.0 * 4.0 * 0.5);
    }

    #[test]
    fn a_run_that_ends_outside_tolerance_never_converges() {
        assert_eq!(run(&[0.0, 0.0, 0.5], 0.0).steps_to_converge, None);
    }

    /// Every component must be inside its own bound, not just the pole angle.
    #[test]
    fn a_cart_left_off_center_has_not_converged() {
        let off_center = CartPoleState { x: 0.5, ..leaning(0.0) };
        assert_eq!(run_states(&[leaning(0.0), off_center, leaning(0.0)], 0.0).steps_to_converge, Some(2));
        assert_eq!(run_states(&[leaning(0.0), leaning(0.0), off_center], 0.0).steps_to_converge, None);
    }

    /// A pole a full revolution from upright is upright, so it counts as settled.
    #[test]
    fn convergence_folds_the_pole_angle() {
        assert_eq!(run(&[2.0 * std::f64::consts::PI], 0.0).steps_to_converge, Some(0));
    }

    #[test]
    fn a_diverged_run_never_converges() {
        assert_eq!(run(&[0.0, f64::NAN], 0.0).steps_to_converge, None);
    }

    /// Hanging at -pi and at +pi is the same pose, and a full revolution away from upright is upright.
    #[test]
    fn upright_error_folds_the_pole_angle() {
        let pi = std::f64::consts::PI;
        assert!((upright_error(&leaning(pi)) - pi).abs() < 1e-12);
        assert!((upright_error(&leaning(-pi)) - pi).abs() < 1e-12);
        assert!(upright_error(&leaning(2.0 * pi)) < 1e-12);
    }
}
