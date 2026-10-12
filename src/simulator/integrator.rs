use crate::plant::kinodynamic_base::KinodynamicBase;

pub struct Integrator;

impl Integrator {
    /// Advances any kinodynamic model by dt seconds using RK4 integration
    pub fn step<M: KinodynamicBase>(model: &M, state: &M::State, control: &M::Control, dt: f64) -> M::State {
        let safe_control = model.clamp_inputs(control);

        let k1 = model.dynamics(state, &safe_control);

        let state_k2 = state.clone() + k1.clone() * (0.5 * dt);
        let k2 = model.dynamics(&state_k2, &safe_control);

        let state_k3 = state.clone() + k2.clone() * (0.5 * dt);
        let k3 = model.dynamics(&state_k3, &safe_control);

        let state_k4 = state.clone() + k3.clone() * dt;
        let k4 = model.dynamics(&state_k4, &safe_control);

        let next_state = state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
        model.clamp_states(&next_state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plant::cart_pole::cart_pole_params::CartPoleParams;
    use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState, SecondOrderCartPole};

    fn cart_pole() -> SecondOrderCartPole {
        SecondOrderCartPole {
            params: CartPoleParams {
                gravity: 9.81,
                friction_cart: 0.0,
                friction_pole: 0.0,
                ..Default::default()
            },
        }
    }

    #[test]
    fn a_cart_pole_can_be_integrated() {
        let model = cart_pole();
        let state = CartPoleState {
            x: 0.0,
            x_dot: 0.0,
            theta: std::f64::consts::PI / 6.0, // 30 degrees
            theta_dot: 0.0,
        };
        let control = CartPoleControl { force: 0.0 };
        let dt = 0.01;

        let next_state = Integrator::step(&model, &state, &control, dt);
        assert!(next_state.x != state.x || next_state.theta != state.theta);
    }

    #[test]
    fn a_cart_pole_can_be_integrated_with_control() {
        let model = cart_pole();
        let state = CartPoleState {
            x: 0.0,
            x_dot: 0.0,
            theta: std::f64::consts::PI / 6.0, // 30 degrees
            theta_dot: 0.0,
        };
        let control = CartPoleControl { force: 10.0 };
        let dt = 0.01;

        let next_state = Integrator::step(&model, &state, &control, dt);
        assert!(next_state.x != state.x || next_state.theta != state.theta);
    }

    #[test]
    fn a_cart_pole_can_be_integrated_with_large_control() {
        let model = cart_pole();
        let state = CartPoleState {
            x: 0.0,
            x_dot: 0.0,
            theta: std::f64::consts::PI / 6.0, // 30 degrees
            theta_dot: 0.0,
        };
        let control = CartPoleControl { force: 1000.0 };
        let dt = 0.01;

        let next_state = Integrator::step(&model, &state, &control, dt);
        assert!(next_state.x != state.x || next_state.theta != state.theta);
    }
}
