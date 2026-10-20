use std::rc::Rc;

use mujoco_rs::prelude::*;

use crate::plant::cart_pole::cart_pole_params::CartPoleParams;
use crate::plant::cart_pole::cart_pole_state::{CartPoleControl, CartPoleState};
use crate::simulator::cart_pole_sim::CartPoleSim;

/// MJCF for a cart-pole with the same physics as `SecondOrderCartPole`, built from `params`.
///
/// - The cart slides along x (qpos[0] = x); the pole hinges about +y with its center of mass
///   `length` above the pivot, so qpos[1] = theta is zero upright and positive leaning toward +x.
/// - Equations 56-57 give the pole an inertia of (7/3) m l^2 about the pivot, more than a
///   uniform rod's (4/3) m l^2, so the inertia is set explicitly: (4/3) m l^2 about the center
///   of mass plus m l^2 from the offset.
/// - Both friction terms are viscous, so they are joint damping: mu_c on the slide and mu_p on
///   the hinge.
/// - Inertia never comes from geoms and geoms never collide; they are only for viewing.
pub fn cart_pole_mjcf(params: &CartPoleParams, timestep: f64) -> String {
    let CartPoleParams {
        mass_cart,
        mass_pole: m,
        length: l,
        gravity,
        friction_cart,
        friction_pole,
        max_force,
        min_force,
    } = *params;
    let pole_inertia = (4.0 / 3.0) * m * l * l;
    let pole_top = 2.0 * l;
    format!(
        r#"<mujoco model="cart_pole">
  <compiler inertiafromgeom="false"/>
  <option timestep="{timestep}" integrator="RK4" gravity="0 0 -{gravity}"/>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <light pos="0 -3 3" dir="0 1 -1"/>
    <geom name="rail" type="capsule" fromto="-5 0 0 5 0 0" size="0.01" rgba="0.4 0.4 0.4 1"/>
    <body name="cart">
      <joint name="slide" type="slide" axis="1 0 0" damping="{friction_cart}"/>
      <inertial pos="0 0 0" mass="{mass_cart}" diaginertia="1 1 1"/>
      <geom name="cart" type="box" size="0.15 0.1 0.05" rgba="0.2 0.4 0.8 1"/>
      <body name="pole">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="{friction_pole}"/>
        <inertial pos="0 0 {l}" mass="{m}" diaginertia="{pole_inertia} {pole_inertia} {pole_inertia}"/>
        <geom name="pole" type="capsule" fromto="0 0 0 0 0 {pole_top}" size="0.02" rgba="0.8 0.3 0.2 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="push" joint="slide" gear="1" ctrllimited="true" ctrlrange="{min_force} {max_force}"/>
  </actuator>
</mujoco>"#
    )
}

/// The cart-pole simulated by MuJoCo, with each control held over however many MuJoCo
/// timesteps make up the control step.
pub struct MujocoCartPole {
    data: MjData<Rc<MjModel>>,
    timestep: f64,
}

impl MujocoCartPole {
    /// Compiles the model for `params` with MuJoCo stepping at `timestep` seconds.
    pub fn new(params: &CartPoleParams, timestep: f64) -> Self {
        let model = MjModel::from_xml_string(&cart_pole_mjcf(params, timestep))
            .expect("the generated cart-pole MJCF should compile");
        Self { data: MjData::new(Rc::new(model)), timestep }
    }
}

impl CartPoleSim for MujocoCartPole {
    fn reset(&mut self, initial: CartPoleState) {
        self.data.reset();
        self.data.qpos_mut().copy_from_slice(&[initial.x, initial.theta]);
        self.data.qvel_mut().copy_from_slice(&[initial.x_dot, initial.theta_dot]);
    }

    fn state(&self) -> CartPoleState {
        let (qpos, qvel) = (self.data.qpos(), self.data.qvel());
        CartPoleState { x: qpos[0], x_dot: qvel[0], theta: qpos[1], theta_dot: qvel[1] }
    }

    /// `dt` must be a whole number of MuJoCo timesteps. MuJoCo clamps the force to `ctrlrange`.
    fn step(&mut self, control: &CartPoleControl, dt: f64) {
        let substeps = (dt / self.timestep).round();
        assert!(
            substeps >= 1.0 && (substeps * self.timestep - dt).abs() < 1e-9,
            "control step {dt} is not a whole number of MuJoCo timesteps of {}",
            self.timestep,
        );
        self.data.ctrl_mut()[0] = control.force;
        for _ in 0..substeps as usize {
            self.data.step();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::random_control::{RandomControl, RandomControlParams};
    use crate::plant::cart_pole::cart_pole_state::SecondOrderCartPole;
    use crate::simulator::cart_pole_sim::AnalyticCartPole;

    fn mujoco() -> MujocoCartPole {
        MujocoCartPole::new(&CartPoleParams::default(), 0.002)
    }

    fn at_rest(theta: f64) -> CartPoleState {
        CartPoleState { x: 0.0, x_dot: 0.0, theta, theta_dot: 0.0 }
    }

    /// One 10 ms control step from `initial` under `force`.
    fn step_from(initial: CartPoleState, force: f64) -> CartPoleState {
        let mut plant = mujoco();
        plant.reset(initial);
        plant.step(&CartPoleControl { force }, 0.01);
        plant.state()
    }

    #[test]
    fn reset_sets_the_state() {
        let initial = CartPoleState { x: 0.1, x_dot: -0.2, theta: 0.3, theta_dot: 0.4 };
        let mut plant = mujoco();
        plant.reset(initial);
        assert_eq!(plant.state(), initial);
    }

    #[test]
    fn upright_at_rest_is_an_equilibrium() {
        assert_eq!(step_from(at_rest(0.0), 0.0), at_rest(0.0));
    }

    /// Checks the slide and hinge axes: pushing right moves the cart right and tips the
    /// upright pole back to the left.
    #[test]
    fn pushing_right_moves_the_cart_right_and_tips_the_pole_left() {
        let next = step_from(at_rest(0.0), 5.0);
        assert!(next.x_dot > 0.0, "cart velocity {}", next.x_dot);
        assert!(next.theta_dot < 0.0, "pole angular velocity {}", next.theta_dot);
    }

    #[test]
    fn a_leaning_pole_falls_the_way_it_leans() {
        for theta in [0.1, -0.1] {
            let next = step_from(at_rest(theta), 0.0);
            assert!(next.theta_dot * theta > 0.0, "theta {theta} gave theta_dot {}", next.theta_dot);
            assert!(next.x_dot * theta < 0.0, "theta {theta} gave x_dot {}", next.x_dot);
        }
    }

    /// A force past the actuator limit moves the cart exactly as the limit itself does.
    #[test]
    fn force_is_saturated_at_the_actuator_limits() {
        assert_eq!(step_from(at_rest(0.0), 50.0), step_from(at_rest(0.0), 10.0));
        assert_eq!(step_from(at_rest(0.0), -50.0), step_from(at_rest(0.0), -10.0));
    }

    #[test]
    #[should_panic(expected = "whole number of MuJoCo timesteps")]
    fn a_control_step_must_be_a_whole_number_of_timesteps() {
        mujoco().step(&CartPoleControl { force: 0.0 }, 0.003);
    }

    /// MuJoCo and the analytic equations, started together and driven by the same random
    /// forces, follow the same trajectory. This pins down inertia, damping, gravity, the
    /// actuator and the sign conventions together.
    #[test]
    fn matches_the_analytic_model() {
        let model = SecondOrderCartPole::default();
        let mut analytic = AnalyticCartPole::new(model);
        let mut mujoco = mujoco();
        let mut forces = RandomControl::new(RandomControlParams::default(), model);

        for initial in [at_rest(0.2), CartPoleState { x: 0.5, x_dot: 1.0, theta: 2.5, theta_dot: -3.0 }] {
            analytic.reset(initial);
            mujoco.reset(initial);
            for step in 0..300 {
                // Forces past the limits exercise both plants' saturation.
                let control = CartPoleControl { force: 1.5 * forces.random_control() };
                analytic.step(&control, 0.01);
                mujoco.step(&control, 0.01);
                let (a, m) = (analytic.state(), mujoco.state());
                let gap = [a.x - m.x, a.x_dot - m.x_dot, a.theta - m.theta, a.theta_dot - m.theta_dot]
                    .iter()
                    .fold(0.0_f64, |worst, d| worst.max(d.abs()));
                assert!(gap < 1e-6, "from {initial:?}, step {step}: analytic {a:?} vs MuJoCo {m:?}");
            }
        }
    }
}
