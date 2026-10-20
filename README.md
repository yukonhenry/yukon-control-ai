# Yukon Control + AI

## Overview
Compare performance of nonlinear feedback based control with real-time control based on Reinforcement Learning, as well as LLM-based control.

Initially use Cart-Pole model as the plant to control.

## Instructions
* Install Rust.
* Run `cargo build` in the root directory to build the backend.
* Run `cargo run` in the root directory to run the performance comparison application.

## Current Status
1. Cart-Pole model implemented. [[1]](#ref-green)
2. Nonlinear feedback control using Energy Shaping and switch to Linear Quadratic Regulator (LQR) implemented.
3. Mujoco integration, in addition to native integrator.

## Todo
1. RL (various, starting with Q-learning) based control.
2. LLM-based control.

## References
1. <a id="ref-green"></a>Colin D. Green, [“Equations of Motion for the Cart and Pole Control Task”](https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html)
2. <a id="ref-sachith"></a>[Mastering CartPole: A Complete Journey Through 6 Reinforcement Learning Algorithms](https://medium.com/@sachith.icc/mastering-cartpole-a-complete-journey-through-6-reinforcement-learning-algorithms-06d92fa6d601#algorithm-2-q-learning)
3. <a id="ref-jev"></a>[Inverted Pendulum Control: A Comparative Study from Inverted Pendulum Control: A Comparative Study from Conventional Control to Reinforcement Learning](https://journal.ugm.ac.id/v3/JNTETI/article/view/7065)
