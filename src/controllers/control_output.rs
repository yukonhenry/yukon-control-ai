pub(crate) trait ControlOutput {
    type State;
    type Control;
    fn compute_control(&mut self, state: &Self::State) -> Self::Control;
}