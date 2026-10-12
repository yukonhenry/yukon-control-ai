#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartPoleParams {
    pub mass_cart: f64,            // kg
    pub mass_pole: f64,            // kg
    pub length: f64,               // m (half the pole length)
    pub gravity: f64,              // m/s^2
    pub friction_cart: f64,        // coefficient of friction between cart and track (mu_c)
    pub friction_pole: f64,        // coefficient of friction at the pole's pivot (mu_p)
    pub max_force: f64,            // N
    pub min_force: f64,            // N (negative for reverse)
}

impl Default for CartPoleParams {
    /// The classic Barto, Sutton & Anderson parameters, with the friction coefficients used
    /// by the SharpNEAT cart-pole reference.
    fn default() -> Self {
        Self {
            mass_cart: 1.0,
            mass_pole: 0.1,
            length: 0.5,
            gravity: 9.8,
            friction_cart: 0.0005,
            friction_pole: 0.000002,
            max_force: 10.0,
            min_force: -10.0,
        }
    }
}

impl CartPoleParams {
    /// Combined mass of the cart and pole (M)
    pub fn total_mass(&self) -> f64 {
        self.mass_cart + self.mass_pole
    }

    /// Clamps a requested cart force into the actuator's [min_force, max_force] range
    pub fn saturate_force(&self, force: f64) -> f64 {
        force.clamp(self.min_force, self.max_force)
    }
}
