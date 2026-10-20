//! mujoco-rs links libmujoco.so dynamically from where its `auto-download-mujoco` feature
//! extracts it, but sets no rpath, so the binaries can't find it at runtime. Embed one.

/// The MuJoCo version mujoco-rs pins (the `+mj-X.Y.Z` in its version).
const MUJOCO_VERSION: &str = "3.12.0";

fn main() {
    println!("cargo::rerun-if-env-changed=MUJOCO_DOWNLOAD_DIR");
    let is_linux = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux");
    if let (true, Ok(dir)) = (is_linux, std::env::var("MUJOCO_DOWNLOAD_DIR")) {
        let lib = std::path::Path::new(&dir).join(format!("mujoco-{MUJOCO_VERSION}")).join("lib");
        println!("cargo::rustc-link-arg=-Wl,-rpath,{}", lib.display());
    }
}
