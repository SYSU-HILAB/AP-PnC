use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=AP_PNC_DIR");
    if env::var_os("CARGO_FEATURE_FLATNESS_FFI").is_none() {
        return;
    }
    let root = PathBuf::from(env::var("AP_PNC_DIR").expect("absolute AP_PNC_DIR required"));
    assert!(root.is_absolute());
    assert_eq!(env::var("CARGO_CFG_TARGET_OS").unwrap(), "linux");
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    assert!(arch == "aarch64" || arch == "x86_64");
    let dir = root
        .join(".artifacts/flatness")
        .join(format!("linux-{arch}"));
    let archive = dir.join("libflatness.a");
    assert!(
        archive.is_file(),
        "run the container flatness codegen first"
    );
    println!("cargo:rerun-if-changed={}", archive.display());
    println!("cargo:rustc-link-search=native={}", dir.display());
    println!("cargo:rustc-link-lib=static=flatness");
    println!("cargo:rustc-link-lib=m");
}
