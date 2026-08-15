fn main() {
    println!("cargo:rerun-if-changed=src/oracle.c");
    println!("cargo:rerun-if-changed=src/oracle_float.c");
    println!("cargo:rerun-if-changed=../reference/bosch/bme68x.c");
    println!("cargo:rerun-if-changed=../reference/bosch/bme68x.h");
    println!("cargo:rerun-if-changed=../reference/bosch/bme68x_defs.h");

    cc::Build::new()
        .file("src/oracle.c")
        .include("../reference/bosch")
        .define("BME68X_DO_NOT_USE_FPU", None)
        .flag_if_supported("-fwrapv")
        .warnings(true)
        .compile("bme68x_reference_oracle");

    cc::Build::new()
        .file("src/oracle_float.c")
        .include("../reference/bosch")
        .flag_if_supported("-ffp-contract=off")
        .warnings(true)
        .compile("bme68x_float_reference_oracle");
}
