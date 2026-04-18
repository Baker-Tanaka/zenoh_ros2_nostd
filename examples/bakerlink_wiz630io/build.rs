use std::fs;

fn main() {
    // Re-run this build script whenever memory.x or the config file changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=config.json");

    // Tell the linker where to find memory.x
    println!(
        "cargo:rustc-link-search={}",
        std::env::var("CARGO_MANIFEST_DIR").unwrap()
    );

    let json = fs::read_to_string("config.json").unwrap_or_else(|_| {
        panic!(
            "\n\n\
             config.json not found.\n\
             Copy config.json.example to config.json and set router_addr.\n\
             This file is git-ignored and will not be committed.\n"
        )
    });

    let router_addr = extract_str(&json, "router_addr").unwrap_or_else(|| {
        panic!(
            "config.json: missing \"router_addr\" field.\n\
             Example: \"router_addr\": \"192.168.1.1:7447\""
        )
    });

    println!("cargo:rustc-env=ZENOH_ROUTER_ADDR={router_addr}");
}

/// Extract the string value for `key` from a simple flat JSON object.
fn extract_str<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let needle = format!("\"{key}\"");
    let start = json.find(needle.as_str())? + needle.len();
    let after_key = json[start..].trim_start();
    let after_colon = after_key.strip_prefix(':')?.trim_start();
    let inner = after_colon.strip_prefix('"')?;
    let end = inner.find('"')?;
    Some(&inner[..end])
}
