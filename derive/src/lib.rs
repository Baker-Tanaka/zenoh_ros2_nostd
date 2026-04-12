//! Derive macro for the [`RosMessage`] trait.
//!
//! Generates [`RosMessage`] implementations from struct attributes,
//! providing compile-time type name and hash constants compatible with
//! `rmw_zenoh_cpp`.
//!
//! # Usage
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd_derive::RosMessage;
//! use serde::{Serialize, Deserialize};
//!
//! #[derive(Serialize, Deserialize, RosMessage)]
//! #[ros_message(
//!     type_name = "std_msgs::msg::dds_::String_",
//!     type_hash = "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
//! )]
//! struct StringMsg {
//!     data: heapless::String<128>,
//! }
//! ```
//!
//! # Attributes
//!
//! | Attribute | Required | Description |
//! |-----------|----------|-------------|
//! | `type_name` | Yes | DDS type name with `dds_::` prefix and `_` suffix |
//! | `type_hash` | Yes | RIHS01 type hash from `rosidl` |

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput, LitStr};

/// Derive the `RosMessage` trait for a struct.
///
/// # Attributes
///
/// Place `#[ros_message(type_name = "...", type_hash = "...")]` on the struct.
///
/// - `type_name`: DDS type name following `rmw_zenoh_cpp` convention
///   (e.g., `"std_msgs::msg::dds_::String_"`)
/// - `type_hash`: RIHS01 type hash from `rosidl`
///   (e.g., `"RIHS01_df668c..."`)
///
/// # Example
///
/// ```rust,ignore
/// #[derive(Serialize, Deserialize, RosMessage)]
/// #[ros_message(
///     type_name = "geometry_msgs::msg::dds_::Twist_",
///     type_hash = "RIHS01_9b0e20a73f3b74c00f80ed1f26a4c7568a63aeec6e79ba04b64bcd5b7f49a2a5",
/// )]
/// struct Twist {
///     linear: Vector3,
///     angular: Vector3,
/// }
/// ```
#[proc_macro_derive(RosMessage, attributes(ros_message))]
pub fn derive_ros_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match derive_ros_message_impl(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

fn derive_ros_message_impl(
    input: &DeriveInput,
) -> Result<proc_macro2::TokenStream, syn::Error> {
    let name = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    let mut type_name: Option<LitStr> = None;
    let mut type_hash: Option<LitStr> = None;

    // Parse #[ros_message(type_name = "...", type_hash = "...", crate = ...)]
    for attr in &input.attrs {
        if !attr.path().is_ident("ros_message") {
            continue;
        }
        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("type_name") {
                let value = meta.value()?;
                type_name = Some(value.parse::<LitStr>()?);
                Ok(())
            } else if meta.path.is_ident("type_hash") {
                let value = meta.value()?;
                type_hash = Some(value.parse::<LitStr>()?);
                Ok(())
            } else {
                Err(meta.error("expected `type_name` or `type_hash`"))
            }
        })?;
    }

    let type_name = type_name.ok_or_else(|| {
        syn::Error::new_spanned(
            &input.ident,
            "missing `type_name` in `#[ros_message(...)]` attribute",
        )
    })?;

    let type_hash = type_hash.ok_or_else(|| {
        syn::Error::new_spanned(
            &input.ident,
            "missing `type_hash` in `#[ros_message(...)]` attribute",
        )
    })?;

    // Validate type_name follows DDS convention
    let tn = type_name.value();
    if !tn.contains("::dds_::") || !tn.ends_with('_') {
        return Err(syn::Error::new_spanned(
            &type_name,
            "type_name must follow DDS convention: `<pkg>::msg::dds_::<Type>_` \
             (with `dds_::` prefix and trailing `_`)",
        ));
    }

    // Validate type_hash starts with RIHS01_
    let th = type_hash.value();
    if !th.starts_with("RIHS01_") {
        return Err(syn::Error::new_spanned(
            &type_hash,
            "type_hash must start with `RIHS01_`",
        ));
    }

    Ok(quote! {
        impl #impl_generics zenoh_ros2_nostd::__private::RosMessage
            for #name #ty_generics #where_clause
        {
            const TYPE_NAME: &'static str = #type_name;
            const TYPE_HASH: &'static str = #type_hash;
        }
    })
}
