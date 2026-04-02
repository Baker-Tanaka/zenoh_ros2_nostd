//! CDR (Common Data Representation) serialization for ROS2 messages.
//!
//! Implements the OMG CDR encoding in Little Endian, as used by ROS2/DDS.
//! This is a `no_std` implementation using `serde`.
//!
//! # CDR Encapsulation
//!
//! ROS2 messages transmitted over DDS/Zenoh are prefixed with a 4-byte
//! encapsulation header:
//! ```text
//! [0x00, 0x01, 0x00, 0x00]  — CDR Little Endian
//! ```

mod de;
mod error;
mod ser;

pub use de::CdrDeserializer;
pub use error::CdrError;
pub use ser::CdrSerializer;

use serde::{Deserialize, Serialize};

/// CDR LE encapsulation header (4 bytes).
pub const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Serialize a value to CDR LE into the provided buffer.
///
/// Returns the number of bytes written.
pub fn serialize_to_buf<T: Serialize>(buf: &mut [u8], value: &T) -> Result<usize, CdrError> {
    let mut ser = CdrSerializer::new(buf);
    value.serialize(&mut ser)?;
    Ok(ser.position())
}

/// Serialize a value with the CDR LE encapsulation header.
///
/// The first 4 bytes are the encapsulation header, followed by CDR data.
/// Returns the total number of bytes written (including header).
pub fn serialize_with_header<T: Serialize>(buf: &mut [u8], value: &T) -> Result<usize, CdrError> {
    if buf.len() < 4 {
        return Err(CdrError::BufferOverflow);
    }
    buf[..4].copy_from_slice(&CDR_LE_HEADER);
    let mut ser = CdrSerializer::new(&mut buf[4..]);
    value.serialize(&mut ser)?;
    Ok(4 + ser.position())
}

/// Deserialize a value from CDR LE bytes.
///
/// Returns the deserialized value and the number of bytes consumed.
pub fn deserialize_from_buf<'de, T: Deserialize<'de>>(
    buf: &'de [u8],
) -> Result<(T, usize), CdrError> {
    let mut de = CdrDeserializer::new(buf);
    let value = T::deserialize(&mut de)?;
    Ok((value, de.position()))
}

/// Deserialize a value from CDR LE bytes with encapsulation header.
///
/// Validates and skips the 4-byte header, then deserializes.
pub fn deserialize_with_header<'de, T: Deserialize<'de>>(
    buf: &'de [u8],
) -> Result<(T, usize), CdrError> {
    if buf.len() < 4 {
        return Err(CdrError::BufferUnderflow);
    }
    if buf[0..2] != [0x00, 0x01] {
        return Err(CdrError::UnsupportedEncapsulation);
    }
    let mut de = CdrDeserializer::new(&buf[4..]);
    let value = T::deserialize(&mut de)?;
    Ok((value, 4 + de.position()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct Vector3 {
        x: f64,
        y: f64,
        z: f64,
    }

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct Twist {
        linear: Vector3,
        angular: Vector3,
    }

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct StdString {
        data: heapless::String<256>,
    }

    #[test]
    fn test_roundtrip_twist() {
        let msg = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            angular: Vector3 {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            },
        };

        let mut buf = [0u8; 256];
        let len = serialize_to_buf(&mut buf, &msg).unwrap();
        assert_eq!(len, 48); // 6 * f64 = 48 bytes

        let (decoded, consumed): (Twist, _) = deserialize_from_buf(&buf[..len]).unwrap();
        assert_eq!(consumed, 48);
        assert_eq!(decoded, msg);
    }

    #[test]
    fn test_roundtrip_with_header() {
        let msg = Vector3 {
            x: 1.0,
            y: 0.0,
            z: -1.0,
        };

        let mut buf = [0u8; 128];
        let len = serialize_with_header(&mut buf, &msg).unwrap();
        assert_eq!(len, 4 + 24); // header + 3 * f64

        assert_eq!(&buf[0..4], &CDR_LE_HEADER);

        let (decoded, consumed): (Vector3, _) = deserialize_with_header(&buf[..len]).unwrap();
        assert_eq!(consumed, 4 + 24);
        assert_eq!(decoded, msg);
    }

    #[test]
    fn test_primitives() {
        let mut buf = [0u8; 64];

        // u8
        let len = serialize_to_buf(&mut buf, &42u8).unwrap();
        assert_eq!(len, 1);
        assert_eq!(buf[0], 42);

        // u32
        let len = serialize_to_buf(&mut buf, &0x12345678u32).unwrap();
        assert_eq!(len, 4);
        assert_eq!(&buf[..4], &[0x78, 0x56, 0x34, 0x12]); // LE

        // bool
        let len = serialize_to_buf(&mut buf, &true).unwrap();
        assert_eq!(len, 1);
        assert_eq!(buf[0], 1);
    }

    #[test]
    fn test_string_alignment() {
        // CDR string: u32 len (with null), bytes, null
        let s: heapless::String<32> = heapless::String::try_from("BLUE").unwrap();

        let mut buf = [0u8; 64];
        let len = serialize_to_buf(&mut buf, &s).unwrap();

        // u32 length (5 = 4 chars + null) = 4 bytes, "BLUE" = 4 bytes, null = 1 byte
        assert_eq!(len, 4 + 5);

        let (decoded, _): (heapless::String<32>, _) = deserialize_from_buf(&buf[..len]).unwrap();
        assert_eq!(decoded.as_str(), "BLUE");
    }

    #[test]
    fn test_struct_alignment() {
        // Struct with mixed types to test alignment padding
        #[derive(Debug, PartialEq, Serialize, Deserialize)]
        struct Mixed {
            a: u8,  // offset 0, 1 byte
            b: u32, // offset 4 (padded from 1 to 4), 4 bytes
            c: u8,  // offset 8, 1 byte
            d: u16, // offset 10 (padded from 9 to 10), 2 bytes
        }

        let msg = Mixed {
            a: 0xAA,
            b: 0x12345678,
            c: 0xBB,
            d: 0xCDEF,
        };

        let mut buf = [0u8; 64];
        let len = serialize_to_buf(&mut buf, &msg).unwrap();
        // a(1) + pad(3) + b(4) + c(1) + pad(1) + d(2) = 12
        assert_eq!(len, 12);

        let (decoded, _): (Mixed, _) = deserialize_from_buf(&buf[..len]).unwrap();
        assert_eq!(decoded, msg);
    }
}
