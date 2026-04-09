//! CDR Little Endian Deserializer (serde-based, no_std).

use super::CdrError;
use serde::de::{self, DeserializeSeed, IntoDeserializer, Visitor};

/// CDR Little Endian deserializer reading from a `&[u8]` slice.
pub struct CdrDeserializer<'de> {
    input: &'de [u8],
    pos: usize,
}

impl<'de> CdrDeserializer<'de> {
    /// Create a new deserializer reading from `input`.
    pub fn new(input: &'de [u8]) -> Self {
        Self { input, pos: 0 }
    }

    /// Current read position (number of bytes consumed).
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Align the read cursor to an `n`-byte boundary.
    fn align(&mut self, n: usize) -> Result<(), CdrError> {
        let rem = self.pos % n;
        if rem != 0 {
            let padding = n - rem;
            if self.pos + padding > self.input.len() {
                return Err(CdrError::BufferUnderflow);
            }
            self.pos += padding;
        }
        Ok(())
    }

    /// Read `n` bytes from the current position.
    fn read_bytes(&mut self, n: usize) -> Result<&'de [u8], CdrError> {
        if self.pos + n > self.input.len() {
            return Err(CdrError::BufferUnderflow);
        }
        let slice = &self.input[self.pos..self.pos + n];
        self.pos += n;
        Ok(slice)
    }

    fn read_u8(&mut self) -> Result<u8, CdrError> {
        Ok(self.read_bytes(1)?[0])
    }

    fn read_u16(&mut self) -> Result<u16, CdrError> {
        self.align(2)?;
        let b = self.read_bytes(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }

    fn read_u32(&mut self) -> Result<u32, CdrError> {
        self.align(4)?;
        let b = self.read_bytes(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn read_u64(&mut self) -> Result<u64, CdrError> {
        self.align(8)?;
        let b = self.read_bytes(8)?;
        Ok(u64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }
}

impl<'de, 'a> de::Deserializer<'de> for &'a mut CdrDeserializer<'de> {
    type Error = CdrError;

    fn deserialize_any<V: Visitor<'de>>(self, _visitor: V) -> Result<V::Value, CdrError> {
        Err(CdrError::DeserializeError) // CDR is not self-describing
    }

    fn deserialize_bool<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let v = self.read_u8()?;
        match v {
            0 => visitor.visit_bool(false),
            1 => visitor.visit_bool(true),
            _ => Err(CdrError::InvalidBool),
        }
    }

    fn deserialize_i8<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_i8(self.read_u8()? as i8)
    }

    fn deserialize_u8<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_u8(self.read_u8()?)
    }

    fn deserialize_i16<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_i16(self.read_u16()? as i16)
    }

    fn deserialize_u16<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_u16(self.read_u16()?)
    }

    fn deserialize_i32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_i32(self.read_u32()? as i32)
    }

    fn deserialize_u32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_u32(self.read_u32()?)
    }

    fn deserialize_i64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_i64(self.read_u64()? as i64)
    }

    fn deserialize_u64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_u64(self.read_u64()?)
    }

    fn deserialize_f32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        self.align(4)?;
        let b = self.read_bytes(4)?;
        visitor.visit_f32(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn deserialize_f64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        self.align(8)?;
        let b = self.read_bytes(8)?;
        visitor.visit_f64(f64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    fn deserialize_char<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let c = self.read_u8()? as char;
        visitor.visit_char(c)
    }

    fn deserialize_str<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let len = self.read_u32()? as usize;
        if len == 0 {
            return visitor.visit_borrowed_str("");
        }
        // len includes the null terminator
        let bytes = self.read_bytes(len)?;
        let str_bytes = &bytes[..len - 1]; // exclude null
        let s = core::str::from_utf8(str_bytes).map_err(|_| CdrError::InvalidUtf8)?;
        visitor.visit_borrowed_str(s)
    }

    fn deserialize_string<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        self.deserialize_str(visitor)
    }

    fn deserialize_bytes<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let len = self.read_u32()? as usize;
        let bytes = self.read_bytes(len)?;
        visitor.visit_borrowed_bytes(bytes)
    }

    fn deserialize_byte_buf<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        self.deserialize_bytes(visitor)
    }

    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let present = self.read_u8()?;
        match present {
            0 => visitor.visit_none(),
            1 => visitor.visit_some(self),
            _ => Err(CdrError::InvalidBool),
        }
    }

    fn deserialize_unit<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_unit()
    }

    fn deserialize_unit_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_unit()
    }

    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_newtype_struct(self)
    }

    fn deserialize_seq<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let len = self.read_u32()? as usize;
        visitor.visit_seq(SeqAccess {
            de: self,
            remaining: len,
        })
    }

    fn deserialize_tuple<V: Visitor<'de>>(
        self,
        len: usize,
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_seq(SeqAccess {
            de: self,
            remaining: len,
        })
    }

    fn deserialize_tuple_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        len: usize,
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_seq(SeqAccess {
            de: self,
            remaining: len,
        })
    }

    fn deserialize_map<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        let len = self.read_u32()? as usize;
        visitor.visit_map(MapAccess {
            de: self,
            remaining: len,
        })
    }

    fn deserialize_struct<V: Visitor<'de>>(
        self,
        _name: &'static str,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_seq(SeqAccess {
            de: self,
            remaining: fields.len(),
        })
    }

    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _name: &'static str,
        _variants: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        visitor.visit_enum(EnumAccess { de: self })
    }

    fn deserialize_identifier<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        self.deserialize_u32(visitor)
    }

    fn deserialize_ignored_any<V: Visitor<'de>>(self, _visitor: V) -> Result<V::Value, CdrError> {
        Err(CdrError::DeserializeError)
    }
}

// --- SeqAccess for sequences, tuples, and structs ---

struct SeqAccess<'a, 'de> {
    de: &'a mut CdrDeserializer<'de>,
    remaining: usize,
}

impl<'a, 'de> de::SeqAccess<'de> for SeqAccess<'a, 'de> {
    type Error = CdrError;

    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, CdrError> {
        if self.remaining == 0 {
            return Ok(None);
        }
        self.remaining -= 1;
        seed.deserialize(&mut *self.de).map(Some)
    }
}

// --- MapAccess ---

struct MapAccess<'a, 'de> {
    de: &'a mut CdrDeserializer<'de>,
    remaining: usize,
}

impl<'a, 'de> de::MapAccess<'de> for MapAccess<'a, 'de> {
    type Error = CdrError;

    fn next_key_seed<K: DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, CdrError> {
        if self.remaining == 0 {
            return Ok(None);
        }
        self.remaining -= 1;
        seed.deserialize(&mut *self.de).map(Some)
    }

    fn next_value_seed<V: DeserializeSeed<'de>>(&mut self, seed: V) -> Result<V::Value, CdrError> {
        seed.deserialize(&mut *self.de)
    }
}

// --- EnumAccess ---

struct EnumAccess<'a, 'de> {
    de: &'a mut CdrDeserializer<'de>,
}

impl<'a, 'de> de::EnumAccess<'de> for EnumAccess<'a, 'de> {
    type Error = CdrError;
    type Variant = Self;

    fn variant_seed<V: DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, Self::Variant), CdrError> {
        let idx = self.de.read_u32()?;
        let val = seed.deserialize(idx.into_deserializer())?;
        Ok((val, self))
    }
}

impl<'a, 'de> de::VariantAccess<'de> for EnumAccess<'a, 'de> {
    type Error = CdrError;

    fn unit_variant(self) -> Result<(), CdrError> {
        Ok(())
    }

    fn newtype_variant_seed<T: DeserializeSeed<'de>>(self, seed: T) -> Result<T::Value, CdrError> {
        seed.deserialize(self.de)
    }

    fn tuple_variant<V: Visitor<'de>>(self, len: usize, visitor: V) -> Result<V::Value, CdrError> {
        de::Deserializer::deserialize_tuple(self.de, len, visitor)
    }

    fn struct_variant<V: Visitor<'de>>(
        self,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, CdrError> {
        de::Deserializer::deserialize_struct(self.de, "", fields, visitor)
    }
}

/// Helper to convert u32 to a serde deserializer (for enum variant indexing).
#[allow(dead_code)]
struct U32Deserializer(u32);

impl From<u32> for U32Deserializer {
    fn from(v: u32) -> Self {
        Self(v)
    }
}

#[allow(dead_code)]
impl U32Deserializer {
    fn into_deserializer(self) -> Self {
        self
    }
}

impl<'de> de::Deserializer<'de> for U32Deserializer {
    type Error = CdrError;

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, CdrError> {
        visitor.visit_u32(self.0)
    }

    serde::forward_to_deserialize_any! {
        bool i8 i16 i32 i64 u8 u16 u32 u64 f32 f64 char str string bytes
        byte_buf option unit unit_struct newtype_struct seq tuple tuple_struct
        map struct enum identifier ignored_any
    }
}
