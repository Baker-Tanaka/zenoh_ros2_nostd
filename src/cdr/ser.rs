//! CDR Little Endian Serializer (serde-based, no_std).
//!
//! Implements the OMG CDR encoding with automatic alignment padding.

use super::CdrError;
use serde::ser::{self, Serialize};

/// CDR Little Endian serializer writing into a `&mut [u8]` buffer.
pub struct CdrSerializer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> CdrSerializer<'a> {
    /// Create a new serializer writing into `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Current write position (number of bytes written so far).
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Align the write cursor to an `n`-byte boundary with zero padding.
    fn align(&mut self, n: usize) -> Result<(), CdrError> {
        let rem = self.pos % n;
        if rem != 0 {
            let padding = n - rem;
            if self.pos + padding > self.buf.len() {
                return Err(CdrError::BufferOverflow);
            }
            for i in 0..padding {
                self.buf[self.pos + i] = 0;
            }
            self.pos += padding;
        }
        Ok(())
    }

    /// Write raw bytes at the current position.
    fn write_bytes(&mut self, data: &[u8]) -> Result<(), CdrError> {
        if self.pos + data.len() > self.buf.len() {
            return Err(CdrError::BufferOverflow);
        }
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Ok(())
    }
}

impl<'a, 'b> ser::Serializer for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;

    fn collect_str<V: core::fmt::Display + ?Sized>(self, value: &V) -> Result<(), CdrError> {
        use core::fmt::Write;
        // Serialize Display types as CDR strings using a small stack buffer
        let mut buf = heapless::String::<256>::new();
        write!(buf, "{}", value).map_err(|_| CdrError::SerializeError)?;
        self.serialize_str(&buf)
    }
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_bool(self, v: bool) -> Result<(), CdrError> {
        self.write_bytes(&[v as u8])
    }

    fn serialize_i8(self, v: i8) -> Result<(), CdrError> {
        self.write_bytes(&[v as u8])
    }

    fn serialize_u8(self, v: u8) -> Result<(), CdrError> {
        self.write_bytes(&[v])
    }

    fn serialize_i16(self, v: i16) -> Result<(), CdrError> {
        self.align(2)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_u16(self, v: u16) -> Result<(), CdrError> {
        self.align(2)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_i32(self, v: i32) -> Result<(), CdrError> {
        self.align(4)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_u32(self, v: u32) -> Result<(), CdrError> {
        self.align(4)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_i64(self, v: i64) -> Result<(), CdrError> {
        self.align(8)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_u64(self, v: u64) -> Result<(), CdrError> {
        self.align(8)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_f32(self, v: f32) -> Result<(), CdrError> {
        self.align(4)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_f64(self, v: f64) -> Result<(), CdrError> {
        self.align(8)?;
        self.write_bytes(&v.to_le_bytes())
    }

    fn serialize_char(self, v: char) -> Result<(), CdrError> {
        // CDR char is a single octet.
        self.write_bytes(&[v as u8])
    }

    fn serialize_str(self, v: &str) -> Result<(), CdrError> {
        // CDR string: u32 length (including null terminator) + bytes + null
        let len_with_null = (v.len() + 1) as u32;
        self.align(4)?;
        self.write_bytes(&len_with_null.to_le_bytes())?;
        self.write_bytes(v.as_bytes())?;
        self.write_bytes(&[0]) // null terminator
    }

    fn serialize_bytes(self, v: &[u8]) -> Result<(), CdrError> {
        // CDR octet sequence: u32 length + bytes
        let len = v.len() as u32;
        self.align(4)?;
        self.write_bytes(&len.to_le_bytes())?;
        self.write_bytes(v)
    }

    fn serialize_none(self) -> Result<(), CdrError> {
        self.serialize_bool(false)
    }

    fn serialize_some<T: ?Sized + Serialize>(self, value: &T) -> Result<(), CdrError> {
        self.serialize_bool(true)?;
        value.serialize(self)
    }

    fn serialize_unit(self) -> Result<(), CdrError> {
        Ok(())
    }

    fn serialize_unit_struct(self, _name: &'static str) -> Result<(), CdrError> {
        Ok(())
    }

    fn serialize_unit_variant(
        self,
        _name: &'static str,
        variant_index: u32,
        _variant: &'static str,
    ) -> Result<(), CdrError> {
        self.serialize_u32(variant_index)
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        value: &T,
    ) -> Result<(), CdrError> {
        value.serialize(self)
    }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self,
        _name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        value: &T,
    ) -> Result<(), CdrError> {
        self.serialize_u32(variant_index)?;
        value.serialize(self)
    }

    fn serialize_seq(self, len: Option<usize>) -> Result<Self::SerializeSeq, CdrError> {
        // CDR variable-length sequence: u32 count prefix
        let len = len.ok_or(CdrError::SerializeError)?;
        self.align(4)?;
        self.write_bytes(&(len as u32).to_le_bytes())?;
        Ok(self)
    }

    fn serialize_tuple(self, _len: usize) -> Result<Self::SerializeTuple, CdrError> {
        // CDR fixed-size array: no length prefix
        Ok(self)
    }

    fn serialize_tuple_struct(
        self,
        _name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleStruct, CdrError> {
        Ok(self)
    }

    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleVariant, CdrError> {
        self.serialize_u32(variant_index)?;
        Ok(self)
    }

    fn serialize_map(self, len: Option<usize>) -> Result<Self::SerializeMap, CdrError> {
        let len = len.ok_or(CdrError::SerializeError)?;
        self.align(4)?;
        self.write_bytes(&(len as u32).to_le_bytes())?;
        Ok(self)
    }

    fn serialize_struct(
        self,
        _name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStruct, CdrError> {
        Ok(self)
    }

    fn serialize_struct_variant(
        self,
        _name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStructVariant, CdrError> {
        self.serialize_u32(variant_index)?;
        Ok(self)
    }
}

// --- Compound serializer trait impls ---

impl<'a, 'b> ser::SerializeSeq for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeTuple for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_element<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeTupleStruct for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeTupleVariant for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_field<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeMap for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_key<T: ?Sized + Serialize>(&mut self, key: &T) -> Result<(), CdrError> {
        key.serialize(&mut **self)
    }

    fn serialize_value<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeStruct for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        _key: &'static str,
        value: &T,
    ) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}

impl<'a, 'b> ser::SerializeStructVariant for &'b mut CdrSerializer<'a> {
    type Ok = ();
    type Error = CdrError;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        _key: &'static str,
        value: &T,
    ) -> Result<(), CdrError> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), CdrError> {
        Ok(())
    }
}
