use core::ops::{Deref, DerefMut};
use aligned::{Aligned, A8};

use crate::MTU;

pub trait RingDescriptor {
    fn setup(&mut self, buffer: *const u8, len: usize, next: Option<&Self>);
}

pub struct RingEntry<T: Clone + RingDescriptor> {
    desc: Aligned<A8, [T; 1]>,
    buffer: Aligned<A8, [u8; MTU]>,
}

impl<T: Clone + RingDescriptor> Clone for RingEntry<T> {
    fn clone(&self) -> Self {
        RingEntry {
            desc: Aligned((*self.desc).clone()),
            buffer: Aligned((*self.buffer).clone()),
        }
    }
}

impl<T: Clone + RingDescriptor + Default> Default for RingEntry<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone + RingDescriptor + Default> RingEntry<T> {
    pub fn new() -> Self {
        RingEntry {
            desc: Aligned([T::default()]),
            buffer: Aligned([0; MTU]),
        }
    }

    pub(crate) fn setup(&mut self, next: Option<&Self>) {
        let buffer = self.buffer.as_ptr();
        let len = self.buffer.len();
        self.desc_mut().setup(
            buffer, len,
            next.map(|next| next.desc())
        );
    }

    #[inline]
    pub(crate) fn desc(&self) -> &T {
        &self.desc.deref()[0]
    }

    #[inline]
    pub(crate) fn desc_mut(&mut self) -> &mut T {
        &mut self.desc.deref_mut()[0]
    }

    #[inline]
    pub(crate) fn as_slice(&self) -> &[u8] {
        &(*self.buffer)[..]
    }

    #[inline]
    pub(crate) fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut (*self.buffer)[..]
    }
}

// pub struct Ring<'a, T: 'a> {
//     entries: &'a mut [RingEntry<T>],
//     next_entry: usize,
// }

// impl<'a, T: 'a> Ring<'a, T> {
//     /// Allocate
//     pub fn new(entries: &'a mut [RingEntry<T>]) -> Self {
//         Ring {
//             entries,
//             next_entry: 0,
//         }
//     }
// }
