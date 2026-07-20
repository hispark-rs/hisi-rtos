use core::num::NonZeroUsize;

use hisi_rf_rtos_driver::Error as DriverError;

const HANDLE_SLOT_BITS: u32 = 8;
const HANDLE_SLOT_MASK: usize = (1usize << HANDLE_SLOT_BITS) - 1;
const HANDLE_GENERATION_MAX: usize = usize::MAX >> HANDLE_SLOT_BITS;

pub(super) const RESOURCE_HANDLE_CAPACITY: usize = 32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum ResourceKind {
    Semaphore,
    Mutex,
}

#[derive(Clone, Copy)]
struct ResourceSlot {
    pointer: usize,
    generation: usize,
    kind: ResourceKind,
}

impl ResourceSlot {
    const EMPTY: Self = Self {
        pointer: 0,
        generation: 0,
        kind: ResourceKind::Semaphore,
    };
}

pub(super) struct ResourceTable<const N: usize> {
    slots: [ResourceSlot; N],
}

impl<const N: usize> ResourceTable<N> {
    pub(super) const fn new() -> Self {
        assert!(N > 0 && N <= HANDLE_SLOT_MASK);
        Self {
            slots: [ResourceSlot::EMPTY; N],
        }
    }

    pub(super) fn insert(
        &mut self,
        pointer: NonZeroUsize,
        kind: ResourceKind,
    ) -> Result<NonZeroUsize, DriverError> {
        let (slot, entry) = self
            .slots
            .iter_mut()
            .enumerate()
            .find(|(_, entry)| entry.pointer == 0)
            .ok_or(DriverError::ResourceExhausted)?;
        entry.generation = next_generation(entry.generation);
        entry.pointer = pointer.get();
        entry.kind = kind;
        encode_handle(slot, entry.generation)
    }

    pub(super) fn resolve(
        &self,
        handle: NonZeroUsize,
        kind: ResourceKind,
    ) -> Result<NonZeroUsize, DriverError> {
        let (slot, generation) = decode_handle::<N>(handle)?;
        let entry = &self.slots[slot];
        if entry.pointer == 0 || entry.generation != generation || entry.kind != kind {
            return Err(DriverError::InvalidHandle);
        }
        NonZeroUsize::new(entry.pointer).ok_or(DriverError::InvalidHandle)
    }

    pub(super) fn remove(
        &mut self,
        handle: NonZeroUsize,
        kind: ResourceKind,
    ) -> Result<NonZeroUsize, DriverError> {
        let pointer = self.resolve(handle, kind)?;
        let (slot, _) = decode_handle::<N>(handle)?;
        self.slots[slot].pointer = 0;
        Ok(pointer)
    }
}

const fn next_generation(current: usize) -> usize {
    if current == 0 || current == HANDLE_GENERATION_MAX {
        1
    } else {
        current + 1
    }
}

fn encode_handle(slot: usize, generation: usize) -> Result<NonZeroUsize, DriverError> {
    if slot >= HANDLE_SLOT_MASK || generation == 0 || generation > HANDLE_GENERATION_MAX {
        return Err(DriverError::Runtime);
    }
    NonZeroUsize::new((generation << HANDLE_SLOT_BITS) | (slot + 1)).ok_or(DriverError::Runtime)
}

fn decode_handle<const N: usize>(handle: NonZeroUsize) -> Result<(usize, usize), DriverError> {
    let raw = handle.get();
    let encoded_slot = raw & HANDLE_SLOT_MASK;
    let generation = raw >> HANDLE_SLOT_BITS;
    if encoded_slot == 0 || encoded_slot > N || generation == 0 {
        return Err(DriverError::InvalidHandle);
    }
    Ok((encoded_slot - 1, generation))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duplicate_and_stale_handles_fail_before_pointer_resolution() {
        let mut table = ResourceTable::<2>::new();
        let first_pointer = NonZeroUsize::new(0x1000).unwrap();
        let first = table
            .insert(first_pointer, ResourceKind::Semaphore)
            .unwrap();
        assert_eq!(
            table.remove(first, ResourceKind::Semaphore),
            Ok(first_pointer)
        );
        assert_eq!(
            table.remove(first, ResourceKind::Semaphore),
            Err(DriverError::InvalidHandle)
        );

        let replacement_pointer = NonZeroUsize::new(0x2000).unwrap();
        let replacement = table
            .insert(replacement_pointer, ResourceKind::Semaphore)
            .unwrap();
        assert_ne!(first, replacement);
        assert_eq!(
            table.resolve(first, ResourceKind::Semaphore),
            Err(DriverError::InvalidHandle)
        );
        assert_eq!(
            table.resolve(replacement, ResourceKind::Semaphore),
            Ok(replacement_pointer)
        );
    }

    #[test]
    fn handle_kind_is_part_of_the_identity() {
        let mut table = ResourceTable::<1>::new();
        let handle = table
            .insert(NonZeroUsize::new(0x1000).unwrap(), ResourceKind::Mutex)
            .unwrap();
        assert_eq!(
            table.resolve(handle, ResourceKind::Semaphore),
            Err(DriverError::InvalidHandle)
        );
    }
}
