use super::*;

const RESERVATION_SLOT_COUNT: usize = 4;
const RESERVATION_SLOT_BITS: u32 = 8;
const RESERVATION_SLOT_MASK: u32 = (1 << RESERVATION_SLOT_BITS) - 1;

#[derive(Clone, Copy)]
struct ReservationEntry {
    generation: u16,
    remaining: u8,
    next_stack: u8,
    stack_count: u8,
    stack_size: usize,
    stacks: [usize; DYNAMIC_TASK_CAPACITY],
    active: bool,
}

impl ReservationEntry {
    const EMPTY: Self = Self {
        generation: 0,
        remaining: 0,
        next_stack: 0,
        stack_count: 0,
        stack_size: 0,
        stacks: [0; DYNAMIC_TASK_CAPACITY],
        active: false,
    };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct ReservedStack {
    pub(super) pointer: usize,
    pub(super) size: usize,
}

#[derive(Debug, Eq, PartialEq)]
pub(super) struct ReleasedTaskResources {
    pub(super) stacks: [usize; DYNAMIC_TASK_CAPACITY],
    pub(super) count: usize,
}

pub(super) struct ReservationTable {
    entries: [ReservationEntry; RESERVATION_SLOT_COUNT],
    total_remaining: usize,
}

impl ReservationTable {
    pub(super) const fn new() -> Self {
        Self {
            entries: [ReservationEntry::EMPTY; RESERVATION_SLOT_COUNT],
            total_remaining: 0,
        }
    }

    pub(super) const fn total_remaining(&self) -> usize {
        self.total_remaining
    }

    pub(super) fn reserve(
        &mut self,
        required: NonZeroUsize,
        available: usize,
    ) -> Result<TaskReservation, TaskAdmissionError> {
        if required.get() > available {
            return Err(TaskAdmissionError::InsufficientTaskSlots {
                required: required.get(),
                available,
            });
        }
        let slot = self
            .entries
            .iter()
            .position(|entry| !entry.active)
            .ok_or(TaskAdmissionError::Runtime(DriverError::ResourceExhausted))?;
        let remaining = u8::try_from(required.get())
            .map_err(|_| TaskAdmissionError::Runtime(DriverError::ResourceExhausted))?;
        let entry = &mut self.entries[slot];
        entry.generation = entry.generation.wrapping_add(1).max(1);
        entry.remaining = remaining;
        entry.next_stack = 0;
        entry.stack_count = 0;
        entry.stack_size = 0;
        entry.stacks = [0; DYNAMIC_TASK_CAPACITY];
        entry.active = true;
        self.total_remaining += required.get();

        let raw = (u32::from(entry.generation) << RESERVATION_SLOT_BITS)
            | u32::try_from(slot + 1).unwrap();
        let raw = NonZeroU32::new(raw).unwrap();
        // SAFETY: this table owns the active generation-bearing entry encoded
        // by `raw` until release invalidates it.
        Ok(unsafe { TaskReservation::from_raw(raw) })
    }

    pub(super) fn reserve_with_stacks(
        &mut self,
        required: TaskResourceRequirements,
        available: usize,
        stacks: [usize; DYNAMIC_TASK_CAPACITY],
    ) -> Result<TaskReservation, TaskAdmissionError> {
        let reservation = self.reserve(required.task_slots(), available)?;
        let slot = self
            .resolve_slot(&reservation)
            .map_err(TaskAdmissionError::Runtime)?;
        let entry = &mut self.entries[slot];
        entry.stack_count = u8::try_from(required.task_slots().get())
            .map_err(|_| TaskAdmissionError::Runtime(DriverError::ResourceExhausted))?;
        entry.stack_size = required.stack_bytes_per_task().get();
        entry.stacks = stacks;
        Ok(reservation)
    }

    pub(super) fn stack_size(
        &self,
        reservation: &TaskReservation,
    ) -> Result<Option<usize>, DriverError> {
        let entry = self.resolve(reservation)?;
        Ok((entry.stack_count != 0).then_some(entry.stack_size))
    }

    pub(super) fn ensure_consumable(
        &self,
        reservation: &TaskReservation,
    ) -> Result<(), DriverError> {
        let entry = self.resolve(reservation)?;
        if entry.remaining == 0 {
            Err(DriverError::NoTaskSlots)
        } else {
            Ok(())
        }
    }

    pub(super) fn consume(
        &mut self,
        reservation: &TaskReservation,
    ) -> Result<Option<ReservedStack>, DriverError> {
        let slot = self.resolve_slot(reservation)?;
        let entry = &mut self.entries[slot];
        if entry.remaining == 0 {
            return Err(DriverError::NoTaskSlots);
        }
        let stack = if entry.stack_count == 0 {
            None
        } else {
            let index = usize::from(entry.next_stack);
            let pointer = entry.stacks[index];
            debug_assert_ne!(pointer, 0);
            entry.stacks[index] = 0;
            entry.next_stack += 1;
            Some(ReservedStack {
                pointer,
                size: entry.stack_size,
            })
        };
        entry.remaining -= 1;
        self.total_remaining -= 1;
        Ok(stack)
    }

    pub(super) fn release(
        &mut self,
        reservation: &TaskReservation,
    ) -> Result<ReleasedTaskResources, DriverError> {
        let slot = self.resolve_slot(reservation)?;
        let entry = &mut self.entries[slot];
        let mut released = ReleasedTaskResources {
            stacks: [0; DYNAMIC_TASK_CAPACITY],
            count: 0,
        };
        for pointer in &entry.stacks[usize::from(entry.next_stack)..usize::from(entry.stack_count)]
        {
            if *pointer != 0 {
                released.stacks[released.count] = *pointer;
                released.count += 1;
            }
        }
        self.total_remaining -= usize::from(entry.remaining);
        entry.remaining = 0;
        entry.next_stack = 0;
        entry.stack_count = 0;
        entry.stack_size = 0;
        entry.stacks = [0; DYNAMIC_TASK_CAPACITY];
        entry.active = false;
        Ok(released)
    }

    fn resolve(&self, reservation: &TaskReservation) -> Result<&ReservationEntry, DriverError> {
        let slot = self.resolve_slot(reservation)?;
        Ok(&self.entries[slot])
    }

    fn resolve_slot(&self, reservation: &TaskReservation) -> Result<usize, DriverError> {
        let raw = reservation.into_raw().get();
        if raw >> (RESERVATION_SLOT_BITS + u16::BITS) != 0 {
            return Err(DriverError::InvalidHandle);
        }
        let encoded_slot = (raw & RESERVATION_SLOT_MASK) as usize;
        let generation = (raw >> RESERVATION_SLOT_BITS) as u16;
        if encoded_slot == 0 || generation == 0 {
            return Err(DriverError::InvalidHandle);
        }
        let slot = encoded_slot - 1;
        let Some(entry) = self.entries.get(slot) else {
            return Err(DriverError::InvalidHandle);
        };
        if !entry.active || entry.generation != generation {
            return Err(DriverError::InvalidHandle);
        }
        Ok(slot)
    }
}
