use super::*;

const RESERVATION_SLOT_COUNT: usize = 4;
const RESERVATION_SLOT_BITS: u32 = 8;
const RESERVATION_SLOT_MASK: u32 = (1 << RESERVATION_SLOT_BITS) - 1;

#[derive(Clone, Copy)]
struct ReservationEntry {
    generation: u16,
    remaining: u8,
    active: bool,
}

impl ReservationEntry {
    const EMPTY: Self = Self {
        generation: 0,
        remaining: 0,
        active: false,
    };
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
        entry.active = true;
        self.total_remaining += required.get();

        let raw = (u32::from(entry.generation) << RESERVATION_SLOT_BITS)
            | u32::try_from(slot + 1).unwrap();
        let raw = NonZeroU32::new(raw).unwrap();
        // SAFETY: this table owns the active generation-bearing entry encoded
        // by `raw` until release invalidates it.
        Ok(unsafe { TaskReservation::from_raw(raw) })
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

    pub(super) fn consume(&mut self, reservation: &TaskReservation) -> Result<(), DriverError> {
        let slot = self.resolve_slot(reservation)?;
        let entry = &mut self.entries[slot];
        if entry.remaining == 0 {
            return Err(DriverError::NoTaskSlots);
        }
        entry.remaining -= 1;
        self.total_remaining -= 1;
        Ok(())
    }

    pub(super) fn release(&mut self, reservation: &TaskReservation) -> Result<(), DriverError> {
        let slot = self.resolve_slot(reservation)?;
        let entry = &mut self.entries[slot];
        self.total_remaining -= usize::from(entry.remaining);
        entry.remaining = 0;
        entry.active = false;
        Ok(())
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
