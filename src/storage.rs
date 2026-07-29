//! Caller-owned storage for dynamically created task stacks.

use core::cell::Cell;
use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::mem::MaybeUninit;

use critical_section::Mutex;
use hisi_alloc::{CHeap, HeapMetrics};

use crate::DYNAMIC_TASK_CAPACITY;

/// Failure to install caller-owned scheduler storage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StorageError {
    /// The storage object was already consumed by a runtime.
    AlreadyInstalled,
    /// The supplied arena cannot back the task-stack allocator.
    InvalidArena,
    /// The requested dynamic task capacity exceeds this runtime build.
    UnsupportedCapacity {
        /// Capacity requested through `SchedulerStorage<N>`.
        requested: usize,
        /// Largest capacity supported by this runtime build.
        maximum: usize,
    },
}

/// Statically placed bytes used exclusively for dynamic task stacks.
#[repr(C, align(16))]
pub struct SchedulerStackArena<const BYTES: usize> {
    bytes: UnsafeCell<[MaybeUninit<u8>; BYTES]>,
}

// SAFETY: the bytes are reachable only through one-shot SchedulerStorage
// installation, which transfers exclusive process-lifetime ownership to CHeap.
unsafe impl<const BYTES: usize> Sync for SchedulerStackArena<BYTES> {}

impl<const BYTES: usize> SchedulerStackArena<BYTES> {
    /// Construct an unclaimed stack arena.
    pub const fn new() -> Self {
        Self {
            bytes: UnsafeCell::new([MaybeUninit::uninit(); BYTES]),
        }
    }
}

impl<const BYTES: usize> Default for SchedulerStackArena<BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

/// Caller-owned task-stack storage for at most `N` dynamic tasks.
///
/// `N` controls the scheduler's dynamic task quota. Stack bytes are supplied
/// separately to [`Self::install`], so variable vendor stack requests remain
/// supported and the application can place the arena in an appropriate memory
/// region.
pub struct SchedulerStorage<const N: usize> {
    heap: CHeap,
    installed: Mutex<Cell<bool>>,
}

impl<const N: usize> SchedulerStorage<N> {
    /// Construct uninstalled scheduler storage.
    pub const fn new() -> Self {
        Self {
            heap: CHeap::empty(),
            installed: Mutex::new(Cell::new(false)),
        }
    }

    /// Install a process-lifetime task-stack arena.
    ///
    /// The arena's one safe ownership path is consumed for the firmware
    /// lifetime. After this succeeds, its bytes are accessed only through the
    /// returned capability and the RTOS.
    pub fn install<const BYTES: usize>(
        &'static self,
        arena: &'static SchedulerStackArena<BYTES>,
    ) -> Result<InstalledSchedulerStorage<N>, StorageError> {
        if N == 0 || N > DYNAMIC_TASK_CAPACITY {
            return Err(StorageError::UnsupportedCapacity {
                requested: N,
                maximum: DYNAMIC_TASK_CAPACITY,
            });
        }
        let already_installed = critical_section::with(|cs| {
            let installed = self.installed.borrow(cs);
            installed.replace(true)
        });
        if already_installed {
            return Err(StorageError::AlreadyInstalled);
        }

        // SAFETY: the exclusive static borrow is consumed by this one-shot
        // installation and CHeap serializes every subsequent access.
        if unsafe { self.heap.init(arena.bytes.get().cast::<u8>(), BYTES) }.is_err() {
            critical_section::with(|cs| self.installed.borrow(cs).set(false));
            return Err(StorageError::InvalidArena);
        }

        Ok(InstalledSchedulerStorage {
            erased: ErasedSchedulerStorage {
                context: &self.heap as *const CHeap as usize,
                allocate: allocate::<N>,
                deallocate: deallocate::<N>,
                metrics: metrics::<N>,
                dynamic_capacity: N,
            },
            _capacity: PhantomData,
        })
    }

    /// Snapshot task-stack arena usage.
    pub fn metrics(&self) -> HeapMetrics {
        self.heap.metrics()
    }
}

impl<const N: usize> Default for SchedulerStorage<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// One-shot proof that caller-owned scheduler storage was installed.
#[must_use = "pass installed scheduler storage to a runtime start function"]
pub struct InstalledSchedulerStorage<const N: usize> {
    pub(crate) erased: ErasedSchedulerStorage,
    _capacity: PhantomData<[(); N]>,
}

impl<const N: usize> InstalledSchedulerStorage<N> {
    /// Dynamic task capacity admitted by this storage.
    pub const fn dynamic_capacity(&self) -> usize {
        N
    }

    /// Snapshot task-stack arena usage.
    pub fn metrics(&self) -> HeapMetrics {
        // SAFETY: installation binds context and function pointers to the same
        // process-lifetime CHeap.
        unsafe { (self.erased.metrics)(self.erased.context) }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct ErasedSchedulerStorage {
    context: usize,
    allocate: unsafe fn(usize, usize) -> *mut u8,
    deallocate: unsafe fn(usize, *mut u8),
    metrics: unsafe fn(usize) -> HeapMetrics,
    pub(crate) dynamic_capacity: usize,
}

impl ErasedSchedulerStorage {
    pub(crate) fn allocate(self, size: usize) -> *mut u8 {
        // SAFETY: only `SchedulerStorage::install` constructs this erased
        // capability, pairing the context with its monomorphized operations.
        unsafe { (self.allocate)(self.context, size) }
    }

    pub(crate) unsafe fn deallocate(self, pointer: *mut u8) {
        // SAFETY: the caller returns a live allocation obtained from this same
        // erased storage capability.
        unsafe { (self.deallocate)(self.context, pointer) };
    }
}

unsafe fn allocate<const N: usize>(context: usize, size: usize) -> *mut u8 {
    // SAFETY: `context` is created from a process-lifetime CHeap in install.
    let heap = unsafe { &*(context as *const CHeap) };
    heap.allocate_zeroed(size, 16)
}

unsafe fn deallocate<const N: usize>(context: usize, pointer: *mut u8) {
    // SAFETY: `context` is created from a process-lifetime CHeap in install.
    let heap = unsafe { &*(context as *const CHeap) };
    // The scheduler only retires pointers allocated by this heap, exactly once.
    let result = unsafe { heap.deallocate(pointer) };
    debug_assert!(result.is_ok());
}

unsafe fn metrics<const N: usize>(context: usize) -> HeapMetrics {
    // SAFETY: `context` is created from a process-lifetime CHeap in install.
    unsafe { &*(context as *const CHeap) }.metrics()
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::boxed::Box;

    use super::*;

    #[test]
    fn caller_owned_arena_is_installed_once_and_reports_usage() {
        let storage = Box::leak(Box::new(SchedulerStorage::<3>::new()));
        let arena = Box::leak(Box::new(SchedulerStackArena::<4096>::new()));
        let installed = storage.install(arena).unwrap();

        assert_eq!(installed.dynamic_capacity(), 3);
        assert!(installed.metrics().initialized);
        assert!(matches!(
            storage.install(Box::leak(Box::new(SchedulerStackArena::<4096>::new()))),
            Err(StorageError::AlreadyInstalled)
        ));
    }

    #[test]
    fn unsupported_capacity_fails_before_claiming_the_arena() {
        let storage = Box::leak(Box::new(SchedulerStorage::<0>::new()));
        let arena = Box::leak(Box::new(SchedulerStackArena::<4096>::new()));
        assert!(matches!(
            storage.install(arena),
            Err(StorageError::UnsupportedCapacity {
                requested: 0,
                maximum: DYNAMIC_TASK_CAPACITY,
            })
        ));
    }
}
