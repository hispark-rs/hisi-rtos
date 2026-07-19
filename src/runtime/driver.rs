use super::*;

fn semaphore_from_handle(handle: SemaphoreHandle) -> &'static Semaphore {
    let pointer = handle.into_raw().get() as *const Semaphore;
    // SAFETY: this backend creates handles only from heap-allocated Semaphore
    // objects and the driver contract requires users to stop all access before
    // destroy.
    unsafe { &*pointer }
}

fn mutex_from_handle(handle: MutexHandle) -> &'static RtosMutex {
    let pointer = handle.into_raw().get() as *const RtosMutex;
    // SAFETY: this backend creates handles only from live RtosMutex allocations.
    unsafe { &*pointer }
}

pub(super) fn semaphore_state_has_waiters(state: &SemState) -> bool {
    state.wait_head != NIL || state.wait_tail != NIL
}

fn semaphore_has_waiters(semaphore: &Semaphore) -> bool {
    critical_section::with(|_| {
        // SAFETY: all semaphore state is serialized by the scheduler critical
        // section on this single-hart runtime.
        let state = unsafe { &*semaphore.inner.get() };
        semaphore_state_has_waiters(state)
    })
}

impl Runtime for HisiRuntime {
    fn contract(&self) -> RuntimeContract {
        RuntimeContract::V1
    }

    fn execution_profile(&self) -> RuntimeExecutionProfile {
        if start_state().port.is_some() {
            RuntimeExecutionProfile::V1_PORTED
        } else {
            RuntimeExecutionProfile::V1_PORTLESS_COOPERATIVE
        }
    }

    fn spawn(
        &self,
        entry: hisi_rf_rtos_driver::TaskEntry,
        arg: *mut c_void,
        config: TaskConfig,
    ) -> Result<TaskId, DriverError> {
        let priority = config.priority.into_raw();
        let slot = spawn(
            entry,
            arg,
            config.stack_size.get(),
            priority,
            start_state().config.radio_task_policy,
        )?;
        let generation =
            critical_section::with(|cs| SCHED.borrow_ref(cs).tasks[slot].identity_generation);
        encode_task_id(slot, generation)
    }

    fn yield_now(&self) -> Result<(), DriverError> {
        yield_now()
    }

    fn sleep_ms(&self, milliseconds: NonZeroU32) -> Result<(), DriverError> {
        sleep_ms(milliseconds.get())
    }

    fn current_task(&self) -> Result<TaskId, DriverError> {
        let slot = current_id();
        let generation =
            critical_section::with(|cs| SCHED.borrow_ref(cs).tasks[slot].identity_generation);
        encode_task_id(slot, generation)
    }

    fn set_task_priority(&self, task: TaskId, priority: TaskPriority) -> Result<(), DriverError> {
        let priority = priority.into_raw();
        let (slot, generation) = decode_task_id(task)?;
        critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let Some(tcb) = scheduler.tasks.get(slot) else {
                return Err(DriverError::InvalidHandle);
            };
            if tcb.state == State::Free || tcb.identity_generation != generation {
                return Err(DriverError::InvalidHandle);
            }
            scheduler.tasks[slot].base_priority = priority;
            scheduler.refresh_inherited_priority(slot, 0);
            Ok(())
        })
    }

    fn lock_scheduler(&self) -> Result<(), DriverError> {
        ensure_switch_delivery()?;
        let now = now_ms();
        let result = critical_section::with(|cs| SCHED.borrow_ref_mut(cs).lock_current(now));
        if result.is_ok() {
            rearm_timer();
        }
        result
    }

    fn unlock_scheduler(&self) -> Result<(), DriverError> {
        ensure_switch_delivery()?;
        let now = now_ms();
        let preemption = critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .unlock_current_and_take_preemption(now)
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        rearm_timer();
        Ok(())
    }

    fn interrupt_enter(&self) -> Result<(), DriverError> {
        crate::interrupt_enter();
        Ok(())
    }

    fn interrupt_exit(&self) -> Result<(), DriverError> {
        crate::interrupt_exit();
        Ok(())
    }

    fn semaphore_create(&self, initial: u32) -> Result<SemaphoreHandle, DriverError> {
        let count = i32::try_from(initial).map_err(|_| DriverError::Runtime)?;
        let pointer = allocate(core::mem::size_of::<Semaphore>()) as *mut Semaphore;
        let raw = NonZeroUsize::new(pointer as usize).ok_or(DriverError::ResourceExhausted)?;
        // SAFETY: the RF allocator guarantees size/alignment for Semaphore.
        unsafe { pointer.write(Semaphore::new(count)) };
        // SAFETY: `raw` identifies this live allocation until destroy.
        Ok(unsafe { SemaphoreHandle::from_raw(raw) })
    }

    fn semaphore_down(
        &self,
        semaphore: SemaphoreHandle,
        timeout: WaitTimeout,
    ) -> Result<WaitOutcome, DriverError> {
        let timeout_ms = match timeout {
            WaitTimeout::NoWait => 0,
            WaitTimeout::Milliseconds(value) => value.get(),
            WaitTimeout::Forever => u32::MAX,
        };
        Ok(
            if semaphore_from_handle(semaphore).down_timeout(timeout_ms)? {
                WaitOutcome::Acquired
            } else {
                WaitOutcome::TimedOut
            },
        )
    }

    fn semaphore_up(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        semaphore_from_handle(semaphore).up()
    }

    unsafe fn semaphore_destroy(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        if semaphore_has_waiters(semaphore_from_handle(semaphore)) {
            return Err(DriverError::InvalidContext);
        }
        deallocate(semaphore.into_raw().get() as *mut u8);
        Ok(())
    }

    fn mutex_create(&self) -> Result<MutexHandle, DriverError> {
        let pointer = allocate(core::mem::size_of::<RtosMutex>()) as *mut RtosMutex;
        let raw = NonZeroUsize::new(pointer as usize).ok_or(DriverError::ResourceExhausted)?;
        // SAFETY: the RF allocator guarantees size/alignment for RtosMutex.
        unsafe { pointer.write(RtosMutex::new()) };
        // SAFETY: raw identifies the live allocation until destroy.
        Ok(unsafe { MutexHandle::from_raw(raw) })
    }

    fn mutex_lock(
        &self,
        mutex: MutexHandle,
        timeout: WaitTimeout,
    ) -> Result<WaitOutcome, DriverError> {
        let timeout_ms = match timeout {
            WaitTimeout::NoWait => 0,
            WaitTimeout::Milliseconds(value) => value.get(),
            WaitTimeout::Forever => u32::MAX,
        };
        Ok(if mutex_from_handle(mutex).lock(timeout_ms)? {
            WaitOutcome::Acquired
        } else {
            WaitOutcome::TimedOut
        })
    }

    fn mutex_unlock(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        mutex_from_handle(mutex).unlock()
    }

    unsafe fn mutex_destroy(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        let busy = critical_section::with(|_| {
            // SAFETY: caller promises the handle is live during this check.
            let state = unsafe { &*mutex_from_handle(mutex).inner.get() };
            state.owner != NIL || state.wait_head != NIL
        });
        if busy {
            return Err(DriverError::InvalidContext);
        }
        deallocate(mutex.into_raw().get() as *mut u8);
        Ok(())
    }
}
