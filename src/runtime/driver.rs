use super::*;

fn resource_pointer(handle: NonZeroUsize, kind: ResourceKind) -> Result<NonZeroUsize, DriverError> {
    critical_section::with(|cs| RESOURCE_HANDLES.borrow_ref(cs).resolve(handle, kind))
}

fn semaphore_from_handle(handle: SemaphoreHandle) -> Result<&'static Semaphore, DriverError> {
    let pointer = resource_pointer(handle.into_raw(), ResourceKind::Semaphore)?;
    // SAFETY: the validated registry entry owns a live Semaphore allocation.
    // The unsafe destroy contract excludes concurrent destruction while a safe
    // operation is using this reference.
    Ok(unsafe { &*(pointer.get() as *const Semaphore) })
}

fn mutex_from_handle(handle: MutexHandle) -> Result<&'static RtosMutex, DriverError> {
    let pointer = resource_pointer(handle.into_raw(), ResourceKind::Mutex)?;
    // SAFETY: the validated registry entry owns a live RtosMutex allocation and
    // the unsafe destroy contract excludes concurrent destruction.
    Ok(unsafe { &*(pointer.get() as *const RtosMutex) })
}

pub(super) fn preallocate_task_stacks(
    required: TaskResourceRequirements,
    mut allocate_stack: impl FnMut(usize) -> *mut u8,
    mut release_stack: impl FnMut(*mut u8),
) -> Result<[usize; DYNAMIC_TASK_CAPACITY], TaskAdmissionError> {
    let mut stacks = [0usize; DYNAMIC_TASK_CAPACITY];
    let stack_size = required.stack_bytes_per_task().get();
    for (allocated, pointer) in stacks
        .iter_mut()
        .take(required.task_slots().get())
        .enumerate()
    {
        let stack = allocate_stack(stack_size);
        if stack.is_null() {
            for stack in &stacks[..allocated] {
                release_stack(*stack as *mut u8);
            }
            return Err(TaskAdmissionError::InsufficientTaskStackMemory {
                required: required.total_stack_bytes(),
                available: allocated * stack_size,
            });
        }
        *pointer = stack as usize;
    }
    Ok(stacks)
}

pub(super) fn semaphore_state_has_waiters(state: &SemState) -> bool {
    state.wait_head != NIL || state.wait_tail != NIL
}

pub(super) fn mutex_state_is_busy(state: &MutexState) -> bool {
    state.owner != NIL || state.wait_head != NIL || state.wait_tail != NIL
}

impl Runtime for HisiRuntime {
    fn contract(&self) -> RuntimeContract {
        RuntimeContract::V1_4
    }

    fn execution_profile(&self) -> RuntimeExecutionProfile {
        if start_state().port.is_some() {
            RuntimeExecutionProfile::V1_PORTED
        } else {
            RuntimeExecutionProfile::V1_PORTLESS_COOPERATIVE
        }
    }

    fn task_capacity(&self) -> Result<TaskCapacity, DriverError> {
        let capacity = dynamic_capacity();
        let diagnostics =
            critical_section::with(|cs| SCHED.borrow_ref(cs).diagnostics_with_capacity(capacity));
        TaskCapacity::new_with_reserved(
            usize::from(diagnostics.dynamic_capacity),
            usize::from(diagnostics.dynamic_used),
            usize::from(diagnostics.dynamic_reserved),
        )
        .ok_or(DriverError::Runtime)
    }

    fn reserve_tasks(&self, required: NonZeroUsize) -> Result<TaskReservation, TaskAdmissionError> {
        let capacity = dynamic_capacity();
        critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .reserve_dynamic_slots_with_capacity(required, capacity)
        })
    }

    fn reserve_task_resources(
        &self,
        required: TaskResourceRequirements,
    ) -> Result<TaskReservation, TaskAdmissionError> {
        let stacks = preallocate_task_stacks(required, allocate, deallocate)?;
        let allocated = required.task_slots().get().min(DYNAMIC_TASK_CAPACITY);
        let capacity = dynamic_capacity();
        let result = critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .reserve_dynamic_task_resources_with_capacity(required, stacks, capacity)
        });
        if result.is_err() {
            for stack in &stacks[..allocated] {
                deallocate(*stack as *mut u8);
            }
        }
        result
    }

    fn release_task_reservation(&self, reservation: &TaskReservation) -> Result<(), DriverError> {
        let released = critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .release_task_reservation(reservation)
        })?;
        for stack in &released.stacks[..released.count] {
            deallocate(*stack as *mut u8);
        }
        Ok(())
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
            None,
        )?;
        let generation =
            critical_section::with(|cs| SCHED.borrow_ref(cs).tasks[slot].identity_generation);
        encode_task_id(slot, generation)
    }

    fn spawn_reserved(
        &self,
        reservation: &TaskReservation,
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
            Some(reservation),
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
        current_task_handle()
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

    fn cancel_wait(&self, task: TaskId) -> Result<WaitCancellationOutcome, DriverError> {
        let (slot, generation) = decode_task_id(task)?;
        let machine_interrupts_enabled = machine_interrupts_enabled();
        let now = now_ms();
        let mut defer_reschedule = false;
        let (outcome, preemption) = critical_section::with(|cs| {
            if INTERRUPT_DEPTH.borrow(cs).get() != 0 {
                return Err(DriverError::InvalidContext);
            }
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let Some(tcb) = scheduler.tasks.get(slot) else {
                return Err(DriverError::InvalidHandle);
            };
            if tcb.state == State::Free || tcb.identity_generation != generation {
                return Err(DriverError::InvalidHandle);
            }
            let outcome = cancel_wait_locked(scheduler, slot, now);
            let preemption =
                if outcome == WaitCancellationOutcome::Cancelled && machine_interrupts_enabled {
                    scheduler
                        .take_preemption_target(now)
                        .map(|(current, next)| scheduler.prepare_switch_intent(current, next))
                } else {
                    defer_reschedule = outcome == WaitCancellationOutcome::Cancelled
                        && !machine_interrupts_enabled;
                    None
                };
            Ok((outcome, preemption))
        })?;
        if let Some(intent) = preemption {
            execute_switch(intent);
        }
        if defer_reschedule {
            request_reschedule();
        }
        rearm_timer();
        Ok(outcome)
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
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            scheduler
                .unlock_current_and_take_preemption(now)
                .map(|target| {
                    target.map(|(current, next)| scheduler.prepare_switch_intent(current, next))
                })
        })?;
        if let Some(intent) = preemption {
            execute_switch(intent);
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
        let handle = critical_section::with(|cs| {
            RESOURCE_HANDLES
                .borrow_ref_mut(cs)
                .insert(raw, ResourceKind::Semaphore)
        });
        let handle = match handle {
            Ok(handle) => handle,
            Err(error) => {
                deallocate(pointer.cast());
                return Err(error);
            }
        };
        // SAFETY: `raw` identifies this live allocation until destroy.
        Ok(unsafe { SemaphoreHandle::from_raw(handle) })
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
            if semaphore_from_handle(semaphore)?.down_timeout(timeout_ms)? {
                WaitOutcome::Acquired
            } else {
                WaitOutcome::TimedOut
            },
        )
    }

    fn semaphore_up(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        semaphore_from_handle(semaphore)?.up()
    }

    unsafe fn semaphore_destroy(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        let pointer = critical_section::with(|cs| {
            let handles = &mut *RESOURCE_HANDLES.borrow_ref_mut(cs);
            let pointer = handles.resolve(semaphore.into_raw(), ResourceKind::Semaphore)?;
            // SAFETY: the registry entry was validated and queue access is
            // serialized by this scheduler critical section.
            let state = unsafe { &*(pointer.get() as *const Semaphore) }.inner.get();
            // SAFETY: the critical section provides exclusive state access.
            let scheduler = &*SCHED.borrow_ref(cs);
            let pending_grant = scheduler
                .tasks
                .iter()
                .any(|task| task.granted_sem == pointer.get());
            if semaphore_state_has_waiters(unsafe { &*state }) || pending_grant {
                return Err(DriverError::InvalidContext);
            }
            handles.remove(semaphore.into_raw(), ResourceKind::Semaphore)
        })?;
        deallocate(pointer.get() as *mut u8);
        Ok(())
    }

    fn mutex_create(&self) -> Result<MutexHandle, DriverError> {
        let pointer = allocate(core::mem::size_of::<RtosMutex>()) as *mut RtosMutex;
        let raw = NonZeroUsize::new(pointer as usize).ok_or(DriverError::ResourceExhausted)?;
        // SAFETY: the RF allocator guarantees size/alignment for RtosMutex.
        unsafe { pointer.write(RtosMutex::new()) };
        let handle = critical_section::with(|cs| {
            RESOURCE_HANDLES
                .borrow_ref_mut(cs)
                .insert(raw, ResourceKind::Mutex)
        });
        let handle = match handle {
            Ok(handle) => handle,
            Err(error) => {
                deallocate(pointer.cast());
                return Err(error);
            }
        };
        // SAFETY: raw identifies the live allocation until destroy.
        Ok(unsafe { MutexHandle::from_raw(handle) })
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
        Ok(if mutex_from_handle(mutex)?.lock(timeout_ms)? {
            WaitOutcome::Acquired
        } else {
            WaitOutcome::TimedOut
        })
    }

    fn mutex_unlock(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        mutex_from_handle(mutex)?.unlock()
    }

    unsafe fn mutex_destroy(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        let pointer = critical_section::with(|cs| {
            let handles = &mut *RESOURCE_HANDLES.borrow_ref_mut(cs);
            let pointer = handles.resolve(mutex.into_raw(), ResourceKind::Mutex)?;
            // SAFETY: the registry entry was validated and state access is
            // serialized by this scheduler critical section.
            let state = unsafe { &*(pointer.get() as *const RtosMutex) }.inner.get();
            // SAFETY: the critical section provides exclusive state access.
            if mutex_state_is_busy(unsafe { &*state }) {
                return Err(DriverError::InvalidContext);
            }
            handles.remove(mutex.into_raw(), ResourceKind::Mutex)
        })?;
        deallocate(pointer.get() as *mut u8);
        Ok(())
    }
}
