use super::*;

// ── Counting semaphore (blocks via the scheduler) ───────────────────────────

/// A counting semaphore. Tasks block in [`Semaphore::down`] when the count is 0
/// and are woken by [`Semaphore::up`]. Backs `osal_sem_*` / `osal_wait_*` /
/// `osal_mutex_*`.
///
/// `&self` methods + interior mutability so it can be a `static` or heap object
/// shared across tasks; all state is touched only inside the scheduler critical
/// section (single-hart exclusive). Waiters are queued on the per-task `next`
/// link (a task is on at most one queue — ready OR one wait queue — at a time).
pub(super) struct Semaphore {
    pub(super) inner: UnsafeCell<SemState>,
}
pub(super) struct SemState {
    pub(super) count: i32,
    pub(super) wait_head: usize,
    pub(super) wait_tail: usize,
}

pub(super) fn enqueue_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
    let priority = sched.tasks[task].priority;
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL && sched.tasks[current].priority <= priority {
        previous = current;
        current = sched.tasks[current].next;
    }
    sched.tasks[task].next = current;
    if previous == NIL {
        state.wait_head = task;
    } else {
        sched.tasks[previous].next = task;
    }
    if current == NIL {
        state.wait_tail = task;
    }
}

pub(super) fn remove_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL {
        if current == task {
            let next = sched.tasks[current].next;
            if previous == NIL {
                state.wait_head = next;
            } else {
                sched.tasks[previous].next = next;
            }
            if state.wait_tail == current {
                state.wait_tail = previous;
            }
            sched.tasks[current].next = NIL;
            return;
        }
        previous = current;
        current = sched.tasks[current].next;
    }
}
// SAFETY: `inner` is only accessed inside `critical_section::with` on a single
// hart, which serialises every access.
unsafe impl Sync for Semaphore {}

impl Semaphore {
    /// Create a semaphore with initial `count`.
    pub(super) const fn new(count: i32) -> Self {
        Semaphore {
            inner: UnsafeCell::new(SemState {
                count,
                wait_head: NIL,
                wait_tail: NIL,
            }),
        }
    }

    /// Acquire (P). Consumes a count if available, else blocks until [`up`] hands
    /// one off. Direct-handoff semantics: being woken == being granted, so there
    /// is no re-check loop (the only thing that unblocks a waiter is `up`).
    ///
    /// [`up`]: Semaphore::up
    fn down(&self) -> Result<(), DriverError> {
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let block = critical_section::with(|cs| -> Result<_, DriverError> {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section (single hart).
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                Ok(false)
            } else {
                if !switch_delivery_available {
                    return Err(DriverError::InvalidContext);
                }
                let cur = s.current_switch_guard()?;
                s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
                s.tasks[cur].state = State::Blocked;
                s.tasks[cur].wake_at = 0;
                s.tasks[cur].waiting_sem = self as *const Self as usize;
                s.tasks[cur].sem_granted = false;
                enqueue_waiter(s, st, cur);
                Ok(true)
            }
        })?;
        if block {
            // Parked on this sem's wait queue; `up` will move us back to Ready
            // (== the grant). When we resume here, we already hold the count.
            switch_away(current_id());
            critical_section::with(|cs| {
                let s = &mut *SCHED.borrow_ref_mut(cs);
                s.tasks[s.current].sem_granted = false;
                s.tasks[s.current].granted_sem = 0;
            });
        }
        Ok(())
    }

    /// Acquire with a timeout (ms). Returns `true` if a count was obtained,
    /// `false` if the deadline passed first. `u32::MAX` (wait-forever) blocks
    /// like [`down`](Semaphore::down).
    ///
    /// The waiter is linked into the semaphore queue so [`up`](Self::up) can
    /// hand the grant directly to it. The scheduler removes it from that queue
    /// if the mask-ROM systick deadline wins first.
    pub(super) fn down_timeout(&self, timeout_ms: u32) -> Result<bool, DriverError> {
        if timeout_ms == u32::MAX {
            self.down()?;
            return Ok(true);
        }
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| -> Result<_, DriverError> {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                return Ok(None);
            }
            if timeout_ms == 0 {
                return Ok(Some(NIL));
            }
            if !switch_delivery_available {
                return Err(DriverError::InvalidContext);
            }
            let cur = s.current_switch_guard()?;
            s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
            s.tasks[cur].state = State::Blocked;
            s.tasks[cur].wake_at = deadline;
            s.tasks[cur].waiting_sem = self as *const Self as usize;
            s.tasks[cur].sem_granted = false;
            enqueue_waiter(s, st, cur);
            Ok(Some(cur))
        })?;
        if matches!(current, Some(slot) if slot != NIL) {
            rearm_timer();
        }
        match current {
            None => Ok(true),
            Some(NIL) => Ok(false),
            Some(current) => {
                switch_away(current);
                Ok(critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    s.tasks[s.current].granted_sem = 0;
                    granted
                }))
            }
        }
    }

    /// Release (V). Wakes one waiter if any, else increments the count.
    pub(super) fn up(&self) -> Result<(), DriverError> {
        let machine_interrupts_enabled = machine_interrupts_enabled();
        let now = now_ms();
        let mut defer_reschedule = false;
        let preemption = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            release_semaphore_locked(s, st, now);
            let interrupt_depth = INTERRUPT_DEPTH.borrow(cs).get();
            if interrupt_depth == 0 && machine_interrupts_enabled {
                s.take_preemption_target(now)
            } else {
                defer_reschedule = interrupt_depth == 0 && !machine_interrupts_enabled;
                None
            }
        });
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        if defer_reschedule {
            request_reschedule();
        }
        rearm_timer();
        Ok(())
    }
}

pub(super) fn release_semaphore_locked(sched: &mut Sched, state: &mut SemState, now: u64) -> usize {
    let waiter = state.wait_head;
    if waiter == NIL {
        state.count += 1;
        return NIL;
    }

    state.wait_head = sched.tasks[waiter].next;
    if state.wait_head == NIL {
        state.wait_tail = NIL;
    }
    sched.tasks[waiter].next = NIL;
    sched.tasks[waiter].wake_at = 0;
    sched.tasks[waiter].granted_sem = sched.tasks[waiter].waiting_sem;
    sched.tasks[waiter].waiting_sem = 0;
    sched.tasks[waiter].sem_granted = true;
    sched.make_ready(waiter, now);
    sched.diagnostics.semaphore_wakes = sched.diagnostics.semaphore_wakes.saturating_add(1);
    waiter
}

// Recursive mutex with priority-ordered waiters and priority inheritance.
pub(super) struct RtosMutex {
    pub(super) inner: UnsafeCell<MutexState>,
}

pub(super) struct MutexState {
    pub(super) owner: usize,
    pub(super) depth: u32,
    pub(super) wait_head: usize,
    pub(super) wait_tail: usize,
}

// SAFETY: all state is accessed under the single-hart scheduler critical section.
unsafe impl Sync for RtosMutex {}

impl RtosMutex {
    pub(super) const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(MutexState {
                owner: NIL,
                depth: 0,
                wait_head: NIL,
                wait_tail: NIL,
            }),
        }
    }

    pub(super) fn lock(&self, timeout_ms: u32) -> Result<bool, DriverError> {
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            let current = s.current;
            // SAFETY: exclusive under the scheduler critical section.
            let state = unsafe { &mut *self.inner.get() };
            if state.owner == current {
                state.depth = state.depth.checked_add(1).ok_or(DriverError::Runtime)?;
                return Ok(None);
            }
            if state.owner == NIL {
                state.owner = current;
                state.depth = 1;
                return Ok(None);
            }
            if timeout_ms == 0 {
                return Ok(Some(NIL));
            }
            if !switch_delivery_available {
                return Err(DriverError::InvalidContext);
            }
            s.current_switch_guard()?;
            if mutex_chain_contains(s, state.owner, current) {
                return Err(DriverError::InvalidContext);
            }

            let owner = state.owner;
            s.tasks[current].state = State::Blocked;
            s.tasks[current].wake_at = if timeout_ms == u32::MAX { 0 } else { deadline };
            s.tasks[current].waiting_mutex = self as *const Self as usize;
            s.tasks[current].sem_granted = false;
            enqueue_mutex_waiter(s, state, current);
            s.add_inheritance(owner, s.tasks[current].priority);
            Ok(Some(current))
        })?;

        if matches!(current, Some(slot) if slot != NIL) {
            rearm_timer();
        }
        match current {
            None => Ok(true),
            Some(NIL) => Ok(false),
            Some(current) => {
                switch_away(current);
                Ok(critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    s.tasks[s.current].granted_mutex = 0;
                    granted
                }))
            }
        }
    }

    pub(super) fn unlock(&self) -> Result<(), DriverError> {
        let machine_interrupts_enabled = machine_interrupts_enabled();
        let now = now_ms();
        let mut defer_reschedule = false;
        let preemption = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            let current = s.current;
            // SAFETY: exclusive under the scheduler critical section.
            let state = unsafe { &mut *self.inner.get() };
            if state.owner != current || state.depth == 0 {
                return Err(DriverError::InvalidContext);
            }
            state.depth -= 1;
            if state.depth != 0 {
                return Ok(None);
            }
            release_mutex_locked(s, state, current, now);

            let interrupt_depth = INTERRUPT_DEPTH.borrow(cs).get();
            Ok(if interrupt_depth == 0 && machine_interrupts_enabled {
                s.take_preemption_target(now)
            } else {
                defer_reschedule = interrupt_depth == 0 && !machine_interrupts_enabled;
                None
            })
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        if defer_reschedule {
            request_reschedule();
        }
        rearm_timer();
        Ok(())
    }
}

pub(super) fn release_mutex_locked(
    sched: &mut Sched,
    state: &mut MutexState,
    owner: usize,
    now: u64,
) {
    let next = pop_mutex_waiter(sched, state);
    if next == NIL {
        state.owner = NIL;
        return;
    }

    // Every waiter donated to the old owner. Remove those contributions before
    // changing ownership, including the waiter receiving the direct handoff.
    sched.remove_inheritance(owner, sched.tasks[next].priority);
    let mut waiter = state.wait_head;
    while waiter != NIL {
        sched.remove_inheritance(owner, sched.tasks[waiter].priority);
        waiter = sched.tasks[waiter].next;
    }

    state.owner = next;
    state.depth = 1;
    sched.tasks[next].granted_mutex = sched.tasks[next].waiting_mutex;
    sched.tasks[next].waiting_mutex = 0;
    sched.tasks[next].wake_at = 0;
    sched.tasks[next].sem_granted = true;
    sched.make_ready(next, now);

    // Remaining waiters now donate to the new owner.
    waiter = state.wait_head;
    while waiter != NIL {
        sched.add_inheritance(next, sched.tasks[waiter].priority);
        waiter = sched.tasks[waiter].next;
    }
}

pub(super) fn cancel_wait_locked(
    sched: &mut Sched,
    task: usize,
    now: u64,
) -> WaitCancellationOutcome {
    let waiting_sem = sched.tasks[task].waiting_sem;
    if waiting_sem != 0 {
        // SAFETY: a queued waiter keeps the semaphore allocation live and this
        // function is called with exclusive scheduler/queue access.
        let state = unsafe { &mut *(*(waiting_sem as *const Semaphore)).inner.get() };
        remove_waiter(sched, state, task);
        sched.tasks[task].waiting_sem = 0;
        sched.tasks[task].wake_at = 0;
        sched.tasks[task].sem_granted = false;
        if sched.tasks[task].state == State::Blocked {
            sched.make_ready(task, now);
        }
        return WaitCancellationOutcome::Cancelled;
    }

    let waiting_mutex = sched.tasks[task].waiting_mutex;
    if waiting_mutex != 0 {
        // SAFETY: a queued waiter keeps the mutex allocation live and queue
        // mutation is serialized by the scheduler critical section.
        let state = unsafe { &mut *(*(waiting_mutex as *const RtosMutex)).inner.get() };
        remove_mutex_waiter(sched, state, task);
        if state.owner != NIL {
            sched.remove_inheritance(state.owner, sched.tasks[task].priority);
        }
        sched.tasks[task].waiting_mutex = 0;
        sched.tasks[task].wake_at = 0;
        sched.tasks[task].sem_granted = false;
        if sched.tasks[task].state == State::Blocked {
            sched.make_ready(task, now);
        }
        return WaitCancellationOutcome::Cancelled;
    }

    let granted_sem = sched.tasks[task].granted_sem;
    if granted_sem != 0 {
        sched.tasks[task].granted_sem = 0;
        sched.tasks[task].sem_granted = false;
        // SAFETY: an unconsumed direct grant keeps the resource allocation live.
        let state = unsafe { &mut *(*(granted_sem as *const Semaphore)).inner.get() };
        release_semaphore_locked(sched, state, now);
        return WaitCancellationOutcome::Cancelled;
    }

    let granted_mutex = sched.tasks[task].granted_mutex;
    if granted_mutex != 0 {
        sched.tasks[task].granted_mutex = 0;
        sched.tasks[task].sem_granted = false;
        // SAFETY: an unconsumed handoff keeps the mutex live; direct handoff
        // makes `task` its depth-one owner until the waiter consumes the grant.
        let state = unsafe { &mut *(*(granted_mutex as *const RtosMutex)).inner.get() };
        debug_assert_eq!(state.owner, task);
        debug_assert_eq!(state.depth, 1);
        state.depth = 0;
        release_mutex_locked(sched, state, task, now);
        return WaitCancellationOutcome::Cancelled;
    }

    WaitCancellationOutcome::NotWaiting
}

pub(super) fn enqueue_mutex_waiter(sched: &mut Sched, state: &mut MutexState, task: usize) {
    let priority = sched.tasks[task].priority;
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL && sched.tasks[current].priority <= priority {
        previous = current;
        current = sched.tasks[current].next;
    }
    sched.tasks[task].next = current;
    if previous == NIL {
        state.wait_head = task;
    } else {
        sched.tasks[previous].next = task;
    }
    if current == NIL {
        state.wait_tail = task;
    }
}

pub(super) fn remove_mutex_waiter(sched: &mut Sched, state: &mut MutexState, task: usize) {
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL {
        if current == task {
            let next = sched.tasks[current].next;
            if previous == NIL {
                state.wait_head = next;
            } else {
                sched.tasks[previous].next = next;
            }
            if state.wait_tail == current {
                state.wait_tail = previous;
            }
            sched.tasks[current].next = NIL;
            return;
        }
        previous = current;
        current = sched.tasks[current].next;
    }
}

fn pop_mutex_waiter(sched: &mut Sched, state: &mut MutexState) -> usize {
    let task = state.wait_head;
    if task != NIL {
        state.wait_head = sched.tasks[task].next;
        if state.wait_head == NIL {
            state.wait_tail = NIL;
        }
        sched.tasks[task].next = NIL;
    }
    task
}

fn mutex_chain_contains(sched: &Sched, mut owner: usize, sought: usize) -> bool {
    for _ in 0..TASK_SLOT_COUNT {
        if owner == sought {
            return true;
        }
        let waiting = sched.tasks[owner].waiting_mutex;
        if waiting == 0 {
            return false;
        }
        // SAFETY: a blocked task keeps the mutex alive under the scheduler lock.
        let state = unsafe { &*(*(waiting as *const RtosMutex)).inner.get() };
        if state.owner == NIL {
            return false;
        }
        owner = state.owner;
    }
    true
}
