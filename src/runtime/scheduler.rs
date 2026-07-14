use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum State {
    Free,
    Ready,
    Running,
    Blocked,
    Sleeping,
    Throttled,
}

/// Task entry signature (matches the OSAL `osal_kthread_func`).
pub(super) type TaskFn = extern "C" fn(*mut c_void) -> *mut c_void;

pub(super) struct Tcb {
    pub(super) ctx: TaskContext,
    pub(super) saved_frame: usize,
    pub(super) resume_generation: u32,
    pub(super) state: State,
    pub(super) stack: usize, // heap allocation addr to free on exit (0 for the main task)
    pub(super) entry: Option<TaskFn>,
    pub(super) arg: usize, // task argument (*mut c_void stored as usize so Tcb is Send)
    pub(super) next: usize, // intrusive link: ready queue OR one wait queue
    pub(super) wake_at: u64, // mask-ROM systick millisecond deadline
    pub(super) waiting_sem: usize,
    pub(super) waiting_mutex: usize,
    pub(super) sem_granted: bool,
    pub(super) base_priority: u8,
    pub(super) priority: u8,
    pub(super) inherited_waiters: [u8; PRIORITY_LEVELS],
    pub(super) scheduler_lock_depth: u16,
    pub(super) scheduler_lock_started_at: Option<u64>,
    pub(super) identity_generation: u16,
    pub(super) run_policy: RunPolicy,
    pub(super) budget: BudgetState,
}
impl Tcb {
    pub(super) const fn empty() -> Self {
        Tcb {
            ctx: TaskContext::zero(),
            saved_frame: 0,
            resume_generation: 0,
            state: State::Free,
            stack: 0,
            entry: None,
            arg: 0,
            next: NIL,
            wake_at: 0,
            waiting_sem: 0,
            waiting_mutex: 0,
            sem_granted: false,
            base_priority: (PRIORITY_LEVELS - 1) as u8,
            priority: (PRIORITY_LEVELS - 1) as u8,
            inherited_waiters: [0; PRIORITY_LEVELS],
            scheduler_lock_depth: 0,
            scheduler_lock_started_at: None,
            identity_generation: 0,
            run_policy: RunPolicy::Cooperative,
            budget: BudgetState::none(),
        }
    }
}

pub(super) struct Sched {
    pub(super) tasks: [Tcb; TASK_SLOT_COUNT],
    pub(super) current: usize,
    pub(super) ready_head: [usize; PRIORITY_LEVELS],
    pub(super) ready_tail: [usize; PRIORITY_LEVELS],
    pub(super) retired_stacks: [usize; TASK_SLOT_COUNT],
    pub(super) retired_count: usize,
    pub(super) slot_generations: [u16; TASK_SLOT_COUNT],
    pub(super) time_slice_pending: bool,
    pub(super) time_slice_deadline: u64,
    pub(super) forced_next: usize,
    pub(super) started: bool,
    pub(super) diagnostics: Diagnostics,
}
impl Sched {
    pub(super) const fn new() -> Self {
        const E: Tcb = Tcb::empty();
        Sched {
            tasks: [E; TASK_SLOT_COUNT],
            current: 0,
            ready_head: [NIL; PRIORITY_LEVELS],
            ready_tail: [NIL; PRIORITY_LEVELS],
            retired_stacks: [0; TASK_SLOT_COUNT],
            retired_count: 0,
            slot_generations: [0; TASK_SLOT_COUNT],
            time_slice_pending: false,
            time_slice_deadline: 0,
            forced_next: NIL,
            started: false,
            diagnostics: Diagnostics::EMPTY,
        }
    }

    pub(super) fn diagnostics(&self) -> Diagnostics {
        let mut snapshot = self.diagnostics;
        snapshot.current_task = self.current;
        snapshot.current_lock_depth = self.tasks[self.current].scheduler_lock_depth;
        for task in &self.tasks {
            match task.state {
                State::Ready => snapshot.ready_tasks = snapshot.ready_tasks.saturating_add(1),
                State::Blocked => snapshot.blocked_tasks = snapshot.blocked_tasks.saturating_add(1),
                State::Sleeping => {
                    snapshot.sleeping_tasks = snapshot.sleeping_tasks.saturating_add(1)
                }
                State::Throttled => {
                    snapshot.throttled_tasks = snapshot.throttled_tasks.saturating_add(1)
                }
                State::Free | State::Running => {}
            }
        }
        snapshot.dynamic_used = self.tasks[(IDLE_SLOT + 1)..]
            .iter()
            .filter(|task| task.state != State::Free)
            .count() as u8;
        snapshot.dynamic_free = snapshot
            .dynamic_capacity
            .saturating_sub(snapshot.dynamic_used);
        snapshot
    }

    pub(super) fn task_diagnostics(&self, output: &mut [TaskDiagnostic]) -> usize {
        let count = output.len().min(TASK_SLOT_COUNT);
        for (index, output) in output[..count].iter_mut().enumerate() {
            let task = &self.tasks[index];
            *output = TaskDiagnostic {
                task: index,
                generation: task.identity_generation,
                state: match task.state {
                    State::Free => TaskState::Free,
                    State::Ready => TaskState::Ready,
                    State::Running => TaskState::Running,
                    State::Blocked => TaskState::Blocked,
                    State::Sleeping => TaskState::Sleeping,
                    State::Throttled => TaskState::Throttled,
                },
                entry: task.entry.map_or(0, |entry| entry as usize),
                waiting_sem: task.waiting_sem,
                waiting_mutex: task.waiting_mutex,
                wake_at: task.wake_at,
                base_priority: task.base_priority,
                priority: task.priority,
                scheduler_lock_depth: task.scheduler_lock_depth,
                run_policy: task.run_policy,
                budget_remaining: task.budget.remaining(),
                budget_replenishes_at: task.budget.replenishes_at(),
            };
        }
        count
    }
    pub(super) fn ready_priority(&self, task: usize) -> usize {
        self.tasks[task].priority as usize
    }
    pub(super) fn ready_push(&mut self, i: usize) {
        let priority = self.ready_priority(i);
        self.tasks[i].next = NIL;
        if self.ready_tail[priority] == NIL {
            self.ready_head[priority] = i;
        } else {
            self.tasks[self.ready_tail[priority]].next = i;
        }
        self.ready_tail[priority] = i;
    }
    pub(super) fn ready_pop(&mut self) -> usize {
        for priority in 0..PRIORITY_LEVELS {
            let i = self.ready_head[priority];
            if i != NIL {
                self.ready_head[priority] = self.tasks[i].next;
                if self.ready_head[priority] == NIL {
                    self.ready_tail[priority] = NIL;
                }
                self.tasks[i].next = NIL;
                return i;
            }
        }
        NIL
    }

    pub(super) fn ready_pop_or_idle(&mut self) -> usize {
        let next = self.ready_pop();
        if next == NIL { IDLE_SLOT } else { next }
    }
    pub(super) fn ready_remove(&mut self, task: usize) {
        let priority = self.ready_priority(task);
        let mut previous = NIL;
        let mut current = self.ready_head[priority];
        while current != NIL {
            if current == task {
                let next = self.tasks[current].next;
                if previous == NIL {
                    self.ready_head[priority] = next;
                } else {
                    self.tasks[previous].next = next;
                }
                if self.ready_tail[priority] == current {
                    self.ready_tail[priority] = previous;
                }
                self.tasks[current].next = NIL;
                return;
            }
            previous = current;
            current = self.tasks[current].next;
        }
    }

    pub(super) fn set_effective_priority(&mut self, task: usize, priority: u8) {
        if self.tasks[task].priority == priority {
            return;
        }
        let ready = self.tasks[task].state == State::Ready;
        if ready {
            self.ready_remove(task);
        }
        self.tasks[task].priority = priority;
        if ready {
            self.ready_push(task);
        }
    }

    pub(super) fn refresh_inherited_priority(&mut self, task: usize, depth: usize) {
        assert!(depth < TASK_SLOT_COUNT, "mutex inheritance cycle");
        let old = self.tasks[task].priority;
        let inherited = self.tasks[task]
            .inherited_waiters
            .iter()
            .position(|count| *count != 0)
            .map_or(self.tasks[task].base_priority, |priority| priority as u8);
        let new = self.tasks[task].base_priority.min(inherited);
        if old == new {
            return;
        }
        self.set_effective_priority(task, new);

        let waiting_mutex = self.tasks[task].waiting_mutex;
        if waiting_mutex != 0 {
            // SAFETY: a blocked task keeps its mutex alive; all mutation occurs
            // under this scheduler critical section.
            let state = unsafe { &mut *(*(waiting_mutex as *const RtosMutex)).inner.get() };
            remove_mutex_waiter(self, state, task);
            enqueue_mutex_waiter(self, state, task);
            if state.owner != NIL {
                self.replace_inheritance(state.owner, old, new, depth + 1);
            }
        }
    }

    pub(super) fn replace_inheritance(&mut self, owner: usize, old: u8, new: u8, depth: usize) {
        if old == new {
            return;
        }
        let old_count = &mut self.tasks[owner].inherited_waiters[old as usize];
        *old_count = old_count.checked_sub(1).expect("missing inherited waiter");
        let new_count = &mut self.tasks[owner].inherited_waiters[new as usize];
        *new_count = new_count
            .checked_add(1)
            .expect("too many inherited waiters");
        self.refresh_inherited_priority(owner, depth);
    }

    pub(super) fn add_inheritance(&mut self, owner: usize, priority: u8) {
        let count = &mut self.tasks[owner].inherited_waiters[priority as usize];
        *count = count.checked_add(1).expect("too many inherited waiters");
        self.diagnostics.priority_inheritances =
            self.diagnostics.priority_inheritances.saturating_add(1);
        self.refresh_inherited_priority(owner, 0);
    }

    pub(super) fn remove_inheritance(&mut self, owner: usize, priority: u8) {
        let count = &mut self.tasks[owner].inherited_waiters[priority as usize];
        *count = count.checked_sub(1).expect("missing inherited waiter");
        self.refresh_inherited_priority(owner, 0);
    }
    pub(super) fn take_yield_target(&mut self, current: usize) -> Option<usize> {
        let next = self.ready_pop();
        if next == NIL {
            return None;
        }
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some(next)
    }
    pub(super) fn take_reschedule_target(
        &mut self,
        allow_equal_priority: bool,
    ) -> Option<(usize, usize)> {
        let current = self.current;
        if self.tasks[current].state != State::Running
            || self.tasks[current].scheduler_lock_depth != 0
            || !matches!(self.tasks[current].run_policy, RunPolicy::Preemptive { .. })
        {
            return None;
        }
        let current_priority = self.ready_priority(current);
        let end = if allow_equal_priority {
            current_priority.saturating_add(1)
        } else {
            current_priority
        };
        if !(0..end).any(|priority| self.ready_head[priority] != NIL) {
            return None;
        }
        let next = self.ready_pop();
        debug_assert!(next != NIL);
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some((current, next))
    }

    pub(super) fn take_preemption_target(&mut self) -> Option<(usize, usize)> {
        self.take_reschedule_target(false)
    }

    #[cfg(test)]
    pub(super) fn take_irq_epilogue_target(
        &mut self,
        interrupt_depth: u16,
    ) -> Option<(usize, usize)> {
        if !self.started || interrupt_depth != 0 {
            return None;
        }
        let time_slice = self.time_slice_pending;
        let current_priority = self.tasks[self.current].priority;
        let target = if self.tasks[self.current].state != State::Running {
            Some((self.current, self.ready_pop_or_idle()))
        } else {
            self.take_reschedule_target(time_slice)
        };
        if target.is_some() {
            self.diagnostics.irq_preemptions = self.diagnostics.irq_preemptions.saturating_add(1);
            if time_slice
                && target.is_some_and(|(_, next)| self.tasks[next].priority == current_priority)
            {
                self.diagnostics.time_slice_preemptions =
                    self.diagnostics.time_slice_preemptions.saturating_add(1);
            }
            self.time_slice_pending = false;
        }
        target
    }

    #[cfg(target_arch = "riscv32")]
    pub(super) fn schedule_from_trap(
        &mut self,
        frame: usize,
        interrupt_depth: u16,
        now: u64,
    ) -> usize {
        if !self.started || interrupt_depth != 0 {
            return frame;
        }

        let current = self.current;
        let current_priority = self.tasks[current].priority;
        let time_slice = self.time_slice_pending;
        let target = if self.forced_next != NIL {
            let next = self.forced_next;
            self.forced_next = NIL;
            Some((current, next))
        } else if self.tasks[current].state != State::Running {
            Some((current, self.ready_pop_or_idle()))
        } else {
            self.take_reschedule_target(time_slice)
        };

        let Some((previous, next)) = target else {
            return frame;
        };

        self.account_switch(previous, next, now);

        if self.tasks[previous].state != State::Free {
            self.tasks[previous].saved_frame = frame;
        }

        let next_frame = self.tasks[next].saved_frame;
        assert!(next_frame != 0, "target task has no saved trap frame");
        self.tasks[next].saved_frame = 0;
        self.tasks[next].state = State::Running;
        self.tasks[next].resume_generation = self.tasks[next].resume_generation.wrapping_add(1);
        self.current = next;
        self.diagnostics.context_switches = self.diagnostics.context_switches.saturating_add(1);
        self.diagnostics.irq_preemptions = self.diagnostics.irq_preemptions.saturating_add(1);
        if time_slice && self.tasks[next].priority == current_priority {
            self.diagnostics.time_slice_preemptions =
                self.diagnostics.time_slice_preemptions.saturating_add(1);
        }
        self.time_slice_pending = false;
        next_frame
    }
    pub(super) fn retire_stack(&mut self, stack: usize) {
        if stack != 0 {
            debug_assert!(self.retired_count < TASK_SLOT_COUNT);
            self.retired_stacks[self.retired_count] = stack;
            self.retired_count += 1;
        }
    }
    pub(super) fn lock_current(&mut self, now: u64) -> Result<(), DriverError> {
        let task = &mut self.tasks[self.current];
        if task.scheduler_lock_depth == 0 {
            task.scheduler_lock_started_at = Some(now);
        }
        task.scheduler_lock_depth = task
            .scheduler_lock_depth
            .checked_add(1)
            .ok_or(DriverError::Runtime)?;
        self.diagnostics.scheduler_locks = self.diagnostics.scheduler_locks.saturating_add(1);
        Ok(())
    }
    pub(super) fn unlock_current(&mut self) -> Result<(), DriverError> {
        let task = &mut self.tasks[self.current];
        if task.scheduler_lock_depth == 0 {
            return Err(DriverError::InvalidContext);
        }
        task.scheduler_lock_depth -= 1;
        if task.scheduler_lock_depth == 0 {
            task.scheduler_lock_started_at = None;
        }
        Ok(())
    }

    pub(super) fn unlock_current_and_take_preemption(
        &mut self,
        now: u64,
    ) -> Result<Option<(usize, usize)>, DriverError> {
        self.unlock_current()?;
        if self.tasks[self.current].scheduler_lock_depth == 0
            && self.tasks[self.current].budget.lock_overrun_pending()
        {
            let current = self.current;
            self.tasks[current]
                .budget
                .throttle_after_lock_overrun(now)
                .expect("pending budget overrun has a budget policy");
            self.tasks[current].state = State::Throttled;
            self.diagnostics.budget_throttles = self.diagnostics.budget_throttles.saturating_add(1);
            let next = self.ready_pop_or_idle();
            return Ok(Some((current, next)));
        }
        let target = self.take_reschedule_target(self.time_slice_pending);
        if target.is_some() {
            self.time_slice_pending = false;
        }
        Ok(target)
    }
    pub(super) fn wake_sleepers(&mut self, now: u64) {
        for i in 0..TASK_SLOT_COUNT {
            if self.tasks[i].state == State::Sleeping && now >= self.tasks[i].wake_at {
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.sleeper_wakes = self.diagnostics.sleeper_wakes.saturating_add(1);
            } else if self.tasks[i].state == State::Blocked
                && self.tasks[i].waiting_sem != 0
                && self.tasks[i].wake_at != 0
                && now >= self.tasks[i].wake_at
            {
                let sem = self.tasks[i].waiting_sem as *const Semaphore;
                // SAFETY: a timed waiter keeps the semaphore alive for the
                // duration of the call, and all queue mutation is serialized by
                // the scheduler critical section.
                let sem_state = unsafe { &mut *(*sem).inner.get() };
                remove_waiter(self, sem_state, i);
                self.tasks[i].waiting_sem = 0;
                self.tasks[i].sem_granted = false;
                self.tasks[i].wake_at = 0;
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.semaphore_timeouts =
                    self.diagnostics.semaphore_timeouts.saturating_add(1);
            } else if self.tasks[i].state == State::Blocked
                && self.tasks[i].waiting_mutex != 0
                && self.tasks[i].wake_at != 0
                && now >= self.tasks[i].wake_at
            {
                let mutex = self.tasks[i].waiting_mutex as *const RtosMutex;
                // SAFETY: the waiter keeps the mutex alive and the scheduler
                // critical section serializes its queue and owner state.
                let state = unsafe { &mut *(*mutex).inner.get() };
                remove_mutex_waiter(self, state, i);
                if state.owner != NIL {
                    self.remove_inheritance(state.owner, self.tasks[i].priority);
                }
                self.tasks[i].waiting_mutex = 0;
                self.tasks[i].sem_granted = false;
                self.tasks[i].wake_at = 0;
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
            }
        }
        self.replenish_budgets(now);
    }

    pub(super) fn replenish_budgets(&mut self, now: u64) {
        for i in 0..TASK_SLOT_COUNT {
            if self.tasks[i].state == State::Throttled && self.tasks[i].budget.replenish_if_due(now)
            {
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.budget_replenishments =
                    self.diagnostics.budget_replenishments.saturating_add(1);
            }
        }
    }

    pub(super) fn on_timer(
        &mut self,
        now: u64,
        max_scheduler_lock_duration: NonZeroU32,
    ) -> Option<ContractViolation> {
        self.wake_sleepers(now);
        let current = self.current;
        if self.tasks[current].state != State::Running {
            return None;
        }
        let locked = self.tasks[current].scheduler_lock_depth != 0;
        match self.tasks[current].budget.on_timer(now, locked) {
            BudgetExpiry::ThrottleNow => {
                self.tasks[current].state = State::Throttled;
                self.diagnostics.budget_exhaustions =
                    self.diagnostics.budget_exhaustions.saturating_add(1);
                self.diagnostics.budget_throttles =
                    self.diagnostics.budget_throttles.saturating_add(1);
            }
            BudgetExpiry::DeferredBySchedulerLock => {
                self.diagnostics.budget_exhaustions =
                    self.diagnostics.budget_exhaustions.saturating_add(1);
                self.diagnostics.budget_lock_overruns =
                    self.diagnostics.budget_lock_overruns.saturating_add(1);
            }
            BudgetExpiry::NotBudgeted | BudgetExpiry::StillAvailable => {}
        }
        let task = &self.tasks[current];
        let violation = task.scheduler_lock_started_at.and_then(|started_at| {
            let held_ms = now.saturating_sub(started_at);
            (held_ms >= u64::from(max_scheduler_lock_duration.get())).then_some(
                ContractViolation::SchedulerLockOverrun {
                    task_slot: current,
                    held_ms,
                    limit_ms: max_scheduler_lock_duration.get(),
                },
            )
        });
        if violation.is_some() {
            self.diagnostics.scheduler_lock_overruns =
                self.diagnostics.scheduler_lock_overruns.saturating_add(1);
        }
        violation
    }

    pub(super) fn account_switch(&mut self, previous: usize, next: usize, now: u64) {
        self.tasks[previous].budget.on_switch_out(now);
        self.tasks[next].budget.on_dispatch(now);
        self.time_slice_deadline = 0;
    }
    pub(super) fn alloc_dynamic_slot(&mut self) -> Result<usize, DriverError> {
        ((IDLE_SLOT + 1)..TASK_SLOT_COUNT)
            .find(|&i| self.tasks[i].state == State::Free)
            .ok_or(DriverError::NoTaskSlots)
    }

    pub(super) fn set_run_policy(&mut self, slot: usize, policy: RunPolicy, now: u64) {
        let was_ready = self.tasks[slot].state == State::Ready;
        let was_throttled = self.tasks[slot].state == State::Throttled;
        if was_ready {
            self.ready_remove(slot);
        }
        let task = &mut self.tasks[slot];
        task.run_policy = policy;
        task.budget = BudgetState::for_policy(policy, now);
        if task.state == State::Running {
            task.budget.on_dispatch(now);
        } else if was_throttled {
            // The old budget no longer owns eligibility after a policy change.
            // A new Budgeted policy also starts with a fresh full budget.
            task.state = State::Ready;
        }
        if was_ready || was_throttled {
            self.ready_push(slot);
        }
    }

    pub(super) fn current_switch_guard(&self) -> Result<usize, DriverError> {
        let current = self.current;
        if self.tasks[current].scheduler_lock_depth != 0 {
            return Err(DriverError::InvalidContext);
        }
        Ok(current)
    }

    pub(super) fn earliest_wake_deadline(&self) -> Option<u64> {
        self.tasks
            .iter()
            .filter(|task| {
                matches!(task.state, State::Sleeping | State::Blocked) && task.wake_at != 0
            })
            .map(|task| task.wake_at)
            .min()
    }

    pub(super) fn earliest_budget_deadline(&self) -> Option<u64> {
        let current_deadline = (self.tasks[self.current].state == State::Running)
            .then(|| self.tasks[self.current].budget.exhaustion_deadline())
            .flatten();
        let replenish_deadline = self
            .tasks
            .iter()
            .filter(|task| task.state == State::Throttled)
            .map(|task| task.budget.replenishes_at())
            .filter(|deadline| *deadline != 0)
            .min();
        current_deadline.into_iter().chain(replenish_deadline).min()
    }

    pub(super) fn scheduler_lock_deadline(&self, max_duration: NonZeroU32) -> Option<u64> {
        let task = &self.tasks[self.current];
        (task.state == State::Running && task.scheduler_lock_depth != 0)
            .then_some(task.scheduler_lock_started_at)
            .flatten()
            .map(|started_at| started_at.saturating_add(u64::from(max_duration.get())))
    }

    pub(super) fn next_time_slice_deadline(&mut self, now: u64) -> Option<u64> {
        let RunPolicy::Preemptive { time_slice } = self.tasks[self.current].run_policy else {
            self.time_slice_deadline = 0;
            return None;
        };
        if !self.has_equal_priority_ready(self.tasks[self.current].priority) {
            self.time_slice_deadline = 0;
            return None;
        }
        if self.time_slice_deadline == 0 {
            self.time_slice_deadline = now.saturating_add(time_slice.get() as u64);
        }
        Some(self.time_slice_deadline)
    }

    pub(super) fn has_equal_priority_ready(&self, priority: u8) -> bool {
        self.ready_head[priority as usize] != NIL
    }
}
