use super::*;
extern crate std;
use hisi_rf_rtos_driver::conformance::{
    Action, ActionOutcome, ActorId, ActorState, Backend, ExecutionProfile, Observation,
    V1_SCENARIOS, Wait, run_suite,
};

struct DeterministicBackend {
    scheduler: Sched,
    profile: ExecutionProfile,
    now: u64,
    interrupt_depth: u16,
    semaphore: Semaphore,
    mutex: RtosMutex,
    remembered_identity: Option<TaskId>,
}

impl DeterministicBackend {
    fn new() -> Self {
        Self {
            scheduler: Sched::new(),
            profile: ExecutionProfile::Cooperative,
            now: 0,
            interrupt_depth: 0,
            semaphore: Semaphore::new(0),
            mutex: RtosMutex::new(),
            remembered_identity: None,
        }
    }

    fn slot(actor: ActorId) -> Result<usize, DriverError> {
        match actor.into_raw() {
            0 => Ok(0),
            1 => Ok(IDLE_SLOT + 1),
            2 => Ok(IDLE_SLOT + 2),
            _ => Err(DriverError::InvalidHandle),
        }
    }

    fn actor(slot: usize) -> Result<ActorId, DriverError> {
        match slot {
            0 => Ok(ActorId::MAIN),
            slot if slot == IDLE_SLOT + 1 => Ok(ActorId::WORKER_A),
            slot if slot == IDLE_SLOT + 2 => Ok(ActorId::WORKER_B),
            _ => Err(DriverError::InvalidHandle),
        }
    }

    fn policy(&self) -> RunPolicy {
        match self.profile {
            ExecutionProfile::Cooperative => RunPolicy::Cooperative,
            ExecutionProfile::Preemptive => RunPolicy::Preemptive {
                time_slice: NonZeroU32::new(1).unwrap(),
            },
        }
    }

    fn switch(&mut self, previous: usize, next: usize) {
        self.scheduler.account_switch(previous, next, self.now);
        self.scheduler.tasks[next].state = State::Running;
        self.scheduler.current = next;
    }

    fn reset_resources(&mut self) {
        // SAFETY: the deterministic backend is single-threaded and no task can
        // retain a resource wait across Reset.
        unsafe {
            let semaphore = &mut *self.semaphore.inner.get();
            semaphore.count = 0;
            semaphore.wait_head = NIL;
            semaphore.wait_tail = NIL;

            let mutex = &mut *self.mutex.inner.get();
            mutex.owner = NIL;
            mutex.depth = 0;
            mutex.wait_head = NIL;
            mutex.wait_tail = NIL;
        }
        self.remembered_identity = None;
    }

    fn yield_current(&mut self) -> Result<Observation, DriverError> {
        let previous = self.scheduler.current;
        if let Some(next) = self.scheduler.take_yield_target(previous, self.now) {
            self.switch(previous, next);
            self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
        } else {
            self.observation(Some(Self::actor(previous)?), ActionOutcome::Completed)
        }
    }

    fn sleep_current(&mut self, milliseconds: u32) -> Result<Observation, DriverError> {
        let previous = self.scheduler.current;
        self.scheduler.tasks[previous].state = State::Sleeping;
        self.scheduler.tasks[previous].wake_at = self.now.saturating_add(u64::from(milliseconds));
        let next = self.scheduler.ready_pop_or_idle();
        if next == IDLE_SLOT {
            return Err(DriverError::Runtime);
        }
        self.switch(previous, next);
        self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
    }

    fn wait_deadline(&self, wait: Wait) -> Option<u64> {
        match wait {
            Wait::NoWait => None,
            Wait::Milliseconds(milliseconds) => {
                Some(self.now.saturating_add(u64::from(milliseconds.get())))
            }
            Wait::Forever => Some(0),
        }
    }

    fn state(&self, actor: ActorId) -> Result<ActorState, DriverError> {
        Ok(match self.scheduler.tasks[Self::slot(actor)?].state {
            State::Ready => ActorState::Ready,
            State::Running => ActorState::Running,
            State::Blocked | State::Throttled => ActorState::Blocked,
            State::Sleeping => ActorState::Sleeping,
            State::Free => ActorState::Exited,
        })
    }

    fn observation(
        &self,
        subject: Option<ActorId>,
        outcome: ActionOutcome,
    ) -> Result<Observation, DriverError> {
        let current = self.scheduler.current;
        Ok(Observation {
            running: Self::actor(current)?,
            subject: subject
                .map(|actor| self.state(actor).map(|state| (actor, state)))
                .transpose()?,
            outcome,
            scheduler_lock_depth: self.scheduler.tasks[current].scheduler_lock_depth,
            interrupt_depth: self.interrupt_depth,
        })
    }
}

impl Backend for DeterministicBackend {
    fn contract(&self) -> RuntimeContract {
        RuntimeContract::V1
    }

    fn execution_profile(&self) -> RuntimeExecutionProfile {
        RuntimeExecutionProfile::V1_PORTED
    }

    fn revision(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    fn apply(&mut self, action: Action) -> Result<Observation, DriverError> {
        match action {
            Action::Reset {
                profile,
                main_priority,
            } => {
                self.scheduler = Sched::new();
                self.profile = profile;
                self.now = 0;
                self.interrupt_depth = 0;
                self.reset_resources();
                self.scheduler.started = true;
                self.scheduler.current = 0;
                self.scheduler.tasks[0].state = State::Running;
                self.scheduler.tasks[0].base_priority = main_priority.into_raw();
                self.scheduler.tasks[0].priority = main_priority.into_raw();
                self.scheduler.tasks[0].run_policy = self.policy();
                self.observation(Some(ActorId::MAIN), ActionOutcome::Completed)
            }
            Action::Spawn { actor, priority } => {
                let slot = Self::slot(actor)?;
                if self.scheduler.tasks[slot].state != State::Free {
                    return Err(DriverError::InvalidHandle);
                }
                self.scheduler.tasks[slot].state = State::Ready;
                self.scheduler.tasks[slot].base_priority = priority.into_raw();
                self.scheduler.tasks[slot].priority = priority.into_raw();
                self.scheduler.slot_generations[slot] =
                    self.scheduler.slot_generations[slot].wrapping_add(1).max(1);
                self.scheduler.tasks[slot].identity_generation =
                    self.scheduler.slot_generations[slot];
                self.scheduler.tasks[slot].run_policy = self.policy();
                self.scheduler.ready_push(slot);

                let current = self.scheduler.current;
                let preemption_deferred = matches!(self.profile, ExecutionProfile::Preemptive)
                    && priority.into_raw() < self.scheduler.tasks[current].priority
                    && self.scheduler.tasks[current].scheduler_lock_depth != 0;
                if let Some((previous, next)) = self.scheduler.take_preemption_target(self.now) {
                    self.switch(previous, next);
                    return self.observation(Some(actor), ActionOutcome::ContextSwitched);
                }
                self.observation(
                    Some(actor),
                    if preemption_deferred {
                        ActionOutcome::PreemptionDeferred
                    } else {
                        ActionOutcome::Spawned
                    },
                )
            }
            Action::Yield => self.yield_current(),
            Action::Delay { milliseconds: 0 } => self.yield_current(),
            Action::Delay { milliseconds } => self.sleep_current(milliseconds),
            Action::LockScheduler => {
                self.scheduler.lock_current(self.now)?;
                self.observation(
                    Some(Self::actor(self.scheduler.current)?),
                    ActionOutcome::Completed,
                )
            }
            Action::UnlockScheduler => {
                let previous = self.scheduler.current;
                if let Some((previous, next)) = self
                    .scheduler
                    .unlock_current_and_take_preemption(self.now)?
                {
                    self.switch(previous, next);
                    self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
                } else {
                    self.observation(Some(Self::actor(previous)?), ActionOutcome::Completed)
                }
            }
            Action::EnterInterrupt => {
                if self.interrupt_depth == 0 {
                    self.scheduler.interrupt_enter(self.now);
                }
                self.interrupt_depth = self
                    .interrupt_depth
                    .checked_add(1)
                    .ok_or(DriverError::Runtime)?;
                self.observation(None, ActionOutcome::Completed)
            }
            Action::ExitInterrupt => {
                if self.interrupt_depth == 0 {
                    return Err(DriverError::InvalidContext);
                }
                self.interrupt_depth -= 1;
                if self.interrupt_depth == 0 {
                    self.scheduler.interrupt_exit(self.now);
                    if let Some((previous, next)) = self
                        .scheduler
                        .take_irq_epilogue_target(self.interrupt_depth, self.now)
                    {
                        self.switch(previous, next);
                        return self.observation(
                            Some(Self::actor(previous)?),
                            ActionOutcome::ContextSwitched,
                        );
                    }
                }
                self.observation(None, ActionOutcome::Completed)
            }
            Action::AdvanceTime { milliseconds } => {
                self.now = self.now.wrapping_add(u64::from(milliseconds));
                self.scheduler.wake_sleepers(self.now);
                self.observation(None, ActionOutcome::Completed)
            }
            Action::Sleep { milliseconds } => {
                if milliseconds == 0 {
                    return Err(DriverError::InvalidContext);
                }
                self.sleep_current(milliseconds)
            }
            Action::Observe { actor } => self.observation(Some(actor), ActionOutcome::Completed),
            Action::ExitTask => {
                let previous = self.scheduler.current;
                self.scheduler.tasks[previous] = Tcb::empty();
                let next = self.scheduler.ready_pop_or_idle();
                if next == IDLE_SLOT {
                    return Err(DriverError::Runtime);
                }
                self.switch(previous, next);
                self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
            }
            Action::SemaphoreWait { timeout } => {
                let previous = self.scheduler.current;
                let deadline = self.wait_deadline(timeout);
                // SAFETY: the deterministic backend has exclusive access.
                let state = unsafe { &mut *self.semaphore.inner.get() };
                if state.count > 0 {
                    state.count -= 1;
                    return self.observation(None, ActionOutcome::Acquired);
                }
                let Some(deadline) = deadline else {
                    return self.observation(None, ActionOutcome::TimedOut);
                };
                self.scheduler.tasks[previous].state = State::Blocked;
                self.scheduler.tasks[previous].wake_at = deadline;
                self.scheduler.tasks[previous].waiting_sem =
                    &self.semaphore as *const Semaphore as usize;
                self.scheduler.tasks[previous].sem_granted = false;
                enqueue_waiter(&mut self.scheduler, state, previous);
                let next = self.scheduler.ready_pop_or_idle();
                if next == IDLE_SLOT {
                    return Err(DriverError::Runtime);
                }
                self.switch(previous, next);
                self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
            }
            Action::SemaphorePost => {
                // SAFETY: the deterministic backend has exclusive access.
                let state = unsafe { &mut *self.semaphore.inner.get() };
                let waiter = release_semaphore_locked(&mut self.scheduler, state, self.now);
                if waiter == NIL {
                    self.observation(None, ActionOutcome::Completed)
                } else {
                    self.observation(Some(Self::actor(waiter)?), ActionOutcome::Granted)
                }
            }
            Action::ObserveGrant { actor } => {
                let granted = self.scheduler.tasks[Self::slot(actor)?].sem_granted;
                self.observation(
                    Some(actor),
                    if granted {
                        ActionOutcome::Acquired
                    } else {
                        ActionOutcome::TimedOut
                    },
                )
            }
            Action::MutexLock { timeout } => {
                let current = self.scheduler.current;
                let deadline = self.wait_deadline(timeout);
                // SAFETY: the deterministic backend has exclusive access.
                let state = unsafe { &mut *self.mutex.inner.get() };
                if state.owner == current {
                    state.depth = state.depth.checked_add(1).ok_or(DriverError::Runtime)?;
                    return self.observation(None, ActionOutcome::Acquired);
                }
                if state.owner == NIL {
                    state.owner = current;
                    state.depth = 1;
                    return self.observation(None, ActionOutcome::Acquired);
                }
                let Some(deadline) = deadline else {
                    return self.observation(None, ActionOutcome::TimedOut);
                };
                let owner = state.owner;
                self.scheduler.tasks[current].state = State::Blocked;
                self.scheduler.tasks[current].wake_at = deadline;
                self.scheduler.tasks[current].waiting_mutex =
                    &self.mutex as *const RtosMutex as usize;
                self.scheduler.tasks[current].sem_granted = false;
                enqueue_mutex_waiter(&mut self.scheduler, state, current);
                self.scheduler
                    .add_inheritance(owner, self.scheduler.tasks[current].priority);
                let next = self.scheduler.ready_pop_or_idle();
                if next == IDLE_SLOT {
                    return Err(DriverError::Runtime);
                }
                self.switch(current, next);
                self.observation(Some(Self::actor(current)?), ActionOutcome::ContextSwitched)
            }
            Action::MutexUnlock => {
                let current = self.scheduler.current;
                // SAFETY: the deterministic backend has exclusive access.
                let state = unsafe { &mut *self.mutex.inner.get() };
                if state.owner != current || state.depth == 0 {
                    return Err(DriverError::InvalidContext);
                }
                state.depth -= 1;
                if state.depth != 0 {
                    return self.observation(None, ActionOutcome::Completed);
                }
                let waiter = state.wait_head;
                release_mutex_locked(&mut self.scheduler, state, current, self.now);
                if waiter == NIL {
                    self.observation(None, ActionOutcome::Completed)
                } else {
                    self.observation(Some(Self::actor(waiter)?), ActionOutcome::Granted)
                }
            }
            Action::ObservePriority { actor } => {
                let priority = TaskPriority::new(self.scheduler.tasks[Self::slot(actor)?].priority)
                    .ok_or(DriverError::Runtime)?;
                self.observation(Some(actor), ActionOutcome::PriorityObserved(priority))
            }
            Action::RememberIdentity { actor } => {
                let slot = Self::slot(actor)?;
                self.remembered_identity = Some(encode_task_id(
                    slot,
                    self.scheduler.tasks[slot].identity_generation,
                )?);
                self.observation(Some(actor), ActionOutcome::IdentityRemembered)
            }
            Action::ValidateRememberedIdentity => {
                let remembered = self.remembered_identity.ok_or(DriverError::InvalidHandle)?;
                let (slot, generation) = decode_task_id(remembered)?;
                if self.scheduler.tasks[slot].state != State::Free
                    && self.scheduler.tasks[slot].identity_generation == generation
                {
                    return Err(DriverError::Runtime);
                }
                self.observation(None, ActionOutcome::StaleIdentityRejected)
            }
        }
    }
}

#[test]
fn runtime_v1_executes_shared_conformance_scenarios() {
    let mut backend = DeterministicBackend::new();
    let report = run_suite(&mut backend, &V1_SCENARIOS);
    assert!(report.all_passed(), "{report:?}");

    let mut json = std::string::String::new();
    report.write_json(&mut json).unwrap();
    assert!(json.contains("\"schema_version\":3"));
    assert!(json.contains("\"execution_profile\":{\"revision\":1,\"modes\":14}"));
    assert!(json.contains("\"priority_then_fifo\""));
    assert!(json.contains("\"nested_scheduler_lock\""));
    assert!(json.contains("\"sleep_deadline\""));
    assert!(json.contains("\"nested_interrupt_exit\""));
    assert!(json.contains("\"task_exit_and_reuse\""));
    assert!(json.contains("\"semaphore_direct_handoff\""));
    assert!(json.contains("\"semaphore_timeout_cleanup\""));
    assert!(json.contains("\"mutex_priority_inheritance\""));
    assert!(json.contains("\"stale_task_identity\""));
    assert!(json.contains("\"zero_delay_yields\""));
    assert!(json.contains("\"wait_forever\""));
    assert!(json.contains("\"same_deadline_fifo\""));
    assert!(json.contains("\"semaphore_highest_priority_waiter\""));
    assert!(!json.contains("\"status\":\"failed\""));
}
