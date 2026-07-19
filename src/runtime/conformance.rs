use super::*;
extern crate std;
use hisi_rf_rtos_driver::conformance::{
    Action, ActionOutcome, ActorId, ActorState, Backend, ExecutionProfile, Observation,
    V1_SCENARIOS, run_suite,
};

struct DeterministicBackend {
    scheduler: Sched,
    profile: ExecutionProfile,
    now: u64,
    interrupt_depth: u16,
}

impl DeterministicBackend {
    fn new() -> Self {
        Self {
            scheduler: Sched::new(),
            profile: ExecutionProfile::Cooperative,
            now: 0,
            interrupt_depth: 0,
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

    fn state(&self, actor: ActorId) -> Result<ActorState, DriverError> {
        Ok(match self.scheduler.tasks[Self::slot(actor)?].state {
            State::Ready => ActorState::Ready,
            State::Running => ActorState::Running,
            State::Blocked | State::Sleeping | State::Throttled => ActorState::Blocked,
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
            Action::Yield => {
                let previous = self.scheduler.current;
                if let Some(next) = self.scheduler.take_yield_target(previous, self.now) {
                    self.switch(previous, next);
                    self.observation(Some(Self::actor(previous)?), ActionOutcome::ContextSwitched)
                } else {
                    self.observation(Some(Self::actor(previous)?), ActionOutcome::Completed)
                }
            }
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
                }
                self.observation(None, ActionOutcome::Completed)
            }
            Action::AdvanceTime { milliseconds } => {
                self.now = self.now.wrapping_add(u64::from(milliseconds));
                self.scheduler.wake_sleepers(self.now);
                self.observation(None, ActionOutcome::Completed)
            }
            Action::ExitTask => Err(DriverError::Runtime),
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
    assert!(json.contains("\"schema_version\":1"));
    assert!(json.contains("\"priority_then_fifo\""));
    assert!(json.contains("\"nested_scheduler_lock\""));
    assert!(!json.contains("\"status\":\"failed\""));
}
