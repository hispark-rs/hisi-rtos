use super::*;

fn ready_task(scheduler: &mut Sched, slot: usize, priority: u8) {
    scheduler.tasks[slot].state = State::Ready;
    scheduler.tasks[slot].priority = priority;
    scheduler.tasks[slot].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(1).unwrap(),
    };
    scheduler.ready_push(slot);
}

#[test]
fn dynamic_allocation_reserves_main_and_idle_slots() {
    let mut scheduler = Sched::new();

    for dynamic in 0..DYNAMIC_TASK_CAPACITY {
        let slot = IDLE_SLOT + 1 + dynamic;
        assert_eq!(scheduler.alloc_dynamic_slot(), Ok(slot));
        scheduler.tasks[slot].state = State::Ready;
    }
    assert_eq!(
        scheduler.alloc_dynamic_slot(),
        Err(DriverError::NoTaskSlots)
    );
    assert_eq!(scheduler.tasks[IDLE_SLOT].state, State::Free);

    let diagnostics = scheduler.diagnostics();
    assert_eq!(diagnostics.internal_tasks, 2);
    assert_eq!(diagnostics.dynamic_capacity, 15);
    assert_eq!(diagnostics.dynamic_used, 15);
    assert_eq!(diagnostics.dynamic_free, 0);
}

#[test]
fn idle_is_selected_only_when_the_ready_queues_are_empty() {
    let mut scheduler = Sched::new();

    assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT);
    ready_task(&mut scheduler, IDLE_SLOT + 1, (PRIORITY_LEVELS - 1) as u8);
    assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT + 1);
    assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT);
}

#[test]
fn idle_yield_hands_off_without_entering_the_ready_queue() {
    let mut scheduler = Sched::new();
    scheduler.current = IDLE_SLOT;
    scheduler.tasks[IDLE_SLOT].state = State::Running;
    ready_task(&mut scheduler, IDLE_SLOT + 1, 4);

    assert_eq!(
        scheduler.take_yield_target(IDLE_SLOT, 0),
        Some(IDLE_SLOT + 1)
    );
    assert_eq!(scheduler.tasks[IDLE_SLOT].state, State::Ready);
    assert_eq!(scheduler.ready_pop(), NIL);
}

#[test]
fn timer_wakeup_preempts_idle_without_queueing_idle() {
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.current = IDLE_SLOT;
    scheduler.tasks[IDLE_SLOT].state = State::Running;
    scheduler.tasks[IDLE_SLOT + 1].state = State::Sleeping;
    scheduler.tasks[IDLE_SLOT + 1].priority = 4;
    scheduler.tasks[IDLE_SLOT + 1].wake_at = 10;

    assert_eq!(scheduler.on_timer(10, NonZeroU32::new(100).unwrap()), None);
    assert_eq!(
        scheduler.take_irq_epilogue_target(0, 10),
        Some((IDLE_SLOT, IDLE_SLOT + 1))
    );
    assert_eq!(scheduler.tasks[IDLE_SLOT].state, State::Ready);
    assert_eq!(scheduler.ready_pop(), NIL);
}

#[test]
fn scheduler_lock_rejects_switching_or_blocking_entry_points() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;

    assert_eq!(scheduler.current_switch_guard(), Ok(0));
    scheduler.lock_current(0).unwrap();
    assert_eq!(
        scheduler.current_switch_guard(),
        Err(DriverError::InvalidContext)
    );
    scheduler.unlock_current(1).unwrap();
    assert_eq!(scheduler.current_switch_guard(), Ok(0));
}

#[test]
fn ported_thread_switch_requires_mie_but_irq_epilogue_does_not() {
    assert!(switch_delivery_is_valid(false, false, false));
    assert!(switch_delivery_is_valid(true, false, true));
    assert!(switch_delivery_is_valid(true, true, false));
    assert!(!switch_delivery_is_valid(true, false, false));
}

#[test]
fn ready_queue_prefers_lower_priority_number_and_keeps_fifo() {
    let mut scheduler = Sched::new();
    ready_task(&mut scheduler, 1, 8);
    ready_task(&mut scheduler, 2, 4);
    ready_task(&mut scheduler, 3, 4);

    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.ready_pop(), 3);
    assert_eq!(scheduler.ready_pop(), 1);
    assert_eq!(scheduler.ready_pop(), NIL);
}

#[test]
fn ready_task_can_move_between_priority_queues() {
    let mut scheduler = Sched::new();
    ready_task(&mut scheduler, 1, 8);
    ready_task(&mut scheduler, 2, 4);

    scheduler.ready_remove(1);
    scheduler.tasks[1].priority = 2;
    scheduler.ready_push(1);

    assert_eq!(scheduler.ready_pop(), 1);
    assert_eq!(scheduler.ready_pop(), 2);
}

#[test]
fn cooperative_yield_hands_off_before_requeueing_higher_priority_task() {
    let mut scheduler = Sched::new();
    let current = IDLE_SLOT + 1;
    let next = IDLE_SLOT + 2;
    scheduler.current = current;
    scheduler.tasks[current].state = State::Running;
    scheduler.tasks[current].priority = 2;
    ready_task(&mut scheduler, next, 8);

    assert_eq!(scheduler.take_yield_target(current, 0), Some(next));
    assert_eq!(scheduler.ready_pop(), current);
}

#[test]
fn completed_irq_switch_restores_the_detached_thread_target() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[2].state = State::Ready;
    scheduler.tasks[2].priority = 4;

    assert!(scheduler.recover_completed_switch_request(0, 2));
    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.diagnostics.switch_race_recoveries, 1);
}

#[test]
fn pending_thread_switch_is_not_mistaken_for_a_completed_irq_switch() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Ready;
    scheduler.tasks[2].state = State::Ready;

    assert!(!scheduler.recover_completed_switch_request(0, 2));
    assert_eq!(scheduler.ready_pop(), NIL);
    assert_eq!(scheduler.diagnostics.switch_race_recoveries, 0);
}

#[test]
fn completed_switch_recovery_does_not_duplicate_an_already_requeued_target() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;
    scheduler.make_ready(2, 0);

    assert!(scheduler.recover_completed_switch_request(0, 2));
    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.ready_pop(), NIL);
}

#[test]
fn completed_switch_recovery_keeps_idle_out_of_ready_queues() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[IDLE_SLOT].state = State::Ready;

    assert!(scheduler.recover_completed_switch_request(0, IDLE_SLOT));
    assert_eq!(scheduler.ready_pop(), NIL);
    assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT);
}

#[test]
fn preemptive_ready_queue_uses_priority_then_fifo() {
    let mut scheduler = Sched::new();
    ready_task(&mut scheduler, 1, 8);
    ready_task(&mut scheduler, 2, 4);
    ready_task(&mut scheduler, 3, 2);

    assert_eq!(scheduler.ready_pop(), 3);
    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.ready_pop(), 1);
}

#[test]
fn all_run_policies_use_effective_priority_then_fifo() {
    let spec =
        BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
    let mut scheduler = Sched::new();
    scheduler.tasks[1].state = State::Ready;
    scheduler.tasks[1].priority = 20;
    scheduler.tasks[1].run_policy = RunPolicy::Cooperative;
    scheduler.ready_push(1);
    scheduler.tasks[2].state = State::Ready;
    scheduler.tasks[2].priority = 2;
    scheduler.tasks[2].run_policy = RunPolicy::Budgeted(spec);
    scheduler.ready_push(2);
    scheduler.tasks[3].state = State::Ready;
    scheduler.tasks[3].priority = 2;
    scheduler.tasks[3].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(1).unwrap(),
    };
    scheduler.ready_push(3);

    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.ready_pop(), 3);
    assert_eq!(scheduler.ready_pop(), 1);
}

#[test]
fn policy_change_releases_a_throttled_task() {
    let spec =
        BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
    let mut scheduler = Sched::new();
    scheduler.tasks[2].state = State::Throttled;
    scheduler.tasks[2].run_policy = RunPolicy::Budgeted(spec);
    scheduler.tasks[2].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 10);

    scheduler.set_run_policy(2, RunPolicy::Cooperative, 12);

    assert_eq!(scheduler.tasks[2].state, State::Ready);
    assert_eq!(scheduler.tasks[2].run_policy, RunPolicy::Cooperative);
    assert_eq!(scheduler.ready_pop(), 2);
}

#[test]
fn exited_stacks_are_retired_for_later_reclamation() {
    let mut scheduler = Sched::new();
    scheduler.retire_stack(0x1000);
    scheduler.retire_stack(0x2000);

    assert_eq!(scheduler.retired_count, 2);
    assert_eq!(&scheduler.retired_stacks[..2], &[0x1000, 0x2000]);
}

#[test]
fn scheduler_lock_is_nested_and_rejects_unbalanced_unlock() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.lock_current(10).unwrap();
    scheduler.lock_current(11).unwrap();
    assert_eq!(scheduler.tasks[0].scheduler_lock_depth, 2);
    scheduler.unlock_current(12).unwrap();
    scheduler.unlock_current(13).unwrap();
    assert_eq!(
        scheduler.unlock_current(14),
        Err(DriverError::InvalidContext)
    );
}

#[test]
fn task_metrics_account_dispatch_cpu_and_ready_latency() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].metrics.on_dispatch(100);
    scheduler.make_ready(1, 103);

    scheduler.account_switch(0, 1, 110);
    scheduler.current = 1;
    scheduler.tasks[1].state = State::Running;

    assert_eq!(scheduler.tasks[0].metrics.cpu_time_ms, 10);
    assert_eq!(scheduler.tasks[0].metrics.max_continuous_run_ms, 10);
    assert_eq!(scheduler.tasks[1].metrics.dispatches, 1);
    assert_eq!(scheduler.tasks[1].metrics.max_ready_latency_ms, 7);

    let mut snapshot = [TaskDiagnostic::default(); TASK_SLOT_COUNT];
    assert_eq!(
        scheduler.task_diagnostics(&mut snapshot, 115),
        TASK_SLOT_COUNT
    );
    assert_eq!(snapshot[1].cpu_time_ms, 5);
    assert_eq!(snapshot[1].max_continuous_run_ms, 5);
}

#[test]
fn task_metrics_measure_outermost_scheduler_lock_interval() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;

    scheduler.lock_current(20).unwrap();
    scheduler.lock_current(22).unwrap();

    let mut snapshot = [TaskDiagnostic::default(); TASK_SLOT_COUNT];
    scheduler.task_diagnostics(&mut snapshot, 27);
    assert_eq!(snapshot[0].scheduler_lock_entries, 1);
    assert_eq!(snapshot[0].max_scheduler_lock_ms, 7);

    scheduler.unlock_current(28).unwrap();
    scheduler.unlock_current(29).unwrap();
    scheduler.task_diagnostics(&mut snapshot, 40);
    assert_eq!(snapshot[0].scheduler_lock_entries, 1);
    assert_eq!(snapshot[0].max_scheduler_lock_ms, 9);
}

#[test]
fn task_metrics_attribute_outermost_irq_span_to_interrupted_task() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;

    scheduler.interrupt_enter(30);
    scheduler.interrupt_exit(34);

    let metrics = &scheduler.tasks[0].metrics;
    assert_eq!(metrics.irq_entries, 1);
    assert_eq!(metrics.irq_time_ms, 4);
    assert_eq!(metrics.max_irq_span_ms, 4);
}

#[test]
fn outermost_scheduler_unlock_releases_pending_higher_priority_task() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 10;
    scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(1).unwrap(),
    };
    ready_task(&mut scheduler, 1, 4);
    scheduler.lock_current(0).unwrap();
    scheduler.lock_current(0).unwrap();

    assert_eq!(
        scheduler.unlock_current_and_take_preemption(0).unwrap(),
        None
    );
    assert_eq!(
        scheduler.unlock_current_and_take_preemption(0).unwrap(),
        Some((0, 1))
    );
    assert!(matches!(scheduler.tasks[0].state, State::Ready));
}

#[test]
fn irq_epilogue_preempts_only_after_outermost_interrupt_exit() {
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 10;
    scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(1).unwrap(),
    };
    ready_task(&mut scheduler, 1, 4);

    assert_eq!(scheduler.take_irq_epilogue_target(1, 0), None);
    assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), Some((0, 1)));
    assert_eq!(scheduler.diagnostics.irq_preemptions, 1);
}

#[test]
fn cooperative_task_is_not_preempted_by_irq_but_can_yield() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 10;
    ready_task(&mut scheduler, 1, 4);

    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), None);
    scheduler.started = true;
    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), None);
    assert_eq!(scheduler.take_yield_target(0, 0), Some(1));
    assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
}

#[test]
fn expired_time_slice_round_robins_equal_priority_tasks() {
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 4;
    scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(5).unwrap(),
    };
    ready_task(&mut scheduler, 1, 4);

    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), None);
    scheduler.time_slice_pending = true;
    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), Some((0, 1)));
    assert_eq!(scheduler.diagnostics.time_slice_preemptions, 1);
    assert!(!scheduler.time_slice_pending);
}

#[test]
fn scheduler_lock_preserves_expired_time_slice_until_unlock() {
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 4;
    scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(5).unwrap(),
    };
    ready_task(&mut scheduler, 1, 4);
    scheduler.time_slice_pending = true;
    scheduler.lock_current(100).unwrap();

    assert_eq!(scheduler.take_irq_epilogue_target(0, 0), None);
    assert!(scheduler.time_slice_pending);
    assert_eq!(
        scheduler.unlock_current_and_take_preemption(0).unwrap(),
        Some((0, 1))
    );
    assert!(!scheduler.time_slice_pending);
}

#[test]
fn budget_exhaustion_removes_task_until_replenishment() {
    let spec =
        BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.current = 0;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 2;
    scheduler.tasks[0].run_policy = RunPolicy::Budgeted(spec);
    scheduler.tasks[0].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 100);
    scheduler.tasks[0].budget.on_dispatch(100);
    ready_task(&mut scheduler, 1, 20);

    assert_eq!(scheduler.on_timer(105, NonZeroU32::new(100).unwrap()), None);
    assert_eq!(scheduler.tasks[0].state, State::Throttled);
    assert_eq!(scheduler.diagnostics.budget_exhaustions, 1);
    assert_eq!(scheduler.tasks[0].metrics.budget_exhaustions, 1);
    assert_eq!(scheduler.take_irq_epilogue_target(0, 105), Some((0, 1)));

    scheduler.replenish_budgets(119);
    assert_eq!(scheduler.tasks[0].state, State::Throttled);
    scheduler.replenish_budgets(120);
    assert_eq!(scheduler.tasks[0].state, State::Ready);
    assert_eq!(scheduler.diagnostics.budget_replenishments, 1);
}

#[test]
fn scheduler_lock_defers_but_cannot_cancel_budget_throttle() {
    let spec =
        BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].priority = 2;
    scheduler.tasks[0].run_policy = RunPolicy::Budgeted(spec);
    scheduler.tasks[0].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 100);
    scheduler.tasks[0].budget.on_dispatch(100);
    ready_task(&mut scheduler, 1, 20);
    scheduler.lock_current(100).unwrap();

    assert_eq!(scheduler.on_timer(105, NonZeroU32::new(100).unwrap()), None);
    assert_eq!(scheduler.tasks[0].state, State::Running);
    assert_eq!(scheduler.take_irq_epilogue_target(0, 105), None);
    assert_eq!(scheduler.diagnostics.budget_lock_overruns, 1);

    assert_eq!(
        scheduler.unlock_current_and_take_preemption(106).unwrap(),
        Some((0, 1))
    );
    assert_eq!(scheduler.tasks[0].state, State::Throttled);
    assert_eq!(scheduler.tasks[0].budget.replenishes_at(), 120);
}

#[test]
fn scheduler_lock_limit_is_a_timer_deadline_and_fail_stop_violation() {
    let mut scheduler = Sched::new();
    scheduler.started = true;
    scheduler.tasks[0].state = State::Running;
    scheduler.lock_current(100).unwrap();
    let limit = NonZeroU32::new(10).unwrap();

    assert_eq!(scheduler.scheduler_lock_deadline(limit), Some(110));
    assert_eq!(scheduler.on_timer(109, limit), None);
    assert_eq!(
        scheduler.on_timer(110, limit),
        Some(ContractViolation::SchedulerLockOverrun {
            task_slot: 0,
            held_ms: 10,
            limit_ms: 10,
        })
    );
    assert_eq!(scheduler.diagnostics.scheduler_lock_overruns, 1);
}

#[test]
fn task_identity_generation_rejects_stale_slot_handle() {
    let stale = encode_task_id(3, 7).unwrap();
    assert_eq!(decode_task_id(stale), Ok((3, 7)));
    let replacement = encode_task_id(3, 8).unwrap();
    assert_ne!(stale, replacement);
    let last_slot = TASK_SLOT_COUNT - 1;
    assert_eq!(
        decode_task_id(encode_task_id(last_slot, 1).unwrap()),
        Ok((last_slot, 1))
    );
    assert_eq!(
        encode_task_id(TASK_SLOT_COUNT, 1),
        Err(DriverError::InvalidHandle)
    );
    assert_eq!(encode_task_id(0, 0), Err(DriverError::InvalidHandle));
}

#[test]
fn earliest_deadline_ignores_forever_waiters() {
    let mut scheduler = Sched::new();
    scheduler.tasks[1].state = State::Blocked;
    scheduler.tasks[1].wake_at = 0;
    scheduler.tasks[2].state = State::Sleeping;
    scheduler.tasks[2].wake_at = 42;
    scheduler.tasks[3].state = State::Blocked;
    scheduler.tasks[3].wake_at = 17;

    assert_eq!(scheduler.earliest_wake_deadline(), Some(17));
}

#[test]
fn shared_timer_uses_earliest_rtos_slice_or_embassy_deadline() {
    assert_eq!(
        earliest_deadline(Some(30), Some(20), Some(15), Some(12), Some(10)),
        Some(10)
    );
    assert_eq!(
        earliest_deadline(Some(30), Some(20), None, Some(18), None),
        Some(18)
    );
    assert_eq!(
        earliest_deadline(Some(30), None, Some(25), None, Some(40)),
        Some(25)
    );
    assert_eq!(
        earliest_deadline(None, None, None, None, Some(40)),
        Some(40)
    );
    assert_eq!(earliest_deadline(None, None, None, None, None), None);
}

#[test]
fn stale_timer_programming_ticket_requires_retry() {
    let generation = Cell::new(0);
    let older = claim_timer_rearm_generation(&generation);
    let newer = claim_timer_rearm_generation(&generation);

    assert_ne!(older, newer);
    assert_ne!(generation.get(), older);
    assert_eq!(generation.get(), newer);

    let retry = claim_timer_rearm_generation(&generation);
    assert_eq!(generation.get(), retry);
}

#[test]
fn unrelated_deadline_rearm_does_not_postpone_time_slice() {
    let mut scheduler = Sched::new();
    ready_task(&mut scheduler, 1, 4);
    scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
        time_slice: NonZeroU32::new(5).unwrap(),
    };
    scheduler.tasks[0].priority = 4;

    assert_eq!(scheduler.next_time_slice_deadline(10), Some(15));
    assert_eq!(scheduler.next_time_slice_deadline(12), Some(15));

    scheduler.time_slice_deadline = 0;
    assert_eq!(scheduler.next_time_slice_deadline(15), Some(20));
    scheduler.ready_pop();
    assert_eq!(scheduler.next_time_slice_deadline(16), None);
}

#[test]
fn forever_semaphore_wait_is_not_treated_as_an_expired_deadline() {
    let semaphore = Semaphore::new(0);
    let mut scheduler = Sched::new();
    scheduler.tasks[1].state = State::Blocked;
    scheduler.tasks[1].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
    scheduler.tasks[1].wake_at = 0;
    unsafe {
        (*semaphore.inner.get()).wait_head = 1;
        (*semaphore.inner.get()).wait_tail = 1;
    }

    scheduler.wake_sleepers(1_000);

    assert!(matches!(scheduler.tasks[1].state, State::Blocked));
    assert_eq!(unsafe { (*semaphore.inner.get()).wait_head }, 1);
    assert_eq!(scheduler.diagnostics.semaphore_timeouts, 0);
}

#[test]
fn timed_semaphore_wait_wakes_only_after_its_deadline() {
    let semaphore = Semaphore::new(0);
    let mut scheduler = Sched::new();
    let waiter = IDLE_SLOT + 1;
    scheduler.tasks[waiter].state = State::Blocked;
    scheduler.tasks[waiter].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
    scheduler.tasks[waiter].wake_at = 10;
    unsafe {
        (*semaphore.inner.get()).wait_head = waiter;
        (*semaphore.inner.get()).wait_tail = waiter;
    }

    scheduler.wake_sleepers(9);
    assert!(matches!(scheduler.tasks[waiter].state, State::Blocked));
    scheduler.wake_sleepers(10);

    assert!(matches!(scheduler.tasks[waiter].state, State::Ready));
    assert_eq!(scheduler.ready_pop(), waiter);
    assert_eq!(scheduler.diagnostics.semaphore_timeouts, 1);
}

#[test]
fn semaphore_with_waiters_cannot_be_destroyed() {
    let semaphore = Semaphore::new(0);
    assert!(!super::driver::semaphore_has_waiters(&semaphore));

    // SAFETY: this test has exclusive access to the local semaphore and only
    // constructs the minimum intrusive-list state inspected by destroy.
    unsafe {
        (*semaphore.inner.get()).wait_head = IDLE_SLOT + 1;
        (*semaphore.inner.get()).wait_tail = IDLE_SLOT + 1;
    }
    assert!(super::driver::semaphore_has_waiters(&semaphore));
}

#[test]
fn semaphore_waiters_are_priority_fifo_and_reorder_on_priority_change() {
    let semaphore = Semaphore::new(0);
    let mut scheduler = Sched::new();
    let low = IDLE_SLOT + 1;
    let high_a = IDLE_SLOT + 2;
    let high_b = IDLE_SLOT + 3;

    for (task, priority) in [(low, 10), (high_a, 2), (high_b, 2)] {
        scheduler.tasks[task].state = State::Blocked;
        scheduler.tasks[task].base_priority = priority;
        scheduler.tasks[task].priority = priority;
        scheduler.tasks[task].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
        unsafe { enqueue_waiter(&mut scheduler, &mut *semaphore.inner.get(), task) };
    }

    let state = unsafe { &*semaphore.inner.get() };
    assert_eq!(state.wait_head, high_a);
    assert_eq!(scheduler.tasks[high_a].next, high_b);
    assert_eq!(scheduler.tasks[high_b].next, low);

    scheduler.set_effective_priority(low, 1);
    let state = unsafe { &*semaphore.inner.get() };
    assert_eq!(state.wait_head, low);
    assert_eq!(scheduler.tasks[low].next, high_a);

    let granted =
        unsafe { release_semaphore_locked(&mut scheduler, &mut *semaphore.inner.get(), 0) };
    assert_eq!(granted, low);
    assert!(scheduler.tasks[low].sem_granted);
}

#[test]
fn duplicate_mutex_waiters_keep_owner_inherited_until_both_leave() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].base_priority = 20;
    scheduler.tasks[0].priority = 20;

    scheduler.add_inheritance(0, 2);
    scheduler.add_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 2);
    scheduler.remove_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 2);
    scheduler.remove_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 20);
}

#[test]
fn chained_mutex_inheritance_propagates_effective_priority() {
    let mut scheduler = Sched::new();
    let upstream = RtosMutex::new();
    let downstream = RtosMutex::new();

    scheduler.tasks[0].state = State::Blocked;
    scheduler.tasks[0].base_priority = 20;
    scheduler.tasks[0].priority = 20;
    scheduler.tasks[0].waiting_mutex = core::ptr::addr_of!(upstream) as usize;
    scheduler.tasks[1].state = State::Running;
    scheduler.tasks[1].base_priority = 30;
    scheduler.tasks[1].priority = 30;
    unsafe {
        (*upstream.inner.get()).owner = 1;
        (*upstream.inner.get()).wait_head = 0;
        (*upstream.inner.get()).wait_tail = 0;
        (*downstream.inner.get()).owner = 0;
    }
    scheduler.add_inheritance(1, 20);

    scheduler.add_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 2);
    assert_eq!(scheduler.tasks[1].priority, 2);

    scheduler.remove_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 20);
    assert_eq!(scheduler.tasks[1].priority, 20);
}

#[test]
fn timed_out_mutex_waiter_restores_owner_priority() {
    let mut scheduler = Sched::new();
    let mutex = RtosMutex::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].base_priority = 20;
    scheduler.tasks[0].priority = 20;
    scheduler.tasks[1].state = State::Blocked;
    scheduler.tasks[1].base_priority = 2;
    scheduler.tasks[1].priority = 2;
    scheduler.tasks[1].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    scheduler.tasks[1].wake_at = 10;
    unsafe {
        (*mutex.inner.get()).owner = 0;
        (*mutex.inner.get()).depth = 1;
        (*mutex.inner.get()).wait_head = 1;
        (*mutex.inner.get()).wait_tail = 1;
    }
    scheduler.add_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 2);

    scheduler.wake_sleepers(10);
    assert_eq!(scheduler.tasks[0].priority, 20);
    assert_eq!(scheduler.tasks[1].state, State::Ready);
    assert_eq!(scheduler.tasks[1].waiting_mutex, 0);
    assert_eq!(unsafe { (*mutex.inner.get()).wait_head }, NIL);
}

#[test]
fn mutex_handoff_transfers_remaining_inheritance_to_new_owner() {
    let mut scheduler = Sched::new();
    let mutex = RtosMutex::new();

    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].base_priority = 20;
    scheduler.tasks[0].priority = 20;
    scheduler.tasks[1].state = State::Blocked;
    scheduler.tasks[1].base_priority = 2;
    scheduler.tasks[1].priority = 2;
    scheduler.tasks[1].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    scheduler.tasks[2].state = State::Blocked;
    scheduler.tasks[2].base_priority = 5;
    scheduler.tasks[2].priority = 5;
    scheduler.tasks[2].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    unsafe {
        let state = &mut *mutex.inner.get();
        state.owner = 0;
        state.depth = 0;
        enqueue_mutex_waiter(&mut scheduler, state, 1);
        enqueue_mutex_waiter(&mut scheduler, state, 2);
    }
    scheduler.add_inheritance(0, 2);
    scheduler.add_inheritance(0, 5);

    unsafe { release_mutex_locked(&mut scheduler, &mut *mutex.inner.get(), 0, 0) };

    let state = unsafe { &*mutex.inner.get() };
    assert_eq!(state.owner, 1);
    assert_eq!(state.wait_head, 2);
    assert_eq!(scheduler.tasks[0].priority, 20);
    assert_eq!(scheduler.tasks[1].priority, 2);
    assert_eq!(scheduler.tasks[1].inherited_waiters[5], 1);
    assert_eq!(scheduler.tasks[1].state, State::Ready);
    assert!(scheduler.tasks[1].sem_granted);
    assert_eq!(scheduler.tasks[1].waiting_mutex, 0);
}

#[test]
fn base_priority_change_preserves_and_then_restores_inheritance() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].base_priority = 20;
    scheduler.tasks[0].priority = 20;

    scheduler.add_inheritance(0, 2);
    scheduler.tasks[0].base_priority = 10;
    scheduler.refresh_inherited_priority(0, 0);
    assert_eq!(scheduler.tasks[0].priority, 2);

    scheduler.remove_inheritance(0, 2);
    assert_eq!(scheduler.tasks[0].priority, 10);
}
