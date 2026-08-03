use super::*;
use hisi_rf_rtos_driver::{TaskResourceGroupRequirements, TaskResourceOwner};

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
    assert_eq!(diagnostics.dynamic_reserved, 0);
    assert_eq!(diagnostics.dynamic_free, 0);
}

#[test]
fn caller_selected_capacity_bounds_dynamic_slots_and_diagnostics() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[IDLE_SLOT].state = State::Ready;

    for dynamic in 0..3 {
        let slot = IDLE_SLOT + 1 + dynamic;
        assert_eq!(scheduler.alloc_dynamic_slot_with_capacity(3), Ok(slot));
        scheduler.tasks[slot].state = State::Ready;
    }
    assert_eq!(
        scheduler.alloc_dynamic_slot_with_capacity(3),
        Err(DriverError::NoTaskSlots)
    );

    let diagnostics = scheduler.diagnostics_with_capacity(3);
    assert_eq!(diagnostics.dynamic_capacity, 3);
    assert_eq!(diagnostics.dynamic_used, 3);
    assert_eq!(diagnostics.dynamic_free, 0);
}

#[test]
fn reservations_protect_promised_slots_from_ordinary_spawns() {
    let mut scheduler = Sched::new();
    let reservation = scheduler
        .reserve_dynamic_slots(NonZeroUsize::new(2).unwrap())
        .unwrap();

    let diagnostics = scheduler.diagnostics();
    assert_eq!(diagnostics.dynamic_used, 0);
    assert_eq!(diagnostics.dynamic_reserved, 2);
    assert_eq!(diagnostics.dynamic_free, 13);

    for _ in 0..13 {
        let slot = scheduler.alloc_dynamic_slot().unwrap();
        scheduler.tasks[slot].state = State::Ready;
    }
    assert_eq!(
        scheduler.alloc_dynamic_slot(),
        Err(DriverError::NoTaskSlots)
    );

    for expected_remaining in [1, 0] {
        let (slot, stack) = scheduler.alloc_reserved_dynamic_slot(&reservation).unwrap();
        assert_eq!(stack, None);
        scheduler.tasks[slot].state = State::Ready;
        assert_eq!(scheduler.diagnostics().dynamic_reserved, expected_remaining);
    }
    assert_eq!(
        scheduler.alloc_reserved_dynamic_slot(&reservation),
        Err(DriverError::NoTaskSlots)
    );
    scheduler.release_task_reservation(&reservation).unwrap();
    assert_eq!(
        scheduler.release_task_reservation(&reservation),
        Err(DriverError::InvalidHandle)
    );
}

#[test]
fn releasing_a_reservation_returns_only_unconsumed_slots() {
    let mut scheduler = Sched::new();
    let old = scheduler
        .reserve_dynamic_slots(NonZeroUsize::new(3).unwrap())
        .unwrap();
    let (slot, stack) = scheduler.alloc_reserved_dynamic_slot(&old).unwrap();
    assert_eq!(stack, None);
    scheduler.tasks[slot].state = State::Ready;
    scheduler.release_task_reservation(&old).unwrap();

    let diagnostics = scheduler.diagnostics();
    assert_eq!(diagnostics.dynamic_used, 1);
    assert_eq!(diagnostics.dynamic_reserved, 0);
    assert_eq!(diagnostics.dynamic_free, 14);
    assert_eq!(
        scheduler.alloc_reserved_dynamic_slot(&old),
        Err(DriverError::InvalidHandle)
    );

    let replacement = scheduler
        .reserve_dynamic_slots(NonZeroUsize::new(14).unwrap())
        .unwrap();
    assert_ne!(old.into_raw(), replacement.into_raw());
    assert_eq!(
        scheduler.reserve_dynamic_slots(NonZeroUsize::new(1).unwrap()),
        Err(TaskAdmissionError::InsufficientTaskSlots {
            required: 1,
            available: 0,
        })
    );
}

#[test]
fn task_resource_reservation_consumes_and_releases_preallocated_stacks() {
    let mut scheduler = Sched::new();
    let requirements = TaskResourceRequirements::new(
        NonZeroUsize::new(3).unwrap(),
        NonZeroUsize::new(24 * 1024).unwrap(),
    )
    .unwrap();
    let mut stacks = [0usize; DYNAMIC_TASK_CAPACITY];
    stacks[..3].copy_from_slice(&[0x1000, 0x2000, 0x3000]);
    let reservation = scheduler
        .reserve_dynamic_task_resources(requirements, stacks)
        .unwrap();

    assert_eq!(
        scheduler.reservation_stack_size(&reservation),
        Ok(Some(24 * 1024))
    );
    let (slot, first) = scheduler.alloc_reserved_dynamic_slot(&reservation).unwrap();
    scheduler.tasks[slot].state = State::Ready;
    assert_eq!(
        first,
        Some(reservation::ReservedStack {
            pointer: 0x1000,
            size: 24 * 1024,
        })
    );

    let released = scheduler.release_task_reservation(&reservation).unwrap();
    assert_eq!(released.count, 2);
    assert_eq!(&released.stacks[..released.count], &[0x2000, 0x3000]);
    assert_eq!(
        scheduler.reservation_stack_size(&reservation),
        Err(DriverError::InvalidHandle)
    );
}

#[test]
fn task_stack_preallocation_rolls_back_every_partial_allocation() {
    let requirements = TaskResourceRequirements::new(
        NonZeroUsize::new(3).unwrap(),
        NonZeroUsize::new(24 * 1024).unwrap(),
    )
    .unwrap();
    let mut allocated = 0usize;
    let mut released = [0usize; 3];
    let mut released_count = 0usize;
    let result = driver::preallocate_task_stacks(
        requirements,
        |_| {
            if allocated == 2 {
                core::ptr::null_mut()
            } else {
                allocated += 1;
                (allocated * 0x1000) as *mut u8
            }
        },
        |pointer| {
            released[released_count] = pointer as usize;
            released_count += 1;
        },
    );

    assert_eq!(
        result,
        Err(TaskAdmissionError::InsufficientTaskStackMemory {
            required: 72 * 1024,
            available: 48 * 1024,
        })
    );
    assert_eq!(released_count, 2);
    assert_eq!(&released[..released_count], &[0x1000, 0x2000]);
}

fn heterogeneous_resource_groups() -> [TaskResourceGroupRequirements; 2] {
    [
        TaskResourceGroupRequirements::new(
            TaskResourceOwner::new(NonZeroU32::new(1).unwrap()),
            TaskResourceRequirements::new(
                NonZeroUsize::new(2).unwrap(),
                NonZeroUsize::new(24 * 1024).unwrap(),
            )
            .unwrap(),
        ),
        TaskResourceGroupRequirements::new(
            TaskResourceOwner::new(NonZeroU32::new(2).unwrap()),
            TaskResourceRequirements::new(
                NonZeroUsize::new(1).unwrap(),
                NonZeroUsize::new(8 * 1024).unwrap(),
            )
            .unwrap(),
        ),
    ]
}

#[test]
fn heterogeneous_resource_plan_reserves_children_in_plan_order() {
    let mut scheduler = Sched::new();
    let groups = heterogeneous_resource_groups();
    let plan = TaskResourcePlan::new(&groups).unwrap();
    let mut stacks = [[0usize; DYNAMIC_TASK_CAPACITY]; TASK_RESOURCE_GROUP_CAPACITY];
    stacks[0][..2].copy_from_slice(&[0x1000, 0x2000]);
    stacks[1][0] = 0x3000;

    let mut batch = scheduler
        .reserve_dynamic_task_resource_plan_with_capacity(plan, stacks, DYNAMIC_TASK_CAPACITY)
        .unwrap();
    assert_eq!(batch.len(), 2);
    assert_eq!(scheduler.diagnostics().dynamic_reserved, 3);

    let vendor = batch.take(0).unwrap();
    let worker = batch.take(1).unwrap();
    assert_eq!(
        scheduler.reservation_stack_size(&vendor),
        Ok(Some(24 * 1024))
    );
    assert_eq!(
        scheduler.reservation_stack_size(&worker),
        Ok(Some(8 * 1024))
    );
    assert_eq!(
        scheduler.release_task_reservation(&vendor).unwrap().count,
        2
    );
    assert_eq!(
        scheduler.release_task_reservation(&worker).unwrap().count,
        1
    );
    assert_eq!(scheduler.diagnostics().dynamic_reserved, 0);
}

#[test]
fn heterogeneous_slot_failure_leaves_no_partial_reservation() {
    let mut scheduler = Sched::new();
    let groups = heterogeneous_resource_groups();
    let plan = TaskResourcePlan::new(&groups).unwrap();
    let mut stacks = [[0usize; DYNAMIC_TASK_CAPACITY]; TASK_RESOURCE_GROUP_CAPACITY];
    stacks[0][..2].copy_from_slice(&[0x1000, 0x2000]);
    stacks[1][0] = 0x3000;

    assert!(matches!(
        scheduler.reserve_dynamic_task_resource_plan_with_capacity(plan, stacks, 2),
        Err(TaskAdmissionError::InsufficientTaskGroupSlots {
            owner,
            required: 1,
            available: 0,
        }) if owner == groups[1].owner()
    ));
    assert_eq!(scheduler.diagnostics_with_capacity(2).dynamic_reserved, 0);
}

#[test]
fn heterogeneous_stack_failure_rolls_back_every_prior_group() {
    let groups = heterogeneous_resource_groups();
    let plan = TaskResourcePlan::new(&groups).unwrap();
    let mut allocation = 0usize;
    let mut released = [0usize; 3];
    let mut released_count = 0usize;

    let result = driver::preallocate_task_resource_plan(
        plan,
        |_| {
            if allocation == 2 {
                core::ptr::null_mut()
            } else {
                allocation += 1;
                (allocation * 0x1000) as *mut u8
            }
        },
        |pointer| {
            released[released_count] = pointer as usize;
            released_count += 1;
        },
        || 7 * 1024,
    );

    assert!(matches!(
        result,
        Err(TaskAdmissionError::InsufficientTaskGroupStackMemory {
            owner,
            required,
            available,
            largest_contiguous,
        }) if owner == groups[1].owner()
            && required == 8 * 1024
            && available == 0
            && largest_contiguous == 7 * 1024
    ));
    assert_eq!(released_count, 2);
    assert_eq!(&released[..released_count], &[0x1000, 0x2000]);
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
fn switch_intent_is_committed_and_consumed_exactly_once() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].identity_generation = 1;
    scheduler.tasks[0].state = State::Ready;
    scheduler.tasks[2].identity_generation = 7;
    scheduler.tasks[2].state = State::Ready;

    let intent = scheduler.prepare_switch_intent(0, 2);
    assert!(scheduler.commit_switch_intent(intent));
    assert_eq!(scheduler.consume_pending_switch(), Some((0, 2)));
    assert_eq!(scheduler.consume_pending_switch(), None);
    assert_eq!(scheduler.diagnostics.switch_intents_created, 1);
    assert_eq!(scheduler.diagnostics.switch_intents_committed, 1);
    assert_eq!(scheduler.diagnostics.switch_intents_completed, 1);
}

#[test]
fn stale_switch_intent_restores_its_detached_target() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].identity_generation = 1;
    scheduler.tasks[0].state = State::Ready;
    scheduler.tasks[2].identity_generation = 7;
    scheduler.tasks[2].state = State::Ready;

    let intent = scheduler.prepare_switch_intent(0, 2);
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].resume_generation = 1;

    assert!(!scheduler.commit_switch_intent(intent));
    assert_eq!(scheduler.ready_pop(), 2);
    assert_eq!(scheduler.diagnostics.switch_intents_cancelled_stale, 1);
    assert_eq!(scheduler.diagnostics.switch_race_recoveries, 1);
}

#[test]
fn switch_intent_rejects_reused_task_identity() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].identity_generation = 1;
    scheduler.tasks[0].state = State::Ready;
    scheduler.tasks[2].identity_generation = 7;
    scheduler.tasks[2].state = State::Ready;

    let intent = scheduler.prepare_switch_intent(0, 2);
    scheduler.tasks[2].identity_generation = 8;

    assert!(!scheduler.commit_switch_intent(intent));
    assert!(scheduler.pending_switch.is_none());
    assert_eq!(scheduler.diagnostics.switch_intents_cancelled_identity, 1);
}

#[test]
fn switch_intent_source_identity_failure_restores_target() {
    let mut scheduler = Sched::new();
    scheduler.current = 0;
    scheduler.tasks[0].identity_generation = 1;
    scheduler.tasks[0].state = State::Ready;
    scheduler.tasks[2].identity_generation = 7;
    scheduler.tasks[2].state = State::Ready;

    let intent = scheduler.prepare_switch_intent(0, 2);
    scheduler.tasks[0].identity_generation = 2;

    assert!(!scheduler.commit_switch_intent(intent));
    assert_eq!(scheduler.ready_pop(), 2);
    assert!(scheduler.pending_switch.is_none());
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
fn task_diagnostics_report_dynamic_stack_allocation_size() {
    let mut scheduler = Sched::new();
    scheduler.tasks[0].state = State::Running;
    scheduler.tasks[0].stack_size = 0;
    scheduler.tasks[2].state = State::Ready;
    scheduler.tasks[2].stack = 0x1000;
    scheduler.tasks[2].stack_size = 24 * 1024;

    let mut snapshot = [TaskDiagnostic::default(); TASK_SLOT_COUNT];
    scheduler.task_diagnostics(&mut snapshot, 0);

    assert_eq!(snapshot[0].stack_size, 0);
    assert_eq!(snapshot[2].stack_size, 24 * 1024);

    scheduler.tasks[2] = Tcb::empty();
    scheduler.task_diagnostics(&mut snapshot, 0);
    assert_eq!(snapshot[2].stack_size, 0);
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
    // SAFETY: this test has exclusive access to the local semaphore.
    let state = unsafe { &mut *semaphore.inner.get() };
    assert!(!super::driver::semaphore_state_has_waiters(state));

    state.wait_head = IDLE_SLOT + 1;
    state.wait_tail = IDLE_SLOT + 1;
    assert!(super::driver::semaphore_state_has_waiters(state));
}

#[test]
fn owned_or_waited_mutex_cannot_be_destroyed() {
    let mutex = RtosMutex::new();
    // SAFETY: this test has exclusive access to the local mutex.
    let state = unsafe { &mut *mutex.inner.get() };
    assert!(!super::driver::mutex_state_is_busy(state));

    state.owner = IDLE_SLOT + 1;
    assert!(super::driver::mutex_state_is_busy(state));
    state.owner = NIL;

    state.wait_head = IDLE_SLOT + 2;
    state.wait_tail = IDLE_SLOT + 2;
    assert!(super::driver::mutex_state_is_busy(state));
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
fn cancelling_queued_semaphore_wait_makes_task_ready_without_minting_a_count() {
    let semaphore = Semaphore::new(0);
    let mut scheduler = Sched::new();
    let waiter = IDLE_SLOT + 1;
    scheduler.tasks[waiter].state = State::Blocked;
    scheduler.tasks[waiter].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
    unsafe { enqueue_waiter(&mut scheduler, &mut *semaphore.inner.get(), waiter) };

    assert_eq!(
        cancel_wait_locked(&mut scheduler, waiter, 7),
        WaitCancellationOutcome::Cancelled
    );
    let state = unsafe { &*semaphore.inner.get() };
    assert_eq!(state.count, 0);
    assert_eq!(state.wait_head, NIL);
    assert_eq!(scheduler.tasks[waiter].waiting_sem, 0);
    assert!(matches!(scheduler.tasks[waiter].state, State::Ready));
    assert_eq!(scheduler.ready_pop(), waiter);
}

#[test]
fn cancelling_semaphore_handoff_returns_exactly_one_count() {
    let semaphore = Semaphore::new(0);
    let mut scheduler = Sched::new();
    let waiter = IDLE_SLOT + 1;
    scheduler.tasks[waiter].state = State::Blocked;
    scheduler.tasks[waiter].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
    unsafe { enqueue_waiter(&mut scheduler, &mut *semaphore.inner.get(), waiter) };
    unsafe { release_semaphore_locked(&mut scheduler, &mut *semaphore.inner.get(), 3) };

    assert_eq!(
        cancel_wait_locked(&mut scheduler, waiter, 4),
        WaitCancellationOutcome::Cancelled
    );
    let state = unsafe { &*semaphore.inner.get() };
    assert_eq!(state.count, 1);
    assert_eq!(scheduler.tasks[waiter].granted_sem, 0);
    assert!(!scheduler.tasks[waiter].sem_granted);
}

#[test]
fn cancelling_mutex_wait_restores_owner_priority() {
    let mutex = RtosMutex::new();
    let mut scheduler = Sched::new();
    let owner = 0;
    let waiter = IDLE_SLOT + 1;
    scheduler.tasks[owner].state = State::Running;
    scheduler.tasks[owner].base_priority = 20;
    scheduler.tasks[owner].priority = 20;
    scheduler.tasks[waiter].state = State::Blocked;
    scheduler.tasks[waiter].priority = 2;
    scheduler.tasks[waiter].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    let state = unsafe { &mut *mutex.inner.get() };
    state.owner = owner;
    state.depth = 1;
    enqueue_mutex_waiter(&mut scheduler, state, waiter);
    scheduler.add_inheritance(owner, 2);

    assert_eq!(
        cancel_wait_locked(&mut scheduler, waiter, 9),
        WaitCancellationOutcome::Cancelled
    );
    assert_eq!(state.wait_head, NIL);
    assert_eq!(scheduler.tasks[owner].priority, 20);
    assert!(matches!(scheduler.tasks[waiter].state, State::Ready));
}

#[test]
fn cancelling_mutex_handoff_releases_unconsumed_ownership() {
    let mutex = RtosMutex::new();
    let mut scheduler = Sched::new();
    let owner = 0;
    let waiter = IDLE_SLOT + 1;
    scheduler.tasks[owner].state = State::Running;
    scheduler.tasks[owner].base_priority = 20;
    scheduler.tasks[owner].priority = 20;
    scheduler.tasks[waiter].state = State::Blocked;
    scheduler.tasks[waiter].priority = 2;
    scheduler.tasks[waiter].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    let state = unsafe { &mut *mutex.inner.get() };
    state.owner = owner;
    state.depth = 1;
    enqueue_mutex_waiter(&mut scheduler, state, waiter);
    scheduler.add_inheritance(owner, 2);
    release_mutex_locked(&mut scheduler, state, owner, 10);

    assert_eq!(state.owner, waiter);
    assert_eq!(
        cancel_wait_locked(&mut scheduler, waiter, 11),
        WaitCancellationOutcome::Cancelled
    );
    assert_eq!(state.owner, NIL);
    assert_eq!(state.depth, 0);
    assert_eq!(scheduler.tasks[waiter].granted_mutex, 0);
    assert!(!scheduler.tasks[waiter].sem_granted);
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
fn equal_priority_mutex_waiters_handoff_in_fifo_order() {
    let mut scheduler = Sched::new();
    let mutex = RtosMutex::new();
    let owner = 0;
    let first = IDLE_SLOT + 1;
    let second = first + 1;

    scheduler.tasks[owner].state = State::Running;
    scheduler.tasks[owner].base_priority = 20;
    scheduler.tasks[owner].priority = 20;
    for waiter in [first, second] {
        scheduler.tasks[waiter].state = State::Blocked;
        scheduler.tasks[waiter].base_priority = 2;
        scheduler.tasks[waiter].priority = 2;
        scheduler.tasks[waiter].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
    }
    unsafe {
        let state = &mut *mutex.inner.get();
        state.owner = owner;
        state.depth = 1;
        enqueue_mutex_waiter(&mut scheduler, state, first);
        enqueue_mutex_waiter(&mut scheduler, state, second);
    }
    scheduler.add_inheritance(owner, 2);
    scheduler.add_inheritance(owner, 2);

    unsafe { release_mutex_locked(&mut scheduler, &mut *mutex.inner.get(), owner, 0) };

    let state = unsafe { &*mutex.inner.get() };
    assert_eq!(state.owner, first);
    assert_eq!(state.wait_head, second);
    assert_eq!(state.wait_tail, second);
}

#[test]
fn mutex_cycle_is_rejected_without_mutating_wait_graph() {
    let mut scheduler = Sched::new();
    let outer = RtosMutex::new();
    let inner = RtosMutex::new();
    let low = 0;
    let mid = IDLE_SLOT + 1;

    unsafe {
        (*outer.inner.get()).owner = low;
        (*inner.inner.get()).owner = mid;
    }
    scheduler.tasks[mid].state = State::Blocked;
    scheduler.tasks[mid].waiting_mutex = core::ptr::addr_of!(outer) as usize;

    let outer_owner_before = unsafe { (*outer.inner.get()).owner };
    let inner_owner_before = unsafe { (*inner.inner.get()).owner };
    assert!(sync::mutex_chain_contains(&scheduler, mid, low));
    assert_eq!(unsafe { (*outer.inner.get()).owner }, outer_owner_before);
    assert_eq!(unsafe { (*inner.inner.get()).owner }, inner_owner_before);
    assert_eq!(
        scheduler.tasks[mid].waiting_mutex,
        core::ptr::addr_of!(outer) as usize
    );
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
