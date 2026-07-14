use core::num::NonZeroU32;

use hisi_rtos::{CooperativeOnly, RunPolicy, RuntimeHandle, TaskId};

fn set_preemptive_policy(handle: &RuntimeHandle<CooperativeOnly>, task: TaskId) {
    let _ = handle.set_task_run_policy(
        task,
        RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(1).unwrap(),
        },
    );
}

fn main() {}
