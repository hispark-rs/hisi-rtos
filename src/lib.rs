//! Cooperative task scheduler and radio runtime backend.
//!
//! Modeled on esp-rtos's scheduler (TCB + context switch + ready queue +
//! blocking primitives), adapted for the WS63 app core (single-hart
//! `rv32imfc`). Key difference from esp32c3 (`rv32imc`): WS63 has the **F**
//! extension, so the context switch must also save/restore the callee-saved FP
//! registers `fs0..fs11` (the WiFi blob does floating-point RF math).
//!
//! This is a **cooperative** scheduler: a task runs until it calls
//! [`yield_now`], blocks on a [`Semaphore`], or [`sleep_ms`]s. That matches the
//! vendor WiFi worker-thread model (it waits on semaphores / sleeps). Preemptive
//! time-slicing (a timer ISR driving the switch) is a follow-on; the cooperative
//! core is what the blob's `osal_kthread_*` / `osal_sem_*` / `osal_wait_*` /
//! `osal_msleep` need.
//!
//! Layering: this crate depends only on `core`, `critical-section`, and the
//! runtime-neutral radio driver contract. The application injects allocation
//! and monotonic-time resources before any radio task can start.

#![no_std]

use core::cell::{Cell, RefCell, UnsafeCell};
use core::ffi::c_void;
use core::num::{NonZeroU32, NonZeroUsize};
use critical_section::Mutex;
use hisi_rf_rtos_driver::{
    Error as DriverError, Runtime, SemaphoreHandle, TaskConfig, TaskId, WaitOutcome, WaitTimeout,
};

/// Max concurrent tasks (slot table; the WiFi stack uses only a few).
const MAX_TASKS: usize = 16;
/// LiteOS-compatible priority levels: 0 is highest, 31 is lowest.
const PRIORITY_LEVELS: usize = 32;
/// Sentinel "no task" index for intrusive list links.
const NIL: usize = usize::MAX;

/// Platform services owned by the application rather than the radio stack.
#[derive(Clone, Copy)]
pub struct Resources {
    /// Allocates `size` bytes, returning null on exhaustion.
    pub allocate: unsafe fn(size: usize) -> *mut u8,
    /// Releases an allocation previously returned by [`Resources::allocate`].
    pub deallocate: unsafe fn(pointer: *mut u8),
    /// Returns a wrapping monotonic millisecond counter.
    pub monotonic_ms: fn() -> u64,
}

/// Scheduler configuration fixed before the first task starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Minimum stack allocation applied to every task request.
    pub minimum_stack_size: NonZeroUsize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // The WS63 vendor archive was built with a 24 KiB LiteOS default.
            minimum_stack_size: NonZeroUsize::new(24 * 1024).unwrap(),
        }
    }
}

/// Failure to start the process-wide runtime.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StartError {
    /// This RTOS instance was already started.
    AlreadyStarted,
    /// Another runtime already owns the radio driver contract.
    Driver(DriverError),
}

#[derive(Clone, Copy)]
struct StartState {
    config: Config,
    resources: Resources,
}

static START_STATE: Mutex<Cell<Option<StartState>>> = Mutex::new(Cell::new(None));

fn start_state() -> StartState {
    critical_section::with(|cs| START_STATE.borrow(cs).get())
        .expect("hisi-rtos must be started before radio runtime use")
}

fn allocate(size: usize) -> *mut u8 {
    // SAFETY: the application-provided allocator contract accepts arbitrary
    // non-zero task/control-block sizes and returns null on exhaustion.
    unsafe { (start_state().resources.allocate)(size) }
}

fn deallocate(pointer: *mut u8) {
    // SAFETY: every pointer passed here came from `allocate` and is released
    // exactly once after it is no longer in use.
    unsafe { (start_state().resources.deallocate)(pointer) }
}

// ── Saved CPU context (offsets MUST match `context_switch` asm) ──────────────
#[repr(C)]
#[derive(Clone, Copy)]
struct Ctx {
    ra: usize,      // 0
    sp: usize,      // 4
    s: [usize; 12], // 8..56  (s0..s11)
    fs: [u32; 12],  // 56..104 (fs0..fs11, FLEN=32)
    tp: usize,      // 104
    mstatus: u32,   // 108
    fcsr: u32,      // 112
}
impl Ctx {
    const fn zero() -> Self {
        Ctx {
            ra: 0,
            sp: 0,
            s: [0; 12],
            fs: [0; 12],
            tp: 0,
            mstatus: 0,
            fcsr: 0,
        }
    }
}

/// Cooperative context switch: save callee-saved regs of the current task to
/// `*old`, restore `*new`, return into the new task. Caller-saved regs are
/// spilled by the compiler around this normal call, so only callee-saved
/// (ra, sp, s0-s11, fs0-fs11) need saving.
#[cfg(target_arch = "riscv32")]
#[unsafe(naked)]
unsafe extern "C" fn context_switch(old: *mut Ctx, new: *const Ctx) {
    core::arch::naked_asm!(
        // Enable the F extension for the fsw/flw below (rv32imfc has it, but the
        // inline-asm assembler context defaults to a baseline without F).
        ".option arch, +f",
        // save current -> *old (a0)
        "sw  ra,  0(a0)",
        "sw  sp,  4(a0)",
        "sw  s0,  8(a0)",
        "sw  s1, 12(a0)",
        "sw  s2, 16(a0)",
        "sw  s3, 20(a0)",
        "sw  s4, 24(a0)",
        "sw  s5, 28(a0)",
        "sw  s6, 32(a0)",
        "sw  s7, 36(a0)",
        "sw  s8, 40(a0)",
        "sw  s9, 44(a0)",
        "sw  s10,48(a0)",
        "sw  s11,52(a0)",
        "fsw fs0, 56(a0)",
        "fsw fs1, 60(a0)",
        "fsw fs2, 64(a0)",
        "fsw fs3, 68(a0)",
        "fsw fs4, 72(a0)",
        "fsw fs5, 76(a0)",
        "fsw fs6, 80(a0)",
        "fsw fs7, 84(a0)",
        "fsw fs8, 88(a0)",
        "fsw fs9, 92(a0)",
        "fsw fs10,96(a0)",
        "fsw fs11,100(a0)",
        "sw  tp, 104(a0)",
        "csrr t0, mstatus",
        "sw  t0, 108(a0)",
        "frcsr t0",
        "sw  t0, 112(a0)",
        // restore *new (a1) -> current
        "lw  ra,  0(a1)",
        "lw  sp,  4(a1)",
        "lw  s0,  8(a1)",
        "lw  s1, 12(a1)",
        "lw  s2, 16(a1)",
        "lw  s3, 20(a1)",
        "lw  s4, 24(a1)",
        "lw  s5, 28(a1)",
        "lw  s6, 32(a1)",
        "lw  s7, 36(a1)",
        "lw  s8, 40(a1)",
        "lw  s9, 44(a1)",
        "lw  s10,48(a1)",
        "lw  s11,52(a1)",
        "flw fs0, 56(a1)",
        "flw fs1, 60(a1)",
        "flw fs2, 64(a1)",
        "flw fs3, 68(a1)",
        "flw fs4, 72(a1)",
        "flw fs5, 76(a1)",
        "flw fs6, 80(a1)",
        "flw fs7, 84(a1)",
        "flw fs8, 88(a1)",
        "flw fs9, 92(a1)",
        "flw fs10,96(a1)",
        "flw fs11,100(a1)",
        "lw  tp, 104(a1)",
        "lw  t0, 112(a1)",
        "fscsr t0",
        // Restore mstatus last: setting MIE before the remaining context is
        // live would allow an interrupt to observe a half-restored task.
        "lw  t0, 108(a1)",
        "csrw mstatus, t0",
        "ret",
    )
}

#[cfg(not(target_arch = "riscv32"))]
unsafe extern "C" fn context_switch(_old: *mut Ctx, _new: *const Ctx) {
    unreachable!("WS63 context switching is only available on riscv32");
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Free,
    Ready,
    Running,
    Blocked,
    Sleeping,
}

/// Task entry signature (matches the OSAL `osal_kthread_func`).
type TaskFn = extern "C" fn(*mut c_void) -> *mut c_void;

struct Tcb {
    ctx: Ctx,
    state: State,
    stack: usize, // heap allocation addr to free on exit (0 for the main task)
    entry: Option<TaskFn>,
    arg: usize,   // task argument (*mut c_void stored as usize so Tcb is Send)
    next: usize,  // intrusive link: ready queue OR one wait queue
    wake_at: u64, // mask-ROM systick millisecond deadline
    waiting_sem: usize,
    sem_granted: bool,
    priority: u8,
}
impl Tcb {
    const fn empty() -> Self {
        Tcb {
            ctx: Ctx::zero(),
            state: State::Free,
            stack: 0,
            entry: None,
            arg: 0,
            next: NIL,
            wake_at: 0,
            waiting_sem: 0,
            sem_granted: false,
            priority: (PRIORITY_LEVELS - 1) as u8,
        }
    }
}

struct Sched {
    tasks: [Tcb; MAX_TASKS],
    current: usize,
    ready_head: [usize; PRIORITY_LEVELS],
    ready_tail: [usize; PRIORITY_LEVELS],
    retired_stacks: [usize; MAX_TASKS],
    retired_count: usize,
    started: bool,
}
impl Sched {
    const fn new() -> Self {
        const E: Tcb = Tcb::empty();
        Sched {
            tasks: [E; MAX_TASKS],
            current: 0,
            ready_head: [NIL; PRIORITY_LEVELS],
            ready_tail: [NIL; PRIORITY_LEVELS],
            retired_stacks: [0; MAX_TASKS],
            retired_count: 0,
            started: false,
        }
    }
    fn ready_push(&mut self, i: usize) {
        let priority = self.tasks[i].priority as usize;
        self.tasks[i].next = NIL;
        if self.ready_tail[priority] == NIL {
            self.ready_head[priority] = i;
        } else {
            self.tasks[self.ready_tail[priority]].next = i;
        }
        self.ready_tail[priority] = i;
    }
    fn ready_pop(&mut self) -> usize {
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
    fn ready_remove(&mut self, task: usize) {
        let priority = self.tasks[task].priority as usize;
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
    fn take_yield_target(&mut self, current: usize) -> Option<usize> {
        let next = self.ready_pop();
        if next == NIL {
            return None;
        }
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some(next)
    }
    fn retire_stack(&mut self, stack: usize) {
        if stack != 0 {
            debug_assert!(self.retired_count < MAX_TASKS);
            self.retired_stacks[self.retired_count] = stack;
            self.retired_count += 1;
        }
    }
    fn wake_sleepers(&mut self, now: u64) {
        for i in 0..MAX_TASKS {
            if self.tasks[i].state == State::Sleeping && now >= self.tasks[i].wake_at {
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
            } else if self.tasks[i].state == State::Blocked
                && self.tasks[i].waiting_sem != 0
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
            }
        }
    }
    fn alloc_slot(&mut self) -> Option<usize> {
        (0..MAX_TASKS).find(|&i| self.tasks[i].state == State::Free)
    }
}

static SCHED: Mutex<RefCell<Sched>> = Mutex::new(RefCell::new(Sched::new()));

fn reclaim_retired_stacks() {
    let (stacks, count) = critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        let stacks = scheduler.retired_stacks;
        let count = scheduler.retired_count;
        scheduler.retired_stacks = [0; MAX_TASKS];
        scheduler.retired_count = 0;
        (stacks, count)
    });
    for stack in &stacks[..count] {
        deallocate(*stack as *mut u8);
    }
}

fn now_ms() -> u64 {
    (start_state().resources.monotonic_ms)()
}

/// First-run trampoline: a freshly switched-to task lands here (its `ctx.ra`),
/// runs its entry, then exits. Reads its own entry/arg from the current TCB.
extern "C" fn trampoline() -> ! {
    let (_slot, entry, arg, _stack) = critical_section::with(|cs| {
        let s = SCHED.borrow_ref(cs);
        let t = &s.tasks[s.current];
        (s.current, t.entry, t.arg, t.stack)
    });
    if let Some(f) = entry {
        f(arg as *mut c_void);
    }
    task_exit();
}

/// Initialize the scheduler, adopting the current execution as the main task
/// (slot 0). Idempotent.
fn init() {
    critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        if s.started {
            return;
        }
        s.tasks[0].state = State::Running;
        s.current = 0;
        s.started = true;
    });
}

/// Spawn a task. Returns its slot index, or `None` if the table/stack is full.
fn spawn(entry: TaskFn, arg: *mut c_void, stack_size: usize, priority: u8) -> Option<usize> {
    init();
    reclaim_retired_stacks();
    let size = stack_size.max(start_state().config.minimum_stack_size.get());
    let stack = allocate(size);
    if stack.is_null() {
        return None;
    }
    // 16-byte aligned stack top.
    let top = (stack as usize + size) & !0xf;
    let slot = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let i = s.alloc_slot()?;
        let t = &mut s.tasks[i];
        t.ctx = Ctx::zero();
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!(
                "mv {tp}, tp",
                "csrr {mstatus}, mstatus",
                "frcsr {fcsr}",
                tp = out(reg) t.ctx.tp,
                mstatus = out(reg) t.ctx.mstatus,
                fcsr = out(reg) t.ctx.fcsr,
                options(nomem, nostack),
            );
        }
        // Cast through a fn pointer (not a direct fn-item->int cast).
        let tramp: extern "C" fn() -> ! = trampoline;
        t.ctx.ra = tramp as usize;
        t.ctx.sp = top;
        t.state = State::Ready;
        t.stack = stack as usize;
        t.entry = Some(entry);
        t.arg = arg as usize;
        t.wake_at = 0;
        t.priority = priority;
        s.ready_push(i);
        Some(i)
    });
    if slot.is_none() {
        deallocate(stack);
    }
    slot
}

/// Switch away from `prev` to the next ready task, busy-idling (waking sleepers)
/// until one is runnable. `prev`'s state must already be set by the caller
/// (Ready+queued for yield, Blocked for a wait, Free for exit).
fn switch_to(prev: usize, next: usize) {
    if next == prev {
        critical_section::with(|cs| {
            SCHED.borrow_ref_mut(cs).tasks[next].state = State::Running;
        });
        return;
    }
    let (op, np) = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.tasks[next].state = State::Running;
        s.current = next;
        (
            core::ptr::addr_of_mut!(s.tasks[prev].ctx),
            core::ptr::addr_of!(s.tasks[next].ctx),
        )
    });
    // SAFETY: contexts live in the static SCHED (stable address); single-hart,
    // and the lock is released so the resumed task can re-enter the scheduler.
    unsafe { context_switch(op, np) };
}

fn switch_away(prev: usize) {
    loop {
        let now = now_ms();
        let next = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            s.wake_sleepers(now);
            s.ready_pop()
        });
        if next == NIL {
            core::hint::spin_loop();
            continue;
        }
        switch_to(prev, next);
        return;
    }
}

/// Yield the CPU: requeue the current task and run the next ready one.
fn yield_now() {
    let now = now_ms();
    let target = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.wake_sleepers(now);
        let cur = s.current;
        // A cooperative yield promises progress to another ready task. Select
        // that task before requeueing the current one; otherwise a strict
        // priority queue would immediately select the yielding high-priority
        // task again and starve lower-priority initialization work.
        s.take_yield_target(cur).map(|next| (cur, next))
    });
    if let Some((prev, next)) = target {
        switch_to(prev, next);
    }
    reclaim_retired_stacks();
}

/// Sleep the current task for `ms` milliseconds (cooperative; wakes when a later
/// schedule sees the deadline pass).
fn sleep_ms(ms: u32) {
    if ms == 0 {
        yield_now();
        return;
    }
    let wake_at = now_ms().saturating_add(ms as u64);
    let prev = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current;
        s.tasks[cur].state = State::Sleeping;
        s.tasks[cur].wake_at = wake_at;
        cur
    });
    switch_away(prev);
    reclaim_retired_stacks();
}

/// Current task slot index (its "pid"/"tid").
fn current_id() -> usize {
    critical_section::with(|cs| SCHED.borrow_ref(cs).current)
}

fn task_exit() -> ! {
    // Retire the stack before switching away. A resumed task drains the retired
    // list only after it is running on a different stack.
    let prev = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current;
        let stack = s.tasks[cur].stack;
        s.tasks[cur] = Tcb::empty();
        s.retire_stack(stack);
        cur
    });
    switch_away(prev);
    unreachable!()
}

// ── Counting semaphore (blocks via the scheduler) ───────────────────────────

/// A counting semaphore. Tasks block in [`Semaphore::down`] when the count is 0
/// and are woken by [`Semaphore::up`]. Backs `osal_sem_*` / `osal_wait_*` /
/// `osal_mutex_*`.
///
/// `&self` methods + interior mutability so it can be a `static` or heap object
/// shared across tasks; all state is touched only inside the scheduler critical
/// section (single-hart exclusive). Waiters are queued on the per-task `next`
/// link (a task is on at most one queue — ready OR one wait queue — at a time).
struct Semaphore {
    inner: UnsafeCell<SemState>,
}
struct SemState {
    count: i32,
    wait_head: usize,
    wait_tail: usize,
}

fn enqueue_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
    sched.tasks[task].next = NIL;
    if state.wait_tail == NIL {
        state.wait_head = task;
    } else {
        sched.tasks[state.wait_tail].next = task;
    }
    state.wait_tail = task;
}

fn remove_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
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
    const fn new(count: i32) -> Self {
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
    fn down(&self) {
        let block = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section (single hart).
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                false
            } else {
                let cur = s.current;
                s.tasks[cur].state = State::Blocked;
                s.tasks[cur].wake_at = 0;
                s.tasks[cur].waiting_sem = self as *const Self as usize;
                s.tasks[cur].sem_granted = false;
                enqueue_waiter(s, st, cur);
                true
            }
        });
        if block {
            // Parked on this sem's wait queue; `up` will move us back to Ready
            // (== the grant). When we resume here, we already hold the count.
            switch_away(current_id());
            critical_section::with(|cs| {
                let s = &mut *SCHED.borrow_ref_mut(cs);
                s.tasks[s.current].sem_granted = false;
            });
        }
    }

    /// Acquire with a timeout (ms). Returns `true` if a count was obtained,
    /// `false` if the deadline passed first. `u32::MAX` (wait-forever) blocks
    /// like [`down`](Semaphore::down).
    ///
    /// The waiter is linked into the semaphore queue so [`up`](Self::up) can
    /// hand the grant directly to it. The scheduler removes it from that queue
    /// if the mask-ROM systick deadline wins first.
    fn down_timeout(&self, timeout_ms: u32) -> bool {
        if timeout_ms == u32::MAX {
            self.down();
            return true;
        }
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                return None;
            }
            if timeout_ms == 0 {
                return Some(NIL);
            }
            let cur = s.current;
            s.tasks[cur].state = State::Blocked;
            s.tasks[cur].wake_at = deadline;
            s.tasks[cur].waiting_sem = self as *const Self as usize;
            s.tasks[cur].sem_granted = false;
            enqueue_waiter(s, st, cur);
            Some(cur)
        });
        match current {
            None => true,
            Some(NIL) => false,
            Some(current) => {
                switch_away(current);
                critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    granted
                })
            }
        }
    }

    /// Release (V). Wakes one waiter if any, else increments the count.
    fn up(&self) {
        critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            let w = st.wait_head;
            if w != NIL {
                st.wait_head = s.tasks[w].next;
                if st.wait_head == NIL {
                    st.wait_tail = NIL;
                }
                s.tasks[w].next = NIL;
                s.tasks[w].wake_at = 0;
                s.tasks[w].waiting_sem = 0;
                s.tasks[w].sem_granted = true;
                s.tasks[w].state = State::Ready;
                s.ready_push(w);
            } else {
                st.count += 1;
            }
        });
    }
}

struct HisiRuntime;

static RUNTIME: HisiRuntime = HisiRuntime;

/// Starts the scheduler and installs it as the firmware's sole radio runtime.
pub fn start(config: Config, resources: Resources) -> Result<(), StartError> {
    let already_started = critical_section::with(|cs| {
        let state = START_STATE.borrow(cs);
        if state.get().is_some() {
            true
        } else {
            state.set(Some(StartState { config, resources }));
            false
        }
    });
    if already_started {
        return Err(StartError::AlreadyStarted);
    }
    if let Err(error) = hisi_rf_rtos_driver::install(&RUNTIME) {
        critical_section::with(|cs| START_STATE.borrow(cs).set(None));
        return Err(StartError::Driver(error));
    }
    init();
    Ok(())
}

fn semaphore_from_handle(handle: SemaphoreHandle) -> &'static Semaphore {
    let pointer = handle.into_raw().get() as *const Semaphore;
    // SAFETY: this backend creates handles only from heap-allocated Semaphore
    // objects and the driver contract requires users to stop all access before
    // destroy.
    unsafe { &*pointer }
}

impl Runtime for HisiRuntime {
    fn spawn(
        &self,
        entry: hisi_rf_rtos_driver::TaskEntry,
        arg: *mut c_void,
        config: TaskConfig,
    ) -> Result<TaskId, DriverError> {
        if config.priority as usize >= PRIORITY_LEVELS {
            return Err(DriverError::Runtime);
        }
        let slot = spawn(entry, arg, config.stack_size.get(), config.priority)
            .ok_or(DriverError::ResourceExhausted)?;
        let raw = u32::try_from(slot).map_err(|_| DriverError::Runtime)?;
        Ok(TaskId::from_raw(raw))
    }

    fn yield_now(&self) -> Result<(), DriverError> {
        yield_now();
        Ok(())
    }

    fn sleep_ms(&self, milliseconds: NonZeroU32) -> Result<(), DriverError> {
        sleep_ms(milliseconds.get());
        Ok(())
    }

    fn current_task(&self) -> Result<TaskId, DriverError> {
        let raw = u32::try_from(current_id()).map_err(|_| DriverError::Runtime)?;
        Ok(TaskId::from_raw(raw))
    }

    fn set_task_priority(&self, task: TaskId, priority: u8) -> Result<(), DriverError> {
        if priority as usize >= PRIORITY_LEVELS {
            return Err(DriverError::Runtime);
        }
        let slot = usize::try_from(task.into_raw()).map_err(|_| DriverError::InvalidHandle)?;
        critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let Some(tcb) = scheduler.tasks.get(slot) else {
                return Err(DriverError::InvalidHandle);
            };
            if tcb.state == State::Free {
                return Err(DriverError::InvalidHandle);
            }
            if tcb.state == State::Ready {
                scheduler.ready_remove(slot);
                scheduler.tasks[slot].priority = priority;
                scheduler.ready_push(slot);
            } else {
                scheduler.tasks[slot].priority = priority;
            }
            Ok(())
        })
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
            if semaphore_from_handle(semaphore).down_timeout(timeout_ms) {
                WaitOutcome::Acquired
            } else {
                WaitOutcome::TimedOut
            },
        )
    }

    fn semaphore_up(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        semaphore_from_handle(semaphore).up();
        Ok(())
    }

    unsafe fn semaphore_destroy(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        deallocate(semaphore.into_raw().get() as *mut u8);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_task(scheduler: &mut Sched, slot: usize, priority: u8) {
        scheduler.tasks[slot].state = State::Ready;
        scheduler.tasks[slot].priority = priority;
        scheduler.ready_push(slot);
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
        scheduler.current = 1;
        scheduler.tasks[1].state = State::Running;
        scheduler.tasks[1].priority = 2;
        ready_task(&mut scheduler, 2, 8);

        assert_eq!(scheduler.take_yield_target(1), Some(2));
        assert_eq!(scheduler.ready_pop(), 1);
    }

    #[test]
    fn exited_stacks_are_retired_for_later_reclamation() {
        let mut scheduler = Sched::new();
        scheduler.retire_stack(0x1000);
        scheduler.retire_stack(0x2000);

        assert_eq!(scheduler.retired_count, 2);
        assert_eq!(&scheduler.retired_stacks[..2], &[0x1000, 0x2000]);
    }
}
