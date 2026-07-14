//! RISC-V task context ABI and the explicit port-less cooperative fallback.
//!
//! Target-backed Cooperative, Budgeted, and Preemptive tasks do not call the
//! fallback below. They all switch through the target port's software interrupt,
//! the unified 272-byte trap frame, and `mret`.

/// Unified 272-byte task context shared with `hisi-riscv-rt` trap frames.
///
/// The field order matches the WS63 LiteOS `TaskContext` behavior oracle. The
/// type is crate-private because it is an architecture-port contract rather
/// than an application-facing API.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub(crate) struct TaskContext {
    pub(crate) mstatus: u32, // 0
    pub(crate) mepc: u32,    // 4
    pub(crate) tp: u32,      // 8
    pub(crate) sp: u32,      // 12
    s: [u32; 12],            // 16..64: s11..s0
    caller_gpr: [u32; 16],   // 64..128: t6..t3,a7..a0,t2..t0,ra
    f: [u32; 32],            // 128..256: fs11..fs0,ft11..ft0
    pub(crate) fcsr: u32,    // 256
    reserved: [u32; 3],      // 260..272
}

impl TaskContext {
    pub(crate) const fn zero() -> Self {
        Self {
            mstatus: 0,
            mepc: 0,
            tp: 0,
            sp: 0,
            s: [0; 12],
            caller_gpr: [0; 16],
            f: [0; 32],
            fcsr: 0,
            reserved: [0; 3],
        }
    }
}

const _: () = {
    assert!(core::mem::size_of::<TaskContext>() == 272);
    assert!(core::mem::offset_of!(TaskContext, mstatus) == 0);
    assert!(core::mem::offset_of!(TaskContext, mepc) == 4);
    assert!(core::mem::offset_of!(TaskContext, s) == 16);
    assert!(core::mem::offset_of!(TaskContext, caller_gpr) == 64);
    assert!(core::mem::offset_of!(TaskContext, f) == 128);
    assert!(core::mem::offset_of!(TaskContext, fcsr) == 256);
};

#[cfg(target_arch = "riscv32")]
const TASK_CONTEXT_WORDS: usize = 68;

#[cfg(target_arch = "riscv32")]
pub(crate) unsafe fn initialize_irq_frame(top: usize, entry: usize, tp: usize, fcsr: u32) -> usize {
    let frame = (top - TASK_CONTEXT_WORDS * core::mem::size_of::<u32>()) as *mut u32;
    // SAFETY: the caller owns the allocated task stack through its TCB, and
    // `top` leaves at least the configured minimum stack size below it.
    unsafe {
        frame.write_bytes(0, TASK_CONTEXT_WORDS);
        frame.add(0).write(0x7880);
        frame.add(1).write(entry as u32);
        frame.add(2).write(tp as u32);
        frame.add(3).write(top as u32);
        frame.add(64).write(fcsr);
    }
    frame as usize
}

/// Port-less cooperative context switch fallback.
///
/// This function is used only by [`crate::start_cooperative`]. Target-backed
/// Cooperative, Budgeted, and Preemptive operation uses SWI/trap-frame/`mret`.
/// Caller-saved registers are spilled around this normal call, so this path
/// stores only ABI callee-saved registers before restoring the unified frame.
#[cfg(target_arch = "riscv32")]
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn cooperative_context_switch_fallback(
    old: *mut TaskContext,
    new: *const TaskContext,
) {
    core::arch::naked_asm!(
        ".option arch, +f",
        "li t1, 8",
        "csrrc t0, mstatus, t1",
        "andi t2, t0, 8",
        "slli t2, t2, 4",
        "li t1, -137",
        "and t0, t0, t1",
        "or t0, t0, t2",
        "li t1, 0x1800",
        "or t0, t0, t1",
        "sw t0, 0(a0)",
        "sw ra, 4(a0)",
        "sw tp, 8(a0)",
        "sw sp, 12(a0)",
        "sw s11, 16(a0)",
        "sw s10, 20(a0)",
        "sw s9, 24(a0)",
        "sw s8, 28(a0)",
        "sw s7, 32(a0)",
        "sw s6, 36(a0)",
        "sw s5, 40(a0)",
        "sw s4, 44(a0)",
        "sw s3, 48(a0)",
        "sw s2, 52(a0)",
        "sw s1, 56(a0)",
        "sw s0, 60(a0)",
        "fsw fs11, 128(a0)",
        "fsw fs10, 132(a0)",
        "fsw fs9, 136(a0)",
        "fsw fs8, 140(a0)",
        "fsw fs7, 144(a0)",
        "fsw fs6, 148(a0)",
        "fsw fs5, 152(a0)",
        "fsw fs4, 156(a0)",
        "fsw fs3, 160(a0)",
        "fsw fs2, 164(a0)",
        "fsw fs1, 168(a0)",
        "fsw fs0, 172(a0)",
        "frcsr t0",
        "sw t0, 256(a0)",
        "mv t0, a1",
        "lw t1, 0(t0)",
        "csrw mstatus, t1",
        "lw t1, 4(t0)",
        "csrw mepc, t1",
        "lw t1, 256(t0)",
        "fscsr t1",
        "flw fs11, 128(t0)",
        "flw fs10, 132(t0)",
        "flw fs9, 136(t0)",
        "flw fs8, 140(t0)",
        "flw fs7, 144(t0)",
        "flw fs6, 148(t0)",
        "flw fs5, 152(t0)",
        "flw fs4, 156(t0)",
        "flw fs3, 160(t0)",
        "flw fs2, 164(t0)",
        "flw fs1, 168(t0)",
        "flw fs0, 172(t0)",
        "flw ft11, 176(t0)",
        "flw ft10, 180(t0)",
        "flw ft9, 184(t0)",
        "flw ft8, 188(t0)",
        "flw fa7, 192(t0)",
        "flw fa6, 196(t0)",
        "flw fa5, 200(t0)",
        "flw fa4, 204(t0)",
        "flw fa3, 208(t0)",
        "flw fa2, 212(t0)",
        "flw fa1, 216(t0)",
        "flw fa0, 220(t0)",
        "flw ft7, 224(t0)",
        "flw ft6, 228(t0)",
        "flw ft5, 232(t0)",
        "flw ft4, 236(t0)",
        "flw ft3, 240(t0)",
        "flw ft2, 244(t0)",
        "flw ft1, 248(t0)",
        "flw ft0, 252(t0)",
        "lw tp, 8(t0)",
        "lw s11, 16(t0)",
        "lw s10, 20(t0)",
        "lw s9, 24(t0)",
        "lw s8, 28(t0)",
        "lw s7, 32(t0)",
        "lw s6, 36(t0)",
        "lw s5, 40(t0)",
        "lw s4, 44(t0)",
        "lw s3, 48(t0)",
        "lw s2, 52(t0)",
        "lw s1, 56(t0)",
        "lw s0, 60(t0)",
        "lw t6, 64(t0)",
        "lw t5, 68(t0)",
        "lw t4, 72(t0)",
        "lw t3, 76(t0)",
        "lw a7, 80(t0)",
        "lw a6, 84(t0)",
        "lw a5, 88(t0)",
        "lw a4, 92(t0)",
        "lw a3, 96(t0)",
        "lw a2, 100(t0)",
        "lw a1, 104(t0)",
        "lw a0, 108(t0)",
        "lw t2, 112(t0)",
        "lw t1, 116(t0)",
        "lw ra, 124(t0)",
        "lw sp, 12(t0)",
        "lw t0, 120(t0)",
        "mret",
    )
}

#[cfg(not(target_arch = "riscv32"))]
pub(crate) unsafe extern "C" fn cooperative_context_switch_fallback(
    _old: *mut TaskContext,
    _new: *const TaskContext,
) {
    unreachable!("WS63 context switching is only available on riscv32");
}
