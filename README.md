# axhal

Hardware abstraction layer, provides unified APIs for platform-specific operations.

It does the bootstrapping and initialization process for the specified platform, and provides useful operations on the hardware.

Currently supported platforms (specify by cargo features):

x86-pc: Standard PC with x86_64 ISA.
riscv64-qemu-virt: QEMU virt machine with RISC-V ISA.
aarch64-qemu-virt: QEMU virt machine with AArch64 ISA.
aarch64-raspi: Raspberry Pi with AArch64 ISA.
dummy: If none of the above platform is selected, the dummy platform will be used. In this platform, most of the operations are no-op or unimplemented!(). This platform is mainly used for cargo test.
Cargo Features
smp: Enable SMP (symmetric multiprocessing) support.
fp_simd: Enable floating-point and SIMD support.
paging: Enable page table manipulation.
irq: Enable interrupt handling support.

## Examples

```rust
#![no_std]
#![no_main]

#[macro_use]
extern crate axlog2;

use core::panic::PanicInfo;
use axhal::mem::memory_regions;

#[no_mangle]
pub extern "Rust" fn runtime_main(cpu_id: usize, _dtb_pa: usize) {
    axhal::arch_init_early(cpu_id);

    axlog2::init("debug");
    info!("[rt_axhal]: ...");

    info!("Found physcial memory regions:");
    for r in memory_regions() {
        info!(
            "  [{:x?}, {:x?}) {} ({:?})",
            r.paddr,
            r.paddr + r.size,
            r.name,
            r.flags
        );
    }

    axhal::platform_init();

    info!("[rt_axhal]: ok!");
    axhal::misc::terminate();
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    arch_boot::panic(info)
}
```

## Re-exports

### `platform_init`

```rust
pub fn platform_init()
```

Initializes the platform devices for the primary CPU.

## Modules

### `arch`

Architecture-specific types and operations.

#### Structs

#### `ExtendedState`

```rust
pub struct ExtendedState {
    pub fxsave_area: FxsaveArea,
}
```

Extended state of a task, such as FP/SIMD states.

#### `FxsaveArea`

```rust
#[repr(C, align(16))]
pub struct FxsaveArea {
    pub fcw: u16,
    pub fsw: u16,
    pub ftw: u16,
    pub fop: u16,
    pub fip: u64,
    pub fdp: u64,
    pub mxcsr: u32,
    pub mxcsr_mask: u32,
    pub st: [u64; 16],
    pub xmm: [u64; 32],
    /* private fields */
}
```

A 512-byte memory region for the FXSAVE/FXRSTOR instruction to save and restore the x87 FPU, MMX, XMM, and MXCSR registers.

#### `GdtStruct`

```rust
#[repr(align(16))]
pub struct GdtStruct { /* private fields */ }
```

A wrapper of the Global Descriptor Table (GDT) with maximum 16 entries.

#### `TaskContext`

```rust
pub struct TaskContext {
    pub kstack_top: VirtAddr,
    pub rsp: u64,
    pub fs_base: usize,
}
```

Saved hardware states of a task.

#### `TaskStateSegment`

```rust
#[repr(C, packed(4))]
pub struct TaskStateSegment {
    pub privilege_stack_table: [VirtAddr; 3],
    pub interrupt_stack_table: [VirtAddr; 7],
    pub iomap_base: u16,
    /* private fields */
}
```

In 64-bit mode the TSS holds information that is not directly related to the task-switch mechanism, but is used for finding kernel level stack if interrupts arrive while in kernel mode.

#### `TrapFrame`

```rust
#[repr(C)]
pub struct TrapFrame {
    pub rax: u64,
    pub rcx: u64,
    pub rdx: u64,
    pub rbx: u64,
    pub rbp: u64,
    pub rsi: u64,
    pub rdi: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
    pub vector: u64,
    pub error_code: u64,
    pub rip: u64,
    pub cs: u64,
    pub rflags: u64,
    pub rsp: u64,
    pub ss: u64,
}
```

Saved registers when a trap (interrupt or exception) occurs.

### `cpu`

CPU-related operations.

#### Functions

##### `_this_cpu_id`

```rust
pub fn _this_cpu_id() -> usize
```

Safety: Makesure that it will be called under No-Preemption.
Returns the ID of the current CPU.

##### `_this_cpu_is_bsp`

```rust
pub fn _this_cpu_is_bsp() -> bool
```

Safety: Makesure that it will be called under No-Preemption.

Returns whether the current CPU is the primary CPU (aka the bootstrap processor or BSP)

##### `current_task_ptr`

```rust
pub fn current_task_ptr<T>() -> *const T
```

Gets the pointer to the current task with preemption-safety.

Preemption may be enabled when calling this function. This function will guarantee the correctness even the current task is preempted.

##### `init_primary`

```rust
pub fn init_primary(cpu_id: usize)
```

Initializes the primary CPU.

##### `init_secondary`

```rust
pub fn init_secondary(cpu_id: usize)
```

Initializes a secondary CPU.

##### `set_current_task_ptr`

```rust
pub unsafe fn set_current_task_ptr<T>(ptr: *const T)
```

Sets the pointer to the current task with preemption-safety.

Preemption may be enabled when calling this function. This function will guarantee the correctness even the current task is preempted.

**Safety**
The given ptr must be pointed to a valid task structure.

### `mem`

Physical memory management.

#### Re-exports

##### `memory_addr::PhysAddr`

```rust
pub struct PhysAddr(/* private fields */);
```

A physical memory address.
It’s a wrapper type around an usize.

##### memory_addr::VirtAddr

```rust
pub struct VirtAddr(/* private fields */);
```

A virtual memory address.
It’s a wrapper type around an usize.

##### `memory_addr::PAGE_SIZE_4K`

```rust
pub const PAGE_SIZE_4K: usize = 0x1000;
```

The size of a 4K page (4096 bytes).

#### Structs

##### `MemRegion`

```rust
pub struct MemRegion {
    pub paddr: PhysAddr,
    pub size: usize,
    pub flags: MemRegionFlags,
    pub name: &'static str,
}
```

A physical memory region.

##### `MemRegionFlags`

```rust
pub struct MemRegionFlags(/* private fields */);
```

The flags of a physical memory region.

#### Functions

##### `clear_bss`

```rust
pub fn clear_bss()
```

Fills the .bss section with zeros.

##### `memory_regions`

```rust
pub fn memory_regions() -> impl Iterator<Item = MemRegion>
```

Returns an iterator over all physical memory regions.

##### `phys_to_virt`

```rust
pub const fn phys_to_virt(paddr: PhysAddr) -> VirtAddr
```

Converts a physical address to a virtual address.
It assumes that there is a linear mapping with the offset PHYS_VIRT_OFFSET, that maps all the physical memory to the virtual space at the address plus the offset. So we have vaddr = paddr + PHYS_VIRT_OFFSET.

##### `virt_to_phys`

```rust
pub const fn virt_to_phys(vaddr: VirtAddr) -> PhysAddr
```

Converts a virtual address to a physical address.
It assumes that there is a linear mapping with the offset PHYS_VIRT_OFFSET, that maps all the physical memory to the virtual space at the address plus the offset. So we have paddr = vaddr - PHYS_VIRT_OFFSET.

### `misc`

Miscellaneous operation, e.g. terminate the system. Misc

#### Functions

##### `terminate`

```rust
pub fn terminate() -> !
```

Shutdown the whole system, including all CPUs.

### `platform`

Platform-specific operations.

#### Modules

##### `console`

###### Functions

**`getchar`**

```rust
pub fn getchar() -> Option<u8>
```

Reads a byte from the console, or returns None if no input is available.

**`putchar`**

```rust
pub fn putchar(c: u8)
```

Writes a byte to the console.

##### `irq`

###### Functions

**`dispatch_irq`**

```rust
pub fn dispatch_irq(irq_num: usize)
```

Dispatches the IRQ.
This function is called by the common interrupt handler. It looks up in the IRQ handler table and calls the corresponding handler. If necessary, it also acknowledges the interrupt controller after handling.

**`set_enable`**

```rust
pub fn set_enable(irq_num: usize, enabled: bool)
```

Enables or disables the given IRQ.

##### `mem`

##### `misc`

###### Functions

**`terminate`**

```rust
pub fn terminate() -> !
```

Shutdown the whole system, including all CPUs.

##### `time`

###### Functions

**`current_ticks`**

```rust
pub fn current_ticks() -> u64
```

Returns the current clock time in hardware ticks.

**nanos_to_ticks**

```rust
pub fn nanos_to_ticks(nanos: u64) -> u64
```

Converts nanoseconds to hardware ticks.

**`set_oneshot_timer`**

```rust
pub fn set_oneshot_timer(deadline_ns: u64)
```

Set a one-shot timer.
A timer interrupt will be triggered at the given deadline (in nanoseconds).

**`ticks_to_nanos`**

```rust
pub fn ticks_to_nanos(ticks: u64) -> u64
```

Converts hardware ticks to nanoseconds.

#### Functions

##### `platform_init`

```rust
pub fn platform_init()
```

Initializes the platform devices for the primary CPU.

##### `set_tss_stack_top`

```rust
pub fn set_tss_stack_top(_kernel_stack_top: VirtAddr)
```

To be called by the kernel to set the top of the TSS stack.

### `time`

Time-related operations.

#### Structs

##### `Duration`

```rust
pub struct Duration { /* private fields */ }
```

A Duration type to represent a span of time, typically used for system timeouts.

Each Duration is composed of a whole number of seconds and a fractional part represented in nanoseconds. If the underlying system does not support nanosecond-level precision, APIs binding a system timeout will typically round up the number of nanoseconds.

Durations implement many common traits, including Add, Sub, and other ops traits. It implements Default by returning a zero-length Duration.

#### Constants

**`MICROS_PER_SEC`**: Number of microseconds in a second.
**`MILLIS_PER_SEC`**: Number of milliseconds in a second.
**`NANOS_PER_MICROS`**: Number of nanoseconds in a microsecond.
**`NANOS_PER_MILLIS`**:Number of nanoseconds in a millisecond.
**`NANOS_PER_SEC`**: Number of nanoseconds in a second.
**`TIMER_IRQ_NUMirq`**: The timer IRQ number.

#### Functions

##### `busy_wait`

```rust
pub fn busy_wait(dur: Duration)
```

Busy waiting for the given duration.

##### `busy_wait_until`

```rust
pub fn busy_wait_until(deadline: TimeValue)
```

Busy waiting until reaching the given deadline.

##### `current_ticks`

```rust
pub fn current_ticks() -> u64
```

Returns the current clock time in hardware ticks.

##### `current_time`

```rust
pub fn current_time() -> TimeValue
```

Returns the current clock time in TimeValue.

##### `current_time_nanos`

```rust
pub fn current_time_nanos() -> u64
```

Returns the current clock time in nanoseconds.

##### `nanos_to_ticks`

```rust
pub fn nanos_to_ticks(nanos: u64) -> u64
```

Converts nanoseconds to hardware ticks.

##### `set_oneshot_timerirq`

```rust
pub fn set_oneshot_timer(deadline_ns: u64)
```

Set a one-shot timer.

##### `ticks_to_nanos`

```rust
pub fn ticks_to_nanos(ticks: u64) -> u64
```

Converts hardware ticks to nanoseconds.

### `trap`

Trap handling.
