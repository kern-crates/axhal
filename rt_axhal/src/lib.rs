#![no_std]

use core::panic::PanicInfo;

#[no_mangle]
pub extern "Rust" fn runtime_main(cpu_id: usize, _dtb_pa: usize) {
    axhal::cpu::init_primary(cpu_id);

    axlog2::init();
    axlog2::set_max_level("debug");
    info!("[rt_axhal]: ...");

    info!("Found physcial memory regions:");
    for r in axhal::mem::memory_regions() {
        info!(
            "  [{:x?}, {:x?}) {} ({:?})",
            r.paddr,
            r.paddr + r.size,
            r.name,
            r.flags
        );
    }

    info!("[rt_axhal]: ok!");
    axhal::misc::terminate();
}

pub fn panic(info: &PanicInfo) -> ! {
    arch_boot::panic(info)
}
