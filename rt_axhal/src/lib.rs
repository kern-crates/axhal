#![no_std]

#[macro_use]
extern crate axlog2;
extern crate alloc;

use core::panic::PanicInfo;
use axhal::mem::memory_regions;
use axtype::{align_up_4k, align_down_4k, phys_to_virt, virt_to_phys};

#[no_mangle]
pub extern "Rust" fn runtime_main(cpu_id: usize, _dtb_pa: usize) {
    axhal::cpu::init_primary(cpu_id);

    axlog2::init();
    axlog2::set_max_level("debug");
    info!("[rt_axhal]: ...");

    init_axalloc();

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

    info!("Initialize kernel page table...");
    remap_kernel_memory();

    info!("[rt_axhal]: ok!");
    axhal::misc::terminate();
}

fn remap_kernel_memory() {
    use axhal::paging::PageTable;
    use axhal::paging::setup_page_table_root;
    use axhal::mem::phys_to_virt;

    let mut kernel_page_table = PageTable::try_new().unwrap();
    for r in memory_regions() {
        kernel_page_table.map_region(
            phys_to_virt(r.paddr),
            r.paddr,
            r.size,
            r.flags.into(),
            true,
        ).unwrap();
    }
    setup_page_table_root(kernel_page_table);
}

fn init_axalloc() {
    let start = align_up_4k(virt_to_phys(_ekernel as usize));
    let end = align_down_4k(axconfig::PHYS_MEMORY_END);
    axalloc::global_init(phys_to_virt(start), end - start);
}

pub fn panic(info: &PanicInfo) -> ! {
    arch_boot::panic(info)
}

extern "C" {
    fn _ekernel();
}
