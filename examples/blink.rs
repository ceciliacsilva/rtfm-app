#![feature(panic_implementation)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_rtfm as rtfm;
extern crate hal;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Threshold};

use hal::stm32l052;
use hal::stm32l052::GPIOA;

app! {
    // device: stm32l0x1,
    device: stm32l052,
    resources: {
        static ON: bool = false;
    },
    tasks: {
        SysTick: {
            path: sys_tick,
            resources: [ON],
        },
    }
}

fn init(mut p: init::Peripherals, r: init::Resources) {
    // `init` can modify all the `resources` declared in `app!`
    r.ON;

    p.device.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());

    p.device.GPIOA.moder.modify(|_, w| unsafe{ w.mode5().bits(1) });

    // p.device.GPIOA.odr.modify(|_, w| w.od5().set_bit());
    
    p.core.SYST.set_clock_source(SystClkSource::Core);
    p.core.SYST.set_reload(2_000_000); // 1s
    p.core.SYST.clear_current();
    p.core.SYST.enable_counter();
    p.core.SYST.enable_interrupt();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

#[allow(unsafe_code)]
fn sys_tick(_t: &mut Threshold, mut r: SysTick::Resources) {
    // toggle state
    *r.ON = !*r.ON;

    if *r.ON {
        unsafe {
            (*GPIOA::ptr()).bsrr.write(|w| w.bs5().set_bit());
        }
    } else {
        unsafe {
            (*GPIOA::ptr()).bsrr.write(|w| w.br5().set_bit());
        }
    }
}



#[allow(deprecated)]
#[panic_implementation]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}