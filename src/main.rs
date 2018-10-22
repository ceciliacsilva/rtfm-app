#![feature(panic_implementation)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;

extern crate stm32l0;
// extern crate stm32l052;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
use cortex_m::peripheral::syst::SystClkSource;

use cortex_m_rt::{entry, exception};

use stm32l0::stm32l0x1;
use stm32l0::stm32l0x1::GPIOA;

// use stm32l052::GPIOA;

#[entry]
fn main() -> ! {
    let hw = stm32l0x1::Peripherals::take().unwrap();
    // let hw = stm32l052::Peripherals::take().unwrap();

    let mut cp = cortex_m::Peripherals::take().unwrap();

    let gpioa = &hw.GPIOA;
    let rcc = &hw.RCC;

    rcc.iopenr.modify(|_, w| w.iopaen().set_bit());

    gpioa.moder.modify(|_, w| unsafe{ w.mode5().bits(1) });

    // gpioa.odr.modify(|_, w| w.od5().set_bit());

    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(2_000_000); // 1s
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    loop {
                
    }
}

#[exception]
fn SysTick() {
    static mut ON: bool = true;

    *ON = !*ON;

    if *ON {
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