#![feature(custom_attribute)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
use cortex_m_rt::{entry, exception};

use narc_hal::stm32l052::interrupt;
use narc_hal::gpio::GpioExt;
use narc_hal::rcc::RccExt;
use narc_hal::flash::FlashExt;
use narc_hal::serial::Serial;
use narc_hal::serial::{Rx, Event, Tx};
use narc_hal::time::U32Ext;
use embedded_hal::prelude::*;
use narc_hal::gpio::{Output, PushPull, gpioa::PA5};
use narc_hal::stm32l052::USART2 as USART2_p;

use embedded_hal::digital::OutputPin;

use cortex_m::peripheral::syst::SystClkSource;

#[entry]
fn main () -> ! {
    let device = narc_hal::stm32l052::Peripherals::take().unwrap();
    let mut core = cortex_m::Peripherals::take().unwrap();

    core.NVIC.enable(narc_hal::stm32l052::Interrupt::USART2);

    unsafe {
        core.NVIC.set_priority(narc_hal::stm32l052::Interrupt::USART2, 48);
    }
    
    //// Systick exception works
    // core.SYST.set_clock_source(SystClkSource::Core);
    // core.SYST.set_reload(2_000_000); // 1s
    // core.SYST.clear_current();
    // core.SYST.enable_counter();
    // core.SYST.enable_interrupt();

    let mut rcc = device.RCC.constrain();
    let mut gpioa = device.GPIOA.split(&mut rcc.iop);
    let mut flash = device.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

    let tx = gpioa.pa2.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);
    let rx = gpioa.pa3.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);

    let serial = Serial::usart2(
        device.USART2,
        (tx, rx),
        9_600.bps(),
        clocks,
        &mut rcc.apb1,
        None
    );

    serial.listen(Event::Rxne);
    
    let (mut tx, mut _rx, mut _ri) = serial.split();

    tx.write(b'o');

    led.set_high();

    loop {}
}

// Old model. 
// interrupt!(USART2, usart2);

// fn usart2 () {
//     bkpt();
// }

fn USART2 () {
    bkpt();
}

// #[exception]
// fn SysTick() {
//     bkpt();
// }

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}