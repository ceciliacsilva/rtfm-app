#![feature(custom_attribute)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;

use rtfm::{app, Instant};
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

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

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut EXTI: narc_hal::stm32l052::EXTI = ();
    static mut LED: PA5<Output<PushPull>> = ();

    #[init]
    fn init () {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        
        let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);
        let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);

        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        EXTI = device.EXTI;
        LED = led;
    }

    #[idle]
    fn idle () -> ! {
        wfi();
        // bkpt();

        loop {
        }
    }

    #[interrupt(resources = [EXTI, LED], schedule = [debouncing_handler])]
    // #[interrupt(resources = [EXTI, LED], spawn = [debouncing_handler])]
    // #[interrupt(resources = [EXTI, LED])]
    fn EXTI4_15 () {
        schedule.debouncing_handler(Instant::now() + 1_000_000.cycles()).unwrap();
        // spawn.debouncing_handler().unwrap();
        // bkpt();
        resources.LED.set_high();
        resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }

    #[task(resources = [LED])]
    fn debouncing_handler () {
        static mut counter: u32 = 0;

        resources.LED.set_low();
        *counter += 1;
        bkpt();
    }

    extern "C" {
        // fn SysTick();
        fn TIM2 ();
    }
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
