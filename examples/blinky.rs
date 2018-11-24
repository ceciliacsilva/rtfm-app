#![feature(custom_attribute)]
// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use narc_hal::stm32l052::Interrupt;
use narc_hal::gpio::GpioExt;
use narc_hal::rcc::RccExt;
use narc_hal::gpio::{Output, PushPull, gpioa::PA5};

use embedded_hal::digital::OutputPin;

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SHARED: u32 = 0;
    static mut LED: PA5<Output<PushPull>> = ();
    static mut EXTI: narc_hal::stm32l052::EXTI = ();

    #[init]
    fn init() {
        let device: narc_hal::stm32l052::Peripherals = device;

        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        // rtfm::pend(Interrupt::EXTI4_15);
        LED = led;
        EXTI = device.EXTI;
    }

    #[idle(resources = [SHARED, LED])]
    fn idle () -> ! {
        // wfi();

        loop {
            resources.LED.set_high();
        }
    }

    #[interrupt(resources = [SHARED, EXTI])]
    fn EXTI4_15 () {
        bkpt();

        *resources.SHARED += 1;

        resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }

    // extern "C" {
    //     fn USART1 ();
    // }
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}