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
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use embedded_hal::digital::OutputPin;

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut LED: PA5<Output<PushPull>> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        
        let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);

        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        // led.set_high();

        LED = led;
    }

    #[idle(resources=[LED])]
    fn idle() -> ! {
    
        resources.LED.lock(|led| led.set_low());

        wfi();
        loop {
            
        }
    }

    #[interrupt(resources=[LED], priority=2)]
    fn EXTI4_15 () {
        resources.LED.set_high();
        // Will not leave the interrupt without the next line.
        // resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }
};



#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}