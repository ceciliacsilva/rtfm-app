#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate timer_wheels;

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

    #[init(spawn = [foo])]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        spawn.foo().unwrap();

        LED = led;
    }

    #[idle]
    fn idle() -> ! {
        wfi();
        loop {
            unimplemented!();
        }
    }

    #[task(resources = [LED])]
    fn foo() {
        common(resources.LED);
    }

    #[task(spawn = [baz])]
    fn bar() {
        spawn.baz();
    }

    #[task(resources = [LED])]
    fn baz() {
        common(resources.LED);
    }

    extern "C" {
        fn SPI1();
    }

};

fn common(mut shared_led: impl rtfm::Mutex<T = PA5<Output<PushPull>>>){
    shared_led.lock(|led| {
        led.set_high();
    });
}


#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
