#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate timer_wheels;
extern crate heapless;

use rtfm::app;
use rtfm::Mutex;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use embedded_hal::digital::OutputPin;
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut Q: Option<Queue<Event, U4>> = None;
    static mut P: Producer<'static, Event, U4> = ();
    static mut C: Consumer<'static, Event, U4> = ();
    // static mut LED: PA5<Output<PushPull>> = ();
    static mut E_LED_ON: Event = ();

    #[init(spawn = [foo], resources = [Q])]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);

        spawn.foo();

        *resources.Q = Some(Queue::new());
        let (p, c) = resources.Q.as_mut().unwrap().split();

        // construir o evento nao o setledon
        E_LED_ON =
            Event {
                e: &mut
                    SetLedOn{
                        led: led,
                    }
            };

        P = p;
        C = c;
        // LED = led;
    }

    #[idle]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    #[task(resources = [P, E_LED_ON])]
    fn foo() {
        // *resources.E_LED_ON = Some(
        // );
        // spawn.bar().unwrap();
        // resources.EVENT.run();

        // resources.P.enqueue(
        //     Event {
        //         e: resources.E_LED_ON,
        //     });
    }

    extern "C" {
        fn SPI1();
    }

};


pub trait ValidEvent {
    fn run(&mut self);
}

pub struct Event
{
    e: &'static mut ValidEvent,
}

unsafe impl Send for Event{}

pub struct SetLedOn
// where
    // T: Mutex<T = PA5<Output<PushPull>>>,
{
    // led: &'a mut PA5<Output<PushPull>>,
    led: PA5<Output<PushPull>>,
    // led: bool,
}

impl ValidEvent for SetLedOn
// where
    // T: Mutex<T = PA5<Output<PushPull>>>
{
    fn run(&mut self) {
        use rtfm::Mutex;

        // self.led.lock(
            // |led| {
        // self.led.set_high(); 
        // });
    }
}

pub struct EventDisableLed
// where
// T: Mutex<T = PA5<Output<PushPull>>>,
{
    led: PA5<Output<PushPull>>,
}

impl ValidEvent for EventDisableLed
// where
// T: Mutex<T = PA5<Output<PushPull>>>
{
    fn run(&mut self) {
        use rtfm::Mutex;

        // self.led.lock(
        // |led| {
        self.led.set_low();
        // });
    }
}

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
