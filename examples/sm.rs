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
#[macro_use]
extern crate nb;
extern crate heapless;

use rtfm::app;
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

use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
    Vec,
};

extern crate sm;
use sm::sm;

sm! {
    Lock {
        InitialStates { Locked, Unlocked }

        TurnKey {
            Locked => Unlocked
            Unlocked => Locked
        }

        Break {
            Locked, Unlocked => Broken
        }
    }
}

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SHARED: bool = false;
    static mut LED: PA5<Output<PushPull>> = ();
    static mut EXTI: narc_hal::stm32l052::EXTI = ();
    static mut RX: Rx<USART2_p> = ();
    static mut TX: Tx<USART2_p> = ();
    static mut Q: Option<Queue<u8, U4>> = None;
    static mut P: Producer<'static, u8, U4> = ();
    static mut C: Consumer<'static, u8, U4> = ();

    #[init(resources=[Q])]
    fn init() {
        let device: narc_hal::stm32l052::Peripherals = device;

        // NOTE: we use `Option` here to work around the lack of
        // a stable `const` constructor
        
        *resources.Q = Some(Queue::new());
        let (p, c) = resources.Q.as_mut().unwrap().split();
        
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
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

        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        let (mut tx, mut rx, mut _ri) = serial.split();

        // let data = block!(rx.read()).unwrap();
        tx.write(b'o');

        // rtfm::pend(Interrupt::EXTI4_15);
        LED = led;
        EXTI = device.EXTI;
        RX = rx;
        TX = tx;
        P = p;
        C = c;
    }

    #[idle(resources = [SHARED, LED, C, TX])]
    fn idle () -> ! {
        wfi();

        let sm: Sm<Init> = Sm::new();

        let mut sm: Sm<Auto> = Sm::<Auto>::from(sm);

        use crate::Lock::*;
        let mut lock = Machine::new(Locked).as_enum();

        loop {
            resources.LED.set_high();

            let mut frame: Vec<u8, U10> = Vec::new();
            if resources.SHARED.lock(|end| *end) {
                while let Some(c) = resources.C.dequeue() {
                    // resources.TX.write(c);
                    frame.push(c);
                }
            }

            // sm = match sm.as_enum(){
                // Locked(m) => m.transition(TurnKey),
                // _ => 
            // }

            if let Variant::InitialLocked(m) = lock {
                lock = m.transition(TurnKey).as_enum()
            };

            match &frame[..] {
                [b'a', b'u', b't', b'o'] => sm = Sm::<Auto>::from(sm),
                [b'm', b'a', b'n', b'u', b'a', b'l'] => sm = Sm::<Auto>::from(sm),
                _ => (),
            };

            resources.SHARED.lock(|end| *end != *end);

            resources.LED.set_low();
        }
    }

    #[interrupt(resources = [SHARED, EXTI])]
    fn EXTI4_15 () {
        // bkpt();
        *resources.SHARED = true;
        resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }

    #[interrupt(resources = [P, RX])]
    fn USART2 () {
        let data = resources.RX.read().unwrap();
        resources.P.enqueue(data).unwrap();
    }
};



struct Sm<S> {
    state: S,
}

//Add behaviours
struct Init;
struct Auto;
struct Manual;
struct Break;

impl Sm<Init> {
    fn new() -> Self {
        Sm {
            state: Init,
        }
    }
}

impl From<Sm<Init>> for Sm<Auto> {
    fn from(_val: Sm<Init>) -> Sm<Auto> {
        Sm {
            state: Auto,
        }
    }
}

impl From<Sm<Auto>> for Sm<Manual> {
    fn from(_val: Sm<Auto>) -> Sm<Manual> {
        Sm {
            state: Manual,
        }
    }
}

impl From<Sm<Manual>> for Sm<Auto> {
    fn from(_val: Sm<Manual>) -> Sm<Auto> {
        Sm {
            state: Auto,
        }
    }
}

impl From<Sm<Manual>> for Sm<Break> {
    fn from(_val: Sm<Manual>) -> Sm<Break> {
        Sm {
            state: Break,
        }
    }
}

impl From<Sm<Break>> for Sm<Auto> {
    fn from(_val: Sm<Break>) -> Sm<Auto> {
        Sm {
            state: Auto,
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}