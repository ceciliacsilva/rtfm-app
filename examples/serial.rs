#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_rtfm as rtfm;
extern crate hal;
extern crate embedded_hal;
extern crate cortex_m_semihosting as sh;
extern crate heapless;
#[macro_use(block)]
extern crate nb;

use heapless::Vec; 
use heapless::{spsc, spsc::Consumer, spsc::Producer};
use heapless::consts::U20; 

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use rtfm::{app, Threshold, Resource};

use embedded_hal::prelude::*;

use hal::rcc::RccExt;
use hal::gpio::GpioExt;
use hal::flash::FlashExt;
use hal::serial::Serial;
use hal::serial::{Rx, Event, Tx};
use hal::time::U32Ext;
use hal::stm32l052;
use hal::stm32l052::USART2 as USART2_p;

use hal::gpio::{Output, PushPull, gpioa::PA5};

// use core::fmt::Write;
// use sh::hio;

enum Command {
    Enable,
    Disable,
    Nothing,
}

app! {
    device: stm32l052,
    resources: {
        static END: bool = false;
        static RB: spsc::Queue<u8, U20> = spsc::Queue::new();
        static RX: Rx<USART2_p>;
        static TX: Tx<USART2_p>;
        static LED: PA5<Output<PushPull>>;
    },
    idle: {
        resources: [LED, END, RB, TX],
    },
    tasks: {
        USART2: {
            priority: 2,
            path: receive_callback,
            resources: [RX, END, RB],
        },
    }
}

fn init(mut p: init::Peripherals, r: init::Resources) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut flash = p.device.FLASH.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.iop);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

    let tx = gpioa.pa2.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);
    let rx = gpioa.pa3.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);

    let serial = Serial::usart2(
        p.device.USART2,
        (tx, rx),
        9_600.bps(),
        clocks,
        &mut rcc.apb1,
        None,
    );

    serial.listen(Event::Rxne);

    let (mut tx, mut rx, _) = serial.split();

    init::LateResources {
        RX: rx,
        LED: led,
        TX: tx,
    }
}

fn idle(mut t: &mut Threshold, mut r: idle::Resources) -> ! {
    // let mut hstdout = hio::hstdout().unwrap();

    loop {
        rtfm::wfi();

        if r.END.claim(&mut t, |end, _| *end) {
            let mut vec: Vec<u8, U20> = Vec::new();

            loop{
                let c = r.RB.claim_mut(t, |rb, _| rb.dequeue());

                match c {
                    Some(data) => 
                        {
                            if data != 32 {
                                vec.push(data);
                            }
                        },
                    _ => break,
                };
            }

            // for (i, e) in vec.iter().enumerate(){
            //     block!(r.TX.write(*e));
            // }

            let op = vec.get(4);

            let command =
                match op {
                    Some(78) | Some (110) => Command::Enable,
                    Some(70) | Some (102) => Command::Disable,
                    _ => Command::Nothing,
                };
            
            match command {
                Command::Enable => r.LED.set_high(),
                Command::Disable => r.LED.set_low(),
                _ => (),
            }

            r.END.claim_mut(&mut t, |end, _| *end = false);
        }
    }
}

fn receive_callback(_t: &mut Threshold, mut r: USART2::Resources) {
    let data = r.RX.read().unwrap();

    if data == 10 {
        *r.END = true;
    } 
    
    r.RB.enqueue(data);
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}