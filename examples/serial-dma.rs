#![no_std]
#![no_main]


#[macro_use]
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
use heapless::consts::U8;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use rtfm::{app, Threshold, Resource};

use embedded_hal::prelude::*;

use hal::rcc::RccExt;
use hal::gpio::GpioExt;
use hal::flash::FlashExt;
use hal::serial::Serial;
use hal::serial::{Rx, Event, Tx, ReleaseInterupt, ClearInterupt};
use hal::time::U32Ext;
use hal::dma::DmaExt;
use hal::dma::CircBufferLinear;
use hal::dma::dma1;
use hal::stm32l052;
use hal::stm32l052::USART1 as USART1_p;
use hal::gpio::{Output, PushPull, gpioa::PA5};

enum Command {
    Enable,
    Disable,
    Nothing,
}

const SPACE: u8 = b' ';
const LF: u8 = 10;
const SIZE_BUF: usize = 16;

app! {
    device: stm32l052,
    resources: {
        static END: bool = false;
        static TX: Tx<USART1_p>;
        static RI: ReleaseInterupt<USART1_p>;
        static LED: PA5<Output<PushPull>>;
        static CB: CircBufferLinear<[u8; SIZE_BUF], dma1::C3>;
    },
    idle: {
        resources: [LED, END, TX, CB],
    },
    tasks: {
        USART1: {
            priority: 1,
            path: end_frame_callback,
            resources: [END, RI],
        },
    }
}

fn init(mut p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut flash = p.device.FLASH.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.iop);
    let mut channels = p.device.DMA1.split(&mut rcc.ahb);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

    let tx = gpioa.pa9.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrh);
    let rx = gpioa.pa10.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrh);

    let serial = Serial::usart1(
        p.device.USART1,
        (tx, rx),
        9_600.bps(),
        clocks,
        &mut rcc.apb2,
        Some(LF)
    );

    serial.listen(Event::Cmie);

    let (mut tx, mut rx, mut ri) = serial.split();

    let buf = singleton!(: [[u8; SIZE_BUF]; 1] = [[0; SIZE_BUF]; 1]).unwrap();
    let cb = rx.circ_buf_linear(channels.3, buf);

    init::LateResources {
        LED: led,
        TX: tx,
        RI: ri,
        CB: cb,
    }
}

fn idle(mut t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        rtfm::wfi();

        // bkpt();

        if r.END.claim(&mut t, |end, _| *end) {
            let (buf1, buf2) = r.CB.partial_peek();

            let mut vec: Vec<u8, U8> = Vec::new();

            for e in buf1.iter().chain(buf2.iter()){
                if *e != SPACE && *e != LF {
                    vec.push(*e);
                }
            }

            for v in &vec[..] {
                block!(r.TX.write(*v)).ok();
            }

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
        }
        
        r.END.claim_mut(&mut t, |end, _| *end = false);
    }
}

fn end_frame_callback(_t: &mut Threshold, mut r: USART1::Resources) {
    *r.END = true;
    r.RI.clear_isr_cmie();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}