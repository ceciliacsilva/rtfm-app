#![feature(panic_implementation)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_rtfm as rtfm;
// extern crate stm32l052;
extern crate hal;
extern crate embedded_hal;
extern crate cortex_m_semihosting as sh;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
// use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Threshold, Resource};

use hal::rcc::RccExt;
use hal::gpio::GpioExt;
use hal::flash::FlashExt;
use hal::serial::Serial;
use hal::time::U32Ext;
use hal::dma::DmaExt;
// use hal::dma::{CircBuffer, dma1, Event};
use hal::serial::{Rx, Event};

use embedded_hal::prelude::*;

use hal::stm32l0;
use hal::stm32l0::stm32l0x1::USART2 as USART2_p;
// use stm32l052::GPIOA;

use hal::gpio::{Output, PushPull, gpioa::PA5};

use core::fmt::Write;
use sh::hio;

pub struct ToSave{
    data: u8,
}

// pub trait Pegar {
//     fn pegar (self) -> u8;
// }

impl ToSave {
    fn new (data: u8) -> Self {
        ToSave{
            data: data,
        }
    }
    fn pegar (&mut self) -> u8 {
        self.data
    }
}

// impl Pegar for ToSave {
//     fn pegar (self) -> u8 {
//         self.data
//     }
// }

app! {
    device: stm32l0::stm32l0x1,
    resources: {
        // static ON: bool = false;
        // static BUFFER: [[u8; 8]; 2] = [[0; 8]; 2];
        // static CB: CircBuffer<[u8; 8], dma1::C4>;
        static DATA: u8 = 0;
        static RX: Rx<USART2_p>;
        static LED: PA5<Output<PushPull>>;
    },
    idle: {
        resources: [DATA, LED],
    },
    tasks: {
        USART2: {
            path: rx,
            resources: [DATA, RX],
        }
    }
}

// static buf: [[u8; 8]; 2] = [[0; 8]; 2];

fn init(mut p: init::Peripherals, r: init::Resources) -> init::LateResources {
    // `init` can modify all the `resources` declared in `app!`

    // p.device.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());

    // p.device.GPIOA.moder.modify(|_, w| unsafe{ w.mode5().bits(1) });

    // // p.device.GPIOA.odr.modify(|_, w| w.od5().set_bit());
    
    // p.core.SYST.set_clock_source(SystClkSource::Core);
    // p.core.SYST.set_reload(2_000_000); // 1s
    // p.core.SYST.clear_current();
    // p.core.SYST.enable_counter();
    // p.core.SYST.enable_interrupt();

    let mut rcc = p.device.RCC.constrain();
    let mut flash = p.device.FLASH.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.iop);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);

    let tx = gpioa.pa2.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);
    let rx = gpioa.pa3.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);

    let serial = Serial::usart2(
        p.device.USART2,
        (tx, rx),
        9_600.bps(),
        clocks,
        &mut rcc.apb1,
    );

    serial.listen(Event::Rxne);

    let (mut tx, mut rx) = serial.split();


    // let mut channels = p.device.DMA1.split(&mut rcc.ahb);
    // channels.4.listen(Event::HalfTransfer);
    // channels.4.listen(Event::TransferComplete);

    init::LateResources {
        RX: rx,
        LED: led,
    }
}

fn idle(t: &mut Threshold, r: idle::Resources) -> ! {
    
    let mut hstdout = hio::hstdout().unwrap();

    loop {
        rtfm::wfi();

        r.LED.set_high();

        r.DATA.claim(t, |data, _| writeln!(hstdout,"uart data: {}", *data).unwrap() );

        // let data = *r.DATA;
        // let data = r.DATA.data;
        // if data == 1 {
        //     r.LED.set_high();
        // }
        // else {
        //     r.LED.set_low();
        // }
    }
}

fn rx(_t: &mut Threshold, mut r: USART2::Resources) {
    // r.CB
    //     .peek(|_buf, _half| {
    //         bkpt();
    //     })
    //     .unwrap();
    // bkpt();

    let a = r.RX.read().unwrap();

    *r.DATA = a;

    // bkpt();
}


#[allow(deprecated)]
#[panic_implementation]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}