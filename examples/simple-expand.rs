#![feature(prelude_import)]
#![no_std]
#![no_main]
#![no_std]
#[prelude_import]
use ::core::prelude::v1::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;

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
use embedded_hal::digital::OutputPin;

const APP: () = { };
#[doc = "Initialization function"]
pub mod init {
    #[doc = r" Variables injected into this context by the `app` attribute"]
    pub struct Context<'a> {
        #[doc = r" Core (Cortex-M) peripherals"]
        pub core: rtfm::Peripherals<'a>,
        #[doc = r" Device specific peripherals"]
        pub device: narc_hal::stm32l052::Peripherals,
    }
}
fn init_h3pg(mut core: rtfm::Peripherals) {
    let mut device = unsafe { narc_hal::stm32l052::Peripherals::steal() };
    let mut rcc = device.RCC.constrain();
    let mut gpioa = device.GPIOA.split(&mut rcc.iop);

    let mut led =
        gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);

    led.set_high();
}
#[doc = "Idle loop"]
pub mod idle {
    #[doc = r" Variables injected into this context by the `app` attribute"]
    pub struct Context {
    }
}
fn idle_g3bo() -> ! {

    wfi();
    loop  {




        {
            $crate::panicking::panic(&("not yet implemented",
                                       "examples/simple.rs", 36u32, 13u32))
        };
    }
}
#[export_name = "main"]
#[allow(unsafe_code)]
#[doc(hidden)]
unsafe fn d1555rpdjb21gh52() -> ! {
    rtfm::export::interrupt::disable();
    let mut p = rtfm::export::Peripherals::steal();
    init_h3pg(rtfm::Peripherals{CBP: p.CBP,
                                CPUID: p.CPUID,
                                DCB: p.DCB,
                                DWT: p.DWT,
                                FPB: p.FPB,
                                FPU: p.FPU,
                                ITM: p.ITM,
                                MPU: p.MPU,
                                SCB: &mut p.SCB,
                                SYST: p.SYST,
                                TPIU: p.TPIU,});
    rtfm::export::interrupt::enable();
    idle_g3bo()
}
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();
    loop  { atomic::compiler_fence(Ordering::SeqCst) }
}
