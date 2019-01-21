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
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use embedded_hal::digital::OutputPin;

#[doc = "LED::e7khl82munooyu6x"]
static mut e7khl82munooyu6x: rtfm::export::MaybeUninit<PA5<Output<PushPull>>>
       =
    rtfm::export::MaybeUninit::uninitialized();
impl <'a> rtfm::Mutex for resources::LED<'a> {
    type
    T
    =
    PA5<Output<PushPull>>;
    #[inline]
    fn lock<R, F>(&mut self, f: F) -> R where F: FnOnce(&mut Self::T) -> R {
        unsafe {
            rtfm::export::claim(unsafe { e7khl82munooyu6x.get_mut() },
                                &self.y4wpf62blri32k0r, 2u8,
                                narc_hal::stm32l052::NVIC_PRIO_BITS, f)
        }
    }
}
#[doc = r" Resource proxies"]
pub mod resources {
    #[doc = "`PA5 < Output < PushPull > >`"]
    pub struct LED<'a> {
        #[doc(hidden)]
        pub y4wpf62blri32k0r: &'a core::cell::Cell<u8>,
    }
}
#[doc = "`EXTI4_15::Resources`"]
#[allow(non_snake_case)]
pub struct c8qww1ppw9g162n7<'a> {
    #[allow(dead_code)]
    y4wpf62blri32k0r: &'a core::cell::Cell<u8>,
    pub LED: rtfm::Exclusive<'a, PA5<Output<PushPull>>>,
}
#[doc = "Interrupt handler"]
pub mod EXTI4_15 {
    #[doc = r" Variables injected into this context by the `app` attribute"]
    pub struct Context<'a> {
        #[doc = r" Resources available in this context"]
        pub resources: Resources<'a>,
    }
    #[doc(inline)]
    pub use super::c8qww1ppw9g162n7 as Resources;
}
const APP: () =
    {
        #[export_name = "EXTI4_15"]
        fn j9134f710w11e608() {
            let _ = narc_hal::stm32l052::interrupt::EXTI4_15;
            let ref y4wpf62blri32k0r = core::cell::Cell::new(2u8);
            #[allow(unused_variables)]
            #[allow(unsafe_code)]
            #[allow(unused_mut)]
            let mut resources =
                unsafe {
                    c8qww1ppw9g162n7{y4wpf62blri32k0r,
                                     LED:
                                         rtfm::Exclusive(e7khl82munooyu6x.get_mut()),}
                };
            use rtfm::Mutex;
            rtfm::export::run(move ||
                                  {



                                      //PA0

                                      // led.set_high();






                                      resources.LED.set_high();
                                  })
        }
    };
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
fn init_a9wm(mut core: rtfm::Peripherals) {
    let mut device = unsafe { narc_hal::stm32l052::Peripherals::steal() };
    let mut rcc = device.RCC.constrain();
    let mut gpioa = device.GPIOA.split(&mut rcc.iop);
    let mut led =
        gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe { w.exti4().bits(0) });
    device.EXTI.imr.modify(|_, w| w.im4().bit(true));
    device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));
    unsafe { e7khl82munooyu6x.set(led); }
}
#[doc = "`idle::Resources`"]
#[allow(non_snake_case)]
pub struct d3544a71u8pm5661<'a> {
    #[allow(dead_code)]
    y4wpf62blri32k0r: &'a core::cell::Cell<u8>,
    pub LED: resources::LED<'a>,
}
#[doc = "Idle loop"]
pub mod idle {
    #[doc = r" Variables injected into this context by the `app` attribute"]
    pub struct Context<'a> {
        #[doc = r" Resources available in this context"]
        pub resources: Resources<'a>,
    }
    #[doc(inline)]
    pub use super::d3544a71u8pm5661 as Resources;
}
fn idle_b3hm() -> ! {
    let ref y4wpf62blri32k0r = core::cell::Cell::new(0u8);
    #[allow(unused_variables)]
    #[allow(unsafe_code)]
    #[allow(unused_mut)]
    let mut resources =
        {
            d3544a71u8pm5661{y4wpf62blri32k0r,
                             LED: resources::LED{y4wpf62blri32k0r,},}
        };
    use rtfm::Mutex;
    resources.LED.lock(|led| led.set_low());
    wfi();
    loop  { }
}
#[export_name = "main"]
#[allow(unsafe_code)]
#[doc(hidden)]
unsafe fn i6wfa33bsdiff94s() -> ! {
    rtfm::export::assert_send::<PA5<Output<PushPull>>>();
    rtfm::export::interrupt::disable();
    let mut p = rtfm::export::Peripherals::steal();
    p.NVIC.enable(narc_hal::stm32l052::Interrupt::EXTI4_15);
    if !(2u8 <= (1 << narc_hal::stm32l052::NVIC_PRIO_BITS)) {

        // Will not leave the interrupt without the next line.
        // resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));




        {
            $crate::panicking::panic(&("assertion failed: 2u8 <= (1 << narc_hal::stm32l052::NVIC_PRIO_BITS)",
                                       "examples/one-resource.rs", 21u32,
                                       1u32))
        }
    };
    p.NVIC.set_priority(narc_hal::stm32l052::Interrupt::EXTI4_15,
                        ((1 << narc_hal::stm32l052::NVIC_PRIO_BITS) - 2u8) <<
                            (8 - narc_hal::stm32l052::NVIC_PRIO_BITS));
    init_a9wm(rtfm::Peripherals{CBP: p.CBP,
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
    idle_b3hm()
}
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();
    loop  { atomic::compiler_fence(Ordering::SeqCst) }
}
