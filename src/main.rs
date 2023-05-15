#![warn(clippy::pedantic, clippy::nursery)]
#![allow(clippy::module_name_repetitions)]
#![no_std]
#![no_main]

mod state_machine;

use core::{
    cell::{Cell, RefCell},
    fmt::Write,
    panic::PanicInfo,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m::{asm, interrupt::Mutex};
use cortex_m_rt::entry;
use fugit::Duration;
use stm32f4xx_hal::{
    self as hal,
    gpio::{EPin, Edge, Output, PushPull, PC13},
    interrupt,
    pac::{TIM2, USART2},
    prelude::*,
    serial::Serial,
    timer::CounterUs,
};

use crate::state_machine::StateMachine;

type StateMachineImpl = StateMachine<3, EPin<Output<PushPull>>, PC13>;

/// Timer period.
const PERIOD: Duration<u32, 1, 1_000> = Duration::<u32, 1, 1_000>::millis(500);

// Global resources

/// Timer that ticks every [`PERIOD`].
///
/// This can be a `Cell` b/c we are only moving it around, not mutating it.
static G_TIM: Mutex<Cell<Option<CounterUs<TIM2>>>> = Mutex::new(Cell::new(None));
/// Flag that is set when the timer ticks.
///
/// Make sure to set this to `false` when you are done with it.
static TIM_FLAG: AtomicBool = AtomicBool::new(false);
/// USART2 serial interface.
static USART: Mutex<RefCell<Option<Serial<USART2>>>> = Mutex::new(RefCell::new(None));
/// State machine.
static G_SM: Mutex<RefCell<Option<StateMachineImpl>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();

    // Setup clocks
    let rss = dp.RCC.constrain();
    let clocks = rss.cfgr.sysclk(16.MHz()).pclk1(8.MHz()).freeze();

    // Get GPIO ports
    let gpio_a = dp.GPIOA.split();
    let gpio_b = dp.GPIOB.split();
    let gpio_c = dp.GPIOC.split();

    // Setup USART
    let cfg = hal::serial::Config::default().baudrate(57600.bps());
    let mut usart = dp
        .USART2
        .serial((gpio_a.pa2, gpio_a.pa3), cfg, &clocks)
        .unwrap();

    // Setup Output LEDs
    let mut l1 = gpio_b.pb5.into_push_pull_output();
    l1.set_low();
    let mut l2 = gpio_b.pb4.into_push_pull_output();
    l2.set_low();
    let mut l3 = gpio_b.pb10.into_push_pull_output();
    l3.set_low();
    let leds = [l1.erase(), l2.erase(), l3.erase()];

    // Setup Input Button
    let mut btn = gpio_c.pc13.into_input().internal_pull_down(true);

    // Setup Button interrupt
    let mut syscfg = dp.SYSCFG.constrain();
    let mut exti = dp.EXTI;
    btn.make_interrupt_source(&mut syscfg);
    btn.trigger_on_edge(&mut exti, Edge::Falling);
    btn.enable_interrupt(&mut exti);
    let btn_interrupt = btn.interrupt();

    // Setup TIM2 Timer
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(PERIOD.convert()).unwrap();
    timer.listen(hal::timer::Event::Update);

    let sm = StateMachine::new(leds, btn);

    writeln!(usart, "Hello, World!").unwrap();

    // Store peripherals in static Mutexes
    cortex_m::interrupt::free(|cs| {
        G_TIM.borrow(cs).set(Some(timer));
        USART.borrow(cs).replace(Some(usart));
        G_SM.borrow(cs).replace(Some(sm));
    });

    // Enable TIM2 & Button interrupts
    unsafe {
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::TIM2);

        // Enable EXTI15_10 interrupt
        cortex_m::peripheral::NVIC::unmask(btn_interrupt);
    }

    loop {
        // Wait for interrupt flag
        while !TIM_FLAG.load(Ordering::Relaxed) {
            // Put processor to sleep
            asm::wfe();
        }

        cortex_m::interrupt::free(|cs| {
            // Clear flag
            TIM_FLAG.store(false, Ordering::Relaxed);

            // Tick state machine
            G_SM.borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .tick(cs)
                .unwrap();
        });
    }
}

#[interrupt]
fn TIM2() {
    static mut TIM: Option<CounterUs<TIM2>> = None;

    // Move timer out of static Mutex into local static variable
    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TIM.borrow(cs).replace(None).unwrap())
    });

    // Set flag
    TIM_FLAG.store(true, Ordering::Relaxed);

    // Clear interrupt flag
    let _ = tim.wait();
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        // Print to USART
        USART
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str("Button pressed!\r\n")
            .unwrap();

        // Handle button interrupt
        G_SM.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .handle_btn_interrupt(cs)
            .unwrap();
    });
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // We've panicked!
    // Disable interrupts, to ensure we are stuck here
    cortex_m::interrupt::disable();

    // Print panic message to USART
    cortex_m::interrupt::free(|cs| {
        if let Some(usart) = USART.borrow(cs).borrow_mut().as_mut() {
            writeln!(usart, "PANIC: {}", info).unwrap();
        }
    });

    loop {}
}
