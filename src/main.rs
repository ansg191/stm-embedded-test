#![no_std]
#![no_main]

mod state_machine;

use core::{
    cell::{Cell, RefCell},
    fmt::Write,
    panic::PanicInfo,
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

const PERIOD: Duration<u32, 1, 1_000> = Duration::<u32, 1, 1_000>::millis(500);

static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static USART: Mutex<RefCell<Option<Serial<USART2>>>> = Mutex::new(RefCell::new(None));

static TIM_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

type StateMachineImpl = StateMachine<3, EPin<Output<PushPull>>, PC13>;

static G_SM: Mutex<RefCell<Option<StateMachineImpl>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = hal::pac::Peripherals::take().unwrap();

    // Setup clocks
    let rss = dp.RCC.constrain();
    let clocks = rss.cfgr.sysclk(16.MHz()).pclk1(8.MHz()).freeze();

    // Get GPIO ports
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // Setup USART
    let cfg = hal::serial::Config::default().baudrate(57600.bps());
    let mut usart = dp
        .USART2
        .serial((gpioa.pa2, gpioa.pa3), cfg, &clocks)
        .unwrap();

    // Setup Output LEDs
    let mut led1 = gpiob.pb5.into_push_pull_output();
    led1.set_low();
    let mut led2 = gpiob.pb4.into_push_pull_output();
    led2.set_low();
    let mut led3 = gpiob.pb10.into_push_pull_output();
    led3.set_low();
    let leds = [led1.erase(), led2.erase(), led3.erase()];

    // Setup Input Button
    let mut btn = gpioc.pc13.into_input().internal_pull_down(true);

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
    writeln!(
        usart,
        "Size of StateMachine: {}",
        core::mem::size_of::<StateMachineImpl>()
    )
    .unwrap();
    writeln!(usart, "Size of USART: {}", core::mem::size_of_val(&USART)).unwrap();

    // Store peripherals in static Mutexes
    cortex_m::interrupt::free(|cs| {
        G_TIM.borrow(cs).replace(Some(timer));
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
        while !cortex_m::interrupt::free(|cs| TIM_FLAG.borrow(cs).get()) {
            // Put processor to sleep
            asm::wfe();
        }

        cortex_m::interrupt::free(|cs| {
            // Clear flag
            TIM_FLAG.borrow(cs).set(false);

            // Tick state machine
            G_SM.borrow(cs).borrow_mut().as_mut().unwrap().tick(cs);
        });
    }
}

#[interrupt]
fn TIM2() {
    static mut TIM: Option<CounterUs<TIM2>> = None;

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TIM.borrow(cs).replace(None).unwrap())
    });

    cortex_m::interrupt::free(|cs| {
        TIM_FLAG.borrow(cs).set(true);
    });

    let _ = tim.wait();
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        USART
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write_str("Button pressed!\r\n")
            .unwrap();
        G_SM.borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .handle_btn_interrupt(cs);
    });
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    cortex_m::interrupt::free(|cs| {
        if let Some(usart) = USART.borrow(cs).borrow_mut().as_mut() {
            writeln!(usart, "PANIC: {}", info).unwrap();
        }
    });

    loop {}
}
