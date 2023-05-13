use core::fmt::{Display, Formatter};
use fugit::Duration;
use stm32f4xx_hal::{
    gpio::ExtiPin,
    hal::digital::v2::{InputPin, OutputPin},
};

use crate::PERIOD;

const TOGGLE_DURATION: Duration<u32, 1, 1_000> = Duration::<u32, 1, 1_000>::millis(1_000);
const TICK_COUNT: u32 = TOGGLE_DURATION.to_millis() / PERIOD.to_millis();

pub struct StateMachine<PIN, BTN> {
    state: State,
    pin: PIN,
    btn: BTN,
    cnt: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum State {
    Off,
    On,
    Disabled,
}

impl Display for State {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            State::Off => f.write_str("Off"),
            State::On => f.write_str("On"),
            State::Disabled => f.write_str("Disabled"),
        }
    }
}

impl<PIN: OutputPin, BTN: InputPin> StateMachine<PIN, BTN>
where
    PIN::Error: core::fmt::Debug,
    BTN::Error: core::fmt::Debug,
{
    pub fn new(pin: PIN, btn: BTN) -> Self {
        Self {
            state: State::Off,
            pin,
            btn,
            cnt: 0,
        }
    }

    pub fn tick(&mut self, cs: &cortex_m::interrupt::CriticalSection) {
        let btn = self.btn.is_low().unwrap();

        // #[cfg(debug_assertions)]
        {
            use core::fmt::Write;
            let mut usart = crate::USART.borrow(cs).borrow_mut();
            let usart = usart.as_mut().unwrap();
            writeln!(
                usart,
                "btn: {:<5} | cnt: {:<2} | state: {:<8}",
                btn, self.cnt, self.state
            )
            .unwrap();
        }

        // Transitions
        match self.state {
            State::Off | State::On if self.cnt != TICK_COUNT - 1 => {
                if btn {
                    self.state = State::Disabled;
                } else {
                    self.cnt += 1;
                }
            }
            State::Off => {
                self.state = State::On;
                self.cnt = 0;
            }
            State::On => {
                self.state = State::Off;
                self.cnt = 0;
            }
            State::Disabled => {
                if !btn {
                    self.state = State::Off;
                }
            }
        };

        // Actions
        self.actions();
    }

    fn actions(&mut self) {
        match self.state {
            State::Off => self.pin.set_low().unwrap(),
            State::On => self.pin.set_high().unwrap(),
            State::Disabled => self.pin.set_low().unwrap(),
        }
    }
}

impl<PIN, BTN> StateMachine<PIN, BTN>
where
    PIN: OutputPin,
    PIN::Error: core::fmt::Debug,
    BTN: InputPin + ExtiPin,
    BTN::Error: core::fmt::Debug,
{
    pub fn handle_btn_interrupt(&mut self, _cs: &cortex_m::interrupt::CriticalSection) {
        let btn = self.btn.is_low().unwrap();

        self.state = if btn {
            State::Disabled
        } else {
            match self.state {
                State::Disabled => State::Off,
                s => s,
            }
        };

        self.actions();

        self.btn.clear_interrupt_pending_bit();
    }
}
