use core::fmt::{Display, Formatter};

use fugit::Duration;
use stm32f4xx_hal::{
    gpio::{ExtiPin, PinState},
    hal::digital::v2::{InputPin, OutputPin},
};

use crate::PERIOD;

const TOGGLE_DURATION: Duration<u32, 1, 1_000> = Duration::<u32, 1, 1_000>::millis(500);
const TICK_COUNT: u8 = (TOGGLE_DURATION.to_millis() / PERIOD.to_millis()) as u8;

pub struct StateMachine<const BITS: usize, PIN, BTN> {
    state: State,
    pins: [PIN; BITS],
    btn: BTN,
    cnt: u8,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum State {
    Off,
    On,
}

impl Display for State {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            State::Off => f.write_str("Off"),
            State::On => f.write_str("On"),
        }
    }
}

impl<const BITS: usize, PIN: OutputPin, BTN: InputPin> StateMachine<BITS, PIN, BTN>
where
    PIN::Error: core::fmt::Debug,
    BTN::Error: core::fmt::Debug,
{
    const MAX_COUNT: u8 = (1 << BITS) - 1;
    const MAX_TICK: u8 = (Self::MAX_COUNT + 1) * TICK_COUNT - 1;

    pub fn new(pins: [PIN; BITS], btn: BTN) -> Self {
        Self {
            state: State::Off,
            pins,
            btn,
            cnt: 0,
        }
    }

    pub fn tick(&mut self, cs: &cortex_m::interrupt::CriticalSection) {
        let btn = self.btn.is_low().unwrap();

        // Transitions
        self.state = match self.state {
            State::Off if !btn => State::On,
            State::Off => State::Off,
            State::On if btn => State::Off,
            State::On => {
                self.cnt = if self.cnt == Self::MAX_TICK {
                    0
                } else {
                    self.cnt + 1
                };
                State::On
            }
        };

        // Actions
        self.actions().unwrap();

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
    }

    fn actions(&mut self) -> Result<(), PIN::Error> {
        match self.state {
            State::Off => self.set_pins(0),
            State::On => self.set_pins(self.cnt / TICK_COUNT),
        }
    }

    fn set_pins(&mut self, n: u8) -> Result<(), PIN::Error> {
        for (i, pin) in self.pins.iter_mut().enumerate() {
            let state = PinState::from(((n >> i) & 0x01) == 1);
            pin.set_state(state)?;
        }
        Ok(())
    }
}

impl<const BITS: usize, PIN, BTN> StateMachine<BITS, PIN, BTN>
where
    PIN: OutputPin,
    PIN::Error: core::fmt::Debug,
    BTN: InputPin + ExtiPin,
    BTN::Error: core::fmt::Debug,
{
    pub fn handle_btn_interrupt(&mut self, _cs: &cortex_m::interrupt::CriticalSection) {
        let btn = self.btn.is_low().unwrap();

        self.state = if btn {
            State::Off
        } else {
            match self.state {
                State::Off => State::On,
                s => s,
            }
        };

        self.actions().unwrap();

        self.btn.clear_interrupt_pending_bit();
    }
}
