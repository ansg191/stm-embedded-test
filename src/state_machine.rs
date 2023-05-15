use core::fmt::{Display, Formatter};

use fugit::Duration;
use stm32f4xx_hal::{
    gpio::{ExtiPin, PinState},
    hal::digital::v2::{InputPin, OutputPin},
};

use crate::PERIOD;

/// How long to wait before incrementing the output counter.
const COUNT_DURATION: Duration<u32, 1, 1_000> = Duration::<u32, 1, 1_000>::millis(500);
/// How many timer interrupts to wait before incrementing the output counter.
///
/// Set by [`COUNT_DURATION`] / [`PERIOD`]
#[allow(clippy::cast_possible_truncation)]
const TICK_COUNT: u8 = (COUNT_DURATION.to_millis() / PERIOD.to_millis()) as u8;

/// State machine that controls the output pins.
///
/// This state machine increments the output pins every [`COUNT_DURATION`].
/// If the button is pressed, the state machine pauses.
///
/// # Type Parameters
///
/// - `BITS`: The number of output pins.
/// - `PIN`: The type of the output pins.
/// - `BTN`: The type of the input button pin.
pub struct StateMachine<const BITS: usize, PIN, BTN> {
    state: State,
    pins: [PIN; BITS],
    btn: BTN,
    cnt: u8,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum State {
    Paused,
    On,
}

impl Display for State {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Paused => f.write_str("Paused"),
            Self::On => f.write_str("On"),
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

    /// Creates a new state machine.
    ///
    /// # Arguments
    ///
    /// * `pins`: The output pins.
    /// * `btn`: The input button pin.
    ///
    /// returns: `StateMachine<{ BITS }, PIN, BTN>`
    pub const fn new(pins: [PIN; BITS], btn: BTN) -> Self {
        Self {
            state: State::Paused,
            pins,
            btn,
            cnt: 0,
        }
    }

    /// Ticks the state machine.
    ///
    /// This function requires a critical section to ensure no interrupts are fired during the
    /// processing of the tick.
    ///
    /// # Arguments
    ///
    /// * `cs`: The critical section from [`cortex_m::interrupt::free`].
    pub fn tick(
        &mut self,
        cs: &cortex_m::interrupt::CriticalSection,
    ) -> Result<(), StateMachineError<PIN, BTN>> {
        let btn = self.btn.is_low().map_err(StateMachineError::ButtonError)?;

        // Transitions
        self.state = match self.state {
            State::Paused if !btn => State::On,
            State::Paused => State::Paused,
            State::On if btn => State::Paused,
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
        self.actions().map_err(StateMachineError::PinError)?;

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

        Ok(())
    }

    fn actions(&mut self) -> Result<(), PIN::Error> {
        match self.state {
            State::Paused | State::On => self.set_pins(self.cnt / TICK_COUNT),
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
    /// Handles the button interrupt.
    ///
    /// This function requires a critical section to ensure no interrupts are fired during the
    /// handling.
    ///
    /// # Arguments
    ///
    /// * `_cs`: The critical section from [`cortex_m::interrupt::free`].
    pub fn handle_btn_interrupt(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
    ) -> Result<(), StateMachineError<PIN, BTN>> {
        let btn = self.btn.is_low().map_err(StateMachineError::ButtonError)?;

        self.state = if btn {
            State::Paused
        } else {
            match self.state {
                State::Paused => State::On,
                s @ State::On => s,
            }
        };

        self.actions().map_err(StateMachineError::PinError)?;

        self.btn.clear_interrupt_pending_bit();

        Ok(())
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum StateMachineError<PIN: OutputPin, BTN: InputPin> {
    PinError(PIN::Error),
    ButtonError(BTN::Error),
}
