use core::cell::Cell;
use core::convert::Infallible;

use embassy_rp::adc::Adc;
use embassy_rp::gpio::{Flex, Level, Output};
use embassy_rp::peripherals::{PIN_24, PIN_25, PIN_29};
use embassy_rp::Peripheral;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal_1::spi::ErrorType;
use embedded_hal_async::spi::{SpiBusFlush, SpiBusRead, SpiBusWrite};
use futures::Future;

pub struct State {
    shared: Mutex<NoopRawMutex, Cell<SharedState>>,
}

struct SharedState {
    cs: Output<'static, PIN_25>,
    di_do_irq: Option<Flex<'static, PIN_24>>,
    sclk_pin: PIN_29,
}

impl State {
    pub fn new(pin24: PIN_24, pin25: PIN_25, pin29: PIN_29) -> Self {
        let cs = Output::new(pin25, Level::High);
        let mut di_do_irq = Flex::new(pin24);
        di_do_irq.set_low();
        di_do_irq.set_as_output();

        State {
            shared: Mutex::new(Cell::new(SharedState {
                cs,
                di_do_irq: Some(di_do_irq),
                sclk_pin: pin29,
            })),
        }
    }
}

pub struct VSys<'d> {
    state: &'d Mutex<NoopRawMutex, Cell<SharedState>>,
}

impl<'d> VSys<'d> {
    pub async fn read_vsys_voltage(&self, adc: &mut Adc<'static>) -> f32 {
        let mut state_guard = self.state.lock().await;
        let state = state_guard.get_mut();

        let adc_reading = adc.read(&mut state.sclk_pin).await;

        adc_reading as f32 * 3.0 * 3.3 / 4096.0
    }
}

pub struct Cyw43Spi<'d> {
    state: &'d Mutex<NoopRawMutex, Cell<SharedState>>,
}

pub fn new<'d>(state: &'d mut State) -> (VSys<'d>, Cyw43Spi<'d>) {
    (VSys { state: &state.shared }, Cyw43Spi { state: &state.shared })
}

impl<'d> ErrorType for Cyw43Spi<'d> {
    type Error = Infallible;
}
unsafe impl<'d> embedded_hal_async::spi::SpiDevice for Cyw43Spi<'d> {
    type Bus = RpRadioSpi;
    async fn transaction<R, F, Fut>(&mut self, f: F) -> Result<R, Infallible>
    where
        F: FnOnce(*mut Self::Bus) -> Fut,
        Fut: Future<Output = Result<R, <Self::Bus as ErrorType>::Error>>,
    {
        let mut state_guard = self.state.lock().await;
        let state = state_guard.get_mut();

        // Could borrow it to RpRadioSpi, but that would require feature(impl_trait_projections).
        // Instead clone and let the clone usage go out of scope...
        //
        // `async fn` return type cannot contain a projection or `Self` that references lifetimes from a parent scope
        // see issue #103532 <https://github.com/rust-lang/rust/issues/103532> for more information
        // add `#![feature(impl_trait_projections)]` to the crate attributes to enable
        let sclk_pin = unsafe { state.sclk_pin.clone_unchecked() };
        let clk = Output::new(sclk_pin, Level::Low);
        if let Some(di_do_irq) = state.di_do_irq.take() {
            let mut bus = RpRadioSpi { clk, di_do_irq };
            state.cs.set_low();
            let f_res = f(&mut bus).await?;

            // On failure, it's important to still flush and deassert CS.
            if let Err(_) = bus.flush().await {
                panic!();
            }
            state.cs.set_high();
            let di_do_irq = bus.desolve();
            state.di_do_irq.replace(di_do_irq);
            Ok(f_res)
        } else {
            panic!()
        }
    }
}

pub struct RpRadioSpi {
    clk: Output<'static, PIN_29>,
    di_do_irq: Flex<'static, PIN_24>,
}
impl RpRadioSpi {
    fn desolve(self) -> Flex<'static, PIN_24> {
        self.di_do_irq
    }
}

impl ErrorType for RpRadioSpi {
    type Error = Infallible;
}

impl SpiBusFlush for RpRadioSpi {
    #[inline]
    async fn flush(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

impl SpiBusRead<u32> for RpRadioSpi {
    async fn read(&mut self, words: &mut [u32]) -> Result<(), Self::Error> {
        self.di_do_irq.set_as_input();
        for word in words {
            let mut w = 0;
            for _ in 0..32 {
                w = w << 1;

                // rising edge, sample data
                if self.di_do_irq.is_high() {
                    w |= 0x01;
                } else {
                }
                self.clk.set_high();

                // falling edge
                self.clk.set_low();
            }
            *word = w
        }
        Ok(())
    }
}

impl SpiBusWrite<u32> for RpRadioSpi {
    async fn write(&mut self, words: &[u32]) -> Result<(), Self::Error> {
        self.di_do_irq.set_as_output();
        for word in words {
            let mut word = *word;
            for _ in 0..32 {
                // falling edge, setup data
                self.clk.set_low();
                if word & 0x8000_0000 == 0 {
                    self.di_do_irq.set_low();
                } else {
                    self.di_do_irq.set_high();
                }
                // rising edge
                self.clk.set_high();

                word = word << 1;
            }
        }
        self.clk.set_low();
        self.di_do_irq.set_as_input();
        Ok(())
    }
}
