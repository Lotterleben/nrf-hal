//! HAL interface to the PWM peripheral

#[cfg(any(feature = "52832", feature = "52840"))]
use crate::target::PWM0;

use crate::{
    gpio::{Floating, Output, Pin},
    slice_in_ram_or,
    target_constants::EASY_DMA_SIZE,
};

pub use pwm0::prescaler::PRESCALERW as Prescaler;

/// Interface to a PWM instance

pub struct Pwm<T>(T);



impl<T> pwm<T>
where
    T: Instance,
{
    pub fn new(pwm: T, channels: PSEL, prescaler: Prescaler) -> Self {

        // Select pins
        pwm.psel.out.write(|w| {
            let w = unsafe { w.pin().bits(channels.pwm_ch0.pin) };
            w.connect().connected()
        });

        // Enable TWIM instance
        pwm.enable.write(|w| w.enable().enabled());

        // Configure frequency
        twm.prescaler.write(|w| w.prescaler().variant(prescaler));

        Pwm(pwm)
    }

    /// Write to an I2C slave
    ///
    /// The buffer must have a length of at most 255 bytes on the nRF52832
    /// and at most 65535 bytes on the nRF52840.
    pub fn write(&mut self, address: u8, buffer: &[u8]) -> Result<(), Error> {
        slice_in_ram_or(buffer, Error::DMABufferNotInDataMemory)?;

        if buffer.len() > EASY_DMA_SIZE {
            return Err(Error::TxBufferTooLong);
        }

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);

        self.0
            .address
            .write(|w| unsafe { w.address().bits(address) });

        // Set up the DMA write
        self.0.txd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the I2C transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(buffer.as_ptr() as u32) });
        self.0.txd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to `u8` is also fine.
            //
            // The MAXCNT field is 8 bits wide and accepts the full range of
            // values.
            unsafe { w.maxcnt().bits(buffer.len() as _) });

        // Start write operation
        self.0.tasks_starttx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait until write operation is about to end
        while self.0.events_lasttx.read().bits() == 0 {}
        self.0.events_lasttx.write(|w| w); // reset event

        // Stop read operation
        self.0.tasks_stop.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait until write operation has ended
        while self.0.events_stopped.read().bits() == 0 {}
        self.0.events_stopped.write(|w| w); // reset event

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        if self.0.txd.amount.read().bits() != buffer.len() as u32 {
            return Err(Error::Transmit);
        }

        Ok(())
    }

    /// Read from an I2C slave
    ///
    /// The buffer must have a length of at most 255 bytes on the nRF52832
    /// and at most 65535 bytes on the nRF52840.
    pub fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // NOTE: RAM slice check is not necessary, as a mutable slice can only be
        // built from data located in RAM

        if buffer.len() > EASY_DMA_SIZE {
            return Err(Error::RxBufferTooLong);
        }

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);

        self.0
            .address
            .write(|w| unsafe { w.address().bits(address) });

        // Set up the DMA read
        self.0.rxd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the I2C transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(buffer.as_mut_ptr() as u32) });
        self.0.rxd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to the type of maxcnt
            // is also fine.
            //
            // Note that that nrf52840 maxcnt is a wider
            // type than a u8, so we use a `_` cast rather than a `u8` cast.
            // The MAXCNT field is thus at least 8 bits wide and accepts the
            // full range of values that fit in a `u8`.
            unsafe { w.maxcnt().bits(buffer.len() as _) });

        // Start read operation
        self.0.tasks_startrx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait until read operation is about to end
        while self.0.events_lastrx.read().bits() == 0 {}
        self.0.events_lastrx.write(|w| w); // reset event

        // Stop read operation
        self.0.tasks_stop.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait until read operation has ended
        while self.0.events_stopped.read().bits() == 0 {}
        self.0.events_stopped.write(|w| w); // reset event

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        if self.0.rxd.amount.read().bits() != buffer.len() as u32 {
            return Err(Error::Receive);
        }

        Ok(())
    }

    /// Write data to an I2C slave, then read data from the slave without
    /// triggering a stop condition between the two
    ///
    /// The buffer must have a length of at most 255 bytes.
    pub fn write_then_read(
        &mut self,
        address: u8,
        wr_buffer: &[u8],
        rd_buffer: &mut [u8],
    ) -> Result<(), Error> {
        // NOTE: RAM slice check for `rd_buffer` is not necessary, as a mutable
        // slice can only be built from data located in RAM
        slice_in_ram_or(wr_buffer, Error::DMABufferNotInDataMemory)?;

        if wr_buffer.len() > EASY_DMA_SIZE {
            return Err(Error::TxBufferTooLong);
        }

        if rd_buffer.len() > EASY_DMA_SIZE {
            return Err(Error::RxBufferTooLong);
        }

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);

        self.0
            .address
            .write(|w| unsafe { w.address().bits(address) });

        // Set up the DMA write
        self.0.txd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the I2C transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(wr_buffer.as_ptr() as u32) });
        self.0.txd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to `u8` is also fine.
            //
            // The MAXCNT field is 8 bits wide and accepts the full range of
            // values.
            unsafe { w.maxcnt().bits(wr_buffer.len() as _) });

        // Set up the DMA read
        self.0.rxd.ptr.write(|w|
            // We're giving the register a pointer to the stack. Since we're
            // waiting for the I2C transaction to end before this stack pointer
            // becomes invalid, there's nothing wrong here.
            //
            // The PTR field is a full 32 bits wide and accepts the full range
            // of values.
            unsafe { w.ptr().bits(rd_buffer.as_mut_ptr() as u32) });
        self.0.rxd.maxcnt.write(|w|
            // We're giving it the length of the buffer, so no danger of
            // accessing invalid memory. We have verified that the length of the
            // buffer fits in an `u8`, so the cast to the type of maxcnt
            // is also fine.
            //
            // Note that that nrf52840 maxcnt is a wider
            // type than a u8, so we use a `_` cast rather than a `u8` cast.
            // The MAXCNT field is thus at least 8 bits wide and accepts the
            // full range of values that fit in a `u8`.
            unsafe { w.maxcnt().bits(rd_buffer.len() as _) });

        // Immediately start RX after TX, then stop
        self.0
            .shorts
            .modify(|_r, w| w.lasttx_startrx().enabled().lastrx_stop().enabled());

        // Start write operation
        self.0.tasks_starttx.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Wait until total operation has ended
        while self.0.events_stopped.read().bits() == 0 {}

        self.0.events_lasttx.write(|w| w); // reset event
        self.0.events_lastrx.write(|w| w); // reset event
        self.0.events_stopped.write(|w| w); // reset event
        self.0.shorts.write(|w| w);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        let bad_write = self.0.txd.amount.read().bits() != wr_buffer.len() as u32;
        let bad_read = self.0.rxd.amount.read().bits() != rd_buffer.len() as u32;

        if bad_write {
            return Err(Error::Transmit);
        }

        if bad_read {
            return Err(Error::Receive);
        }

        Ok(())
    }

    /// Return the raw interface to the underlying TWIM peripheral
    pub fn free(self) -> T {
        self.0
    }
}



/// The pins used by the pwm peripheral
///
/// Currently, only P0 pins are supported.
pub struct PSEL {

    pub out: [OUT; 4],


}
//Change
#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    Transmit,
    Receive,
    DMABufferNotInDataMemory,
}

/// Implemented by all TWIM instances
pub trait Instance: Deref<Target = twim0::RegisterBlock> {}

impl Instance for TWIM0 {}

#[cfg(any(feature = "52832", feature = "52840"))]
impl Instance for TWIM1 {}
