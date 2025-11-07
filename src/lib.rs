#![cfg_attr(not(test), no_std)]

use core::{array::TryFromSliceError, convert::Infallible};

use bilge::prelude::*;
use embedded_hal::{delay::DelayNs, digital::{ErrorType, OutputPin}, spi::SpiDevice};
use paste::paste;

mod macros;
mod seal {
    pub(super) trait Sealed {}
}


#[allow(private_bounds)]
pub trait OptionalPin: seal::Sealed {}

impl<T: OutputPin + seal::Sealed> OptionalPin for T {}
impl<T: OutputPin> seal::Sealed for T {}
pub struct NoPin {}


impl ErrorType for NoPin {
    type Error = Infallible;
}
impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(()) 
    }
}

pub struct LT8722<SPI, SYNC, EN, SWEN, D>
where
	SPI: SpiDevice,
    SYNC: OptionalPin,
	EN: OptionalPin,
	SWEN: OptionalPin,
	D: DelayNs,
{
	spi:           SPI,
	_sync_pin:     SYNC,
	enable:        EN,
	switch_enable: SWEN,
	delay:         D,
	transfer_buf:  [u8; 8],
	// Keep track of what's going on on the device
	registers:     Registers,
}

pub struct Registers {
	command:   Command,
	status:    Status,
	dac_ilimn: DACILimN,
	dac_ilimp: DACILimP,
	dac:       DAC,
	ov_clamp:  OVClamp,
	uv_clamp:  UVClamp,
	amux:      AMux,
}

impl Default for Registers {
	fn default() -> Self {
		Self {
			command:   Command {
				value: u22::new(0x08A214u32),
			},
			status:    Status {
				value: u11::new(0x0u16),
			},
			dac_ilimn: DACILimN {
				value: u9::new(0x1FFu16),
			},
			dac_ilimp: DACILimP {
				value: u9::new(0x0u16),
			},
			dac:       DAC::new(i25::MIN.value()),
			ov_clamp:  OVClamp {
				value: u4::new(0xFu8),
			},
			uv_clamp:  UVClamp {
				value: u4::new(0x0u8),
			},
			amux:      AMux {
				value: u7::new(0x0u8),
			},
		}
	}
}

#[allow(non_camel_case_types)]
#[bitsize(8)]
#[derive(FromBits, Clone, Copy)]
pub enum RegAddr {
	Command,
	Status,
	DAC_ILimN,
	DAC_ILimP,
	DAC,
	Ov_Clamp,
	Uv_Clamp,
	AMux,
	#[fallback]
	Invalid,
}

pub enum SpiPacket {
	Status { addr: RegAddr },
	DataRead { addr: RegAddr },
	DataWrite { addr: RegAddr, data: u32 },
}

#[bitsize(8)]
#[derive(FromBits, Debug, defmt::Format)]
pub enum SpiAck {
	Stuck0        = 0x00,
	RejectBadAddr = 0x0F,
	Ack           = 0xA5,
	NACK          = 0xC3,
	Stuck1        = 0xFF,
	#[fallback]
	Corrupt,
}

#[derive(Debug, defmt::Format)]
pub struct SpiResponse {
	status: Status,
	data:   Option<u32>,
}

#[bitsize(22)]
#[derive(FromBits)]
pub struct Command {
	enable_req:  bool,
	swen_req:    bool,
	sw_freq_set: PwmFreqCtrl,
	sw_freq_adj: PwmFreqAdj,
	sys_dc:      SysDc,
	vcc_vreg:    VccVreg,
	reserved:    u1, // MUST BE 0
	sw_vc_int:   VcInt,
	spi_rst:     bool,
	pwr_lim:     PwrLim,
	reserved:    u3,
}

#[bitsize(3)]
#[derive(FromBits, Default)]
pub enum PwmFreqCtrl {
	_0_5MHz,
	_1MHz,
	_1_5MHz,
	_2MHz,
	_2_5MHz,
	#[fallback]
	#[default]
	_3MHz,
}

#[bitsize(2)]
#[derive(FromBits, Default)]
pub enum PwmFreqAdj {
	#[default]
	#[fallback]
	_0,
	Plus15,
	Minus15,
}

#[bitsize(2)]
#[derive(FromBits, Default)]
pub enum SysDc {
	#[default]
	_20_80,
	_15_85,
	#[fallback]
	_10_90,
}

#[bitsize(1)]
#[derive(FromBits, Default)]
pub enum VccVreg {
	_3_1V,
	#[default]
	_3_4V,
}

#[bitsize(3)]
#[derive(FromBits, Default)]
pub enum VcInt {
	_0_252A,
	_0_594A,
	_0_936A,
	_1_278A,
	#[default]
	_1_620A,
	_1_962A,
	_2_304A,
	_2_646A,
}

#[bitsize(4)]
#[derive(FromBits)]
pub enum PwrLim {
	_2W   = 0x0,
	None  = 0x5,
	_3W   = 0xA,
	_3_5W = 0xF,
	#[fallback]
	Invalid,
}

#[bitsize(11)]
#[derive(FromBits, DebugBits, Clone, Copy)]
pub struct Status {
	swen:         bool,
	srvo_ilim:    bool,
	srvo_plim:    bool,
	min_ot:       bool,
	por_occ:      bool,
	over_current: bool,
	tsd:          bool,
	vcc_uvlo:     bool,
	vddio_uvlo:   bool,
	cp_uvlo:      bool,
	vp25_uvlo:    bool,
}

impl defmt::Format for Status {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:#X}", self.value.as_u16())
    }
}  

impl Status {
	pub fn clear(&mut self) {
		self.set_swen(false);
		self.set_srvo_ilim(false);
		self.set_srvo_plim(false);
		self.set_min_ot(false);
		self.set_por_occ(false);
		self.set_over_current(false);
		self.set_tsd(false);
		self.set_vcc_uvlo(false);
		self.set_vddio_uvlo(false);
		self.set_cp_uvlo(false);
		self.set_vp25_uvlo(false);
	}
}

#[bitsize(9)]
pub struct DACILimN(u9);
#[bitsize(9)]
pub struct DACILimP(u9);

#[bitsize(32)]
pub struct DAC(i32);

impl DAC {
	// Sets the value and sign-expands to 32bit
	pub fn set(&mut self, val: i25) {
		self.set_val_0(val.value());
	}

	pub fn get(&self) -> i25 {
        i25::from_bits((self.val_0() & i25::MASK) as u32) 
	}
}
#[bitsize(4)]
pub struct OVClamp(u4);
#[bitsize(4)]
pub struct UVClamp(u4);

#[bitsize(7)]
#[derive(FromBits)]
/// See table on datasheet page 26 for options
pub struct AMux {
	amux:      u4,
	amux_test: u2,
	aout_en:   bool,
}

#[derive(Debug, defmt::Format)]
pub enum LtError<SpiError, ENError, SWENError> {
	Spi(SpiError),
	EN(ENError),
	SWEN(SWENError),
	BadChecksum,
	NAK(SpiAck),
	Misc,
}

impl<S, E, SW> LtError<S, E, SW> {
	pub fn from_spi(val: S) -> Self {
		Self::Spi(val)
	}

	pub fn from_en(val: E) -> Self {
		Self::EN(val)
	}

	pub fn from_swen(val: SW) -> Self {
		Self::SWEN(val)
	}
}

impl<S, E, SW> From<TryFromSliceError> for LtError<S, E, SW> {
	fn from(_value: TryFromSliceError) -> Self {
		Self::Misc
	}
}

pub trait Er {
	type Error;
}

impl<SPI, SYNC, EN, SWEN, D> Er for LT8722<SPI, SYNC, EN, SWEN, D>
where
	SPI: SpiDevice,
    SYNC: OptionalPin,
	EN: OutputPin,
	SWEN: OutputPin,
	D: DelayNs,
{
	type Error = LtError<SPI::Error, EN::Error, SWEN::Error>;
}

// Ramping Config. All derived from the required soft-start duration.
const DAC_STEP_DUR_US: u32 = 100;
const DAC_STEPS: i32 = 5000 / DAC_STEP_DUR_US as i32;
const DAC_RAMP: i32 = 0xFFFFFF / DAC_STEPS;
const DAC_RAMP_STEP: i25 = i25::from_i32(DAC_RAMP);

const _DAC_RAMP_VALID: () = {
	let diff = ((1 << 24) - DAC_RAMP * DAC_STEPS).abs();
	assert!(diff < 0xFF)
};

impl<SPI, SYNC, EN, SWEN, D> LT8722<SPI, SYNC, EN, SWEN, D>
where
	SPI: SpiDevice,
    SYNC: OptionalPin,
	EN: OutputPin,
	SWEN: OutputPin,
	D: DelayNs,
{
	const CRC: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_SMBUS);

	impl_reg_field_access! {(
	enable_req, bool,
	swen_req, bool,
	sw_freq_adj, PwmFreqAdj,
	sys_dc, SysDc,
	vcc_vreg, VccVreg,
	sw_vc_int, VcInt,
	spi_rst, bool,
	pwr_lim, PwrLim
	),
	Command, u22}

    impl_reg_access! { DAC_ILimN, DACILimN, u9}
    impl_reg_access! { DAC_ILimP, DACILimP, u9}

    impl_reg_field_access! {(
    amux, u4,
    amux_test, u2,
    aout_en, bool
    ),
    AMux, u7}

    impl_reg_access! { Ov_Clamp, OVClamp, u4}
    impl_reg_access! { Uv_Clamp, UVClamp, u4}

	/// Create a new driver. Requires an [SpiDevice] that owns the relevant CS.
	pub fn new(
		spi: SPI,
		_sync_pin: SYNC,
		enable: EN,
		switch_enable: SWEN,
		delay: D,
	) -> Result<Self, <Self as Er>::Error> {
		let mut driver = Self {
			spi,
			_sync_pin,
			enable,
			switch_enable,
			delay,
			transfer_buf: [0u8; 8],
			registers: Registers::default(),
		};

		driver.enable.set_low().map_err(LtError::from_en)?;
		driver.switch_enable.set_low().map_err(LtError::from_swen)?;

		Ok(driver)
	}

	/// Start the device in a manner that prevents large inrush current, as per pg12-13 on the
	/// datasheet
	pub fn soft_start(&mut self) -> Result<(), <Self as Er>::Error> {
		self.enable.set_high().map_err(LtError::from_en)?;
		write_reg!(self, Command, command.set_enable_req(true));

		let dac: i25 = i25::MIN;
		write_reg!(self, DAC, dac.set(dac));
		write_reg!(self, Status, status.clear());
		self.delay.delay_ms(1);

		// Ramp up to 0 over >= 5ms
		self.set_dac(i25::ZERO)?;

		self.switch_enable.set_high().map_err(LtError::from_swen)?;
		write_reg!(self, Command, command.set_swen_req(true));

		self.delay.delay_us(160);
		Ok(())
	}

	// Takes a voltage in Volts. Will ramp if the difference is larger than a defined threshold
	pub fn set_voltage(&mut self, target: f32) -> Result<(), <Self as Er>::Error> {
		// Eq. 8 in the datasheet:
		// Vout = 16 * SPIS_DAC * V2p5 * 2^-25
		// Rearragned, you get:
		// DAC = Vout / (2.5 * 2^(-25 + 4))
		// DAC = Vout * (2^21 / 2.5)
		const FACTOR: f32 = 2_u32.pow(21) as f32 / 2.5;
		let dac_target = i25::new((target * FACTOR) as i32);

		self.set_dac(dac_target)
	}

	// Sets the SPIS_DAC value, ramping if larger than the defined DAC_RAMP_STEP
	pub fn set_dac(&mut self, target: i25) -> Result<(), <Self as Er>::Error> {
		let mut dac_diff = self.registers.dac.get() - target;
		if dac_diff == i25::ZERO {
			return Ok(());
		};

		let dac_sign = if dac_diff.is_negative() { i25::new(-1) } else { i25::new(1) };
		while dac_diff.value() * dac_sign.value() > DAC_RAMP_STEP.value() {
            #[cfg(test)]
            {
                println!("Ramp: {:#08X} -= {1:#08X} ({1})", self.registers.dac.get(), DAC_RAMP_STEP * dac_sign);
            }
			write_reg!(
				self,
				DAC,
				dac.set(
                    self.registers.dac.get() - DAC_RAMP_STEP * dac_sign,
                )
			);
			dac_diff -= DAC_RAMP_STEP * dac_sign;
			self.delay.delay_us(DAC_STEP_DUR_US);
		}

		write_reg!(self, DAC, dac.set(target));

		Ok(())
	}

    pub fn clear_status(&mut self) -> Result<Status, <Self as Er>::Error> {
        write_reg!(self, Status, status.clear());
        Ok(self.registers.status)
    }

	/// Perform an SPI transaction with the driver. Handles Opcodes/Addressing/CRC/Ack
	fn spi_transaction(&mut self, packet: SpiPacket) -> Result<SpiResponse, <Self as Er>::Error> {
		let mut buf = self.transfer_buf;

		let xfer = match packet {
			SpiPacket::Status { addr } => {
				buf[0] = 0xF0;
				buf[1] = addr as u8;
				buf[2] = Self::CRC.checksum(&buf[0..2]);
				&mut buf[0..4]
			}
			SpiPacket::DataRead { addr } => {
				buf[0] = 0xF4;
				buf[1] = addr as u8;
				buf[2] = Self::CRC.checksum(&buf[0..2]);
				&mut buf[..]
			}
			SpiPacket::DataWrite { addr, data } => {
				buf[0] = 0xF2;
				buf[1] = addr as u8;
				buf[2..6].copy_from_slice(&data.to_be_bytes());
				buf[6] = Self::CRC.checksum(&buf[0..6]);
				&mut buf[..]
			}
		};

		self.spi
			.transfer_in_place(xfer)
			.map_err(LtError::from_spi)?;

		let (crc_match, ack, resp) = match packet {
			SpiPacket::Status { .. } => {
				let crc = Self::CRC.checksum(&buf[0..2]);
				let resp = SpiResponse {
					status: Status::from(u11::from_u16(
						((buf[0] as u16) << 8 | buf[1] as u16) & 0x07FF,
					)),
					data:   None,
				};
				(crc == buf[2], SpiAck::from(buf[3]), resp)
			}
			SpiPacket::DataRead { .. } => {
				let crc = Self::CRC.checksum(&buf[0..6]);
				let resp = SpiResponse {
					status: Status::from(u11::from_u16(
						((buf[0] as u16) << 8 | buf[1] as u16) & 0x07FF,
					)),
					data:   buf[2..6].try_into().ok().map(u32::from_be_bytes),
				};
				(crc == buf[6], SpiAck::from(buf[7]), resp)
			}
			SpiPacket::DataWrite { .. } => {
				let crc = Self::CRC.checksum(&buf[0..2]);
				let resp = SpiResponse {
					status: Status::from(u11::from_u16(
						((buf[0] as u16) << 8 | buf[1] as u16) & 0x07FF,
					)),
					data:   None,
				};
				(crc == buf[2], SpiAck::from(buf[7]), resp)
			}
		};

		if !crc_match {
			return Err(LtError::BadChecksum);
		};

		if let SpiAck::Ack = ack {
			Ok(resp)
		} else {
			Err(LtError::NAK(ack))
		}
	}

    /// Free the resources used by this perioheral
    pub fn free(self) -> (SPI, SYNC, EN, SWEN, D) {
        (self.spi, self._sync_pin, self.enable, self.switch_enable, self.delay)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use embedded_hal_mock::eh1::{spi, delay};
	const CRC: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_SMBUS);

    #[test]
    fn soft_start() {

        let mut expectations = vec![
            spi::Transaction::transaction_start(),
            spi::Transaction::transfer_in_place(
                vec![0xF2, 0x00, 0x00, 0x08, 0xA2, 0x15, 0x65, 0x00], 
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
            ),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::transfer_in_place(
                vec![0xF2, 0x04, 0xFF, 0x00, 0x00, 0x00, 0x33, 0x00], 
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
            ),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::transfer_in_place(
                vec![0xF2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00], 
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
            ),
            spi::Transaction::transaction_end(),
        ];

        let mut dac = i25::MIN;
        for _ in 0..DAC_STEPS {
            dac = dac.saturating_add(DAC_RAMP_STEP);
            let mut packet: Vec<u8> = vec![0xF2, 0x04];
            packet.append(&mut dac.value().to_be_bytes().to_vec()); 
            packet.append(&mut vec![CRC.checksum(&packet[..]), 0x00]);
            expectations.append(&mut vec![
                spi::Transaction::transaction_start(),
                spi::Transaction::transfer_in_place(
                    packet,
                    vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
                ),
                spi::Transaction::transaction_end(),
            ]); 
        }

        expectations.append(&mut vec![
            spi::Transaction::transaction_start(),
            spi::Transaction::transfer_in_place(
                vec![0xF2, 0x04, 0x00, 0x00, 0x00, 0x00, 0xE2, 0x00], 
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
            ),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::transfer_in_place(
                vec![0xF2, 0x00, 0x00, 0x08, 0xA2, 0x17, 0x6B, 0x00], 
                vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
            ),
            spi::Transaction::transaction_end(),
        ]);

        let spi_mock = spi::Mock::new(&expectations);

        let mut lt = LT8722::new(spi_mock, NoPin {}, NoPin {}, NoPin {}, delay::NoopDelay)
            .expect("Failed to construct")
        ;

        lt.soft_start().expect("Failed to soft start");

        let (mut spi_mock, ..) = lt.free();
        spi_mock.done()
    }
}
