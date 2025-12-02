#![cfg_attr(not(test), no_std)]

use core::{array::TryFromSliceError, convert::Infallible};

use bilge::prelude::*;
use embedded_hal::{
	digital::{ErrorType, OutputPin},
	spi::SpiDevice,
};
use fugit::ExtU64 as _;
use paste::paste;
use rtic_time::Monotonic;

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
	D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
{
	spi:           SPI,
	_sync_pin:     SYNC,
	enable:        EN,
	switch_enable: SWEN,
	_delay:         D,
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
	pub enable_req:  bool,
	pub swen_req:    bool,
	pub sw_freq_set: PwmFreqCtrl,
	pub sw_freq_adj: PwmFreqAdj,
	pub sys_dc:      SysDc,
	pub vcc_vreg:    VccVreg,
	pub reserved:    u1, // MUST BE 0
	pub sw_vc_int:   VcInt,
	pub spi_rst:     bool,
	pub pwr_lim:     PwrLim,
	pub reserved:    u3,
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
	pub swen:         bool,
	pub srvo_ilim:    bool,
	pub srvo_plim:    bool,
	pub min_ot:       bool,
	pub por_occ:      bool,
	pub over_current: bool,
	pub tsd:          bool,
	pub vcc_uvlo:     bool,
	pub vddio_uvlo:   bool,
	pub cp_uvlo:      bool,
	pub vp25_uvlo:    bool,
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
#[derive(FromBits)]
pub struct DACILimN(pub u9);
#[bitsize(9)]
#[derive(FromBits)]
pub struct DACILimP(pub u9);

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
#[derive(FromBits)]
pub struct OVClamp(pub u4);
#[bitsize(4)]
#[derive(FromBits)]
pub struct UVClamp(pub u4);

#[bitsize(7)]
#[derive(FromBits)]
/// See table on datasheet page 26 for options
pub struct AMux {
	pub amux:      u4,
	pub amux_test: u2,
	pub aout_en:   bool,
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
	D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
{
	type Error = LtError<SPI::Error, EN::Error, SWEN::Error>;
}

// Ramping Config. All derived from the required soft-start duration.
const DAC_STEP_DUR_US: u64 = 100;
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
	D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
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
			_delay: delay,
			transfer_buf: [0u8; 8],
			registers: Registers::default(),
		};

		driver.enable.set_low().map_err(LtError::from_en)?;
		driver.switch_enable.set_low().map_err(LtError::from_swen)?;

		Ok(driver)
	}

	/// Start the device in a manner that prevents large inrush current, ~~as per pg12-13 on the
	/// datasheet~~ as per the linudino example
	pub async fn soft_start(&mut self) -> Result<(), <Self as Er>::Error> {
		self.enable.set_high().map_err(LtError::from_en)?;

		self.registers.command = Command::from(u22::new(0x00004000));
		write_reg!(self, Command, command);
		D::delay(100_u64.micros()).await;
		self.registers.command = Command::from(u22::new(0x0000120D));
		write_reg!(self, Command, command);
		D::delay(100_u64.micros()).await;

		self.registers.status = Status::from(u11::ZERO);
		write_reg!(self, Status, status);
		D::delay(100_u64.micros()).await;
		write_reg!(self, Command, command);
		D::delay(100_u64.micros()).await;

		let dac: i25 = i25::MIN;
		write_reg!(self, DAC, dac.set(dac));
		D::delay(100_u64.micros()).await;

		// Ramp up to 0 over >= 5ms
		self.set_dac(i25::ZERO).await?;

		self.registers.status = Status::from(u11::ZERO);
		write_reg!(self, Status, status);
		D::delay(100_u64.micros()).await;

		self.switch_enable.set_high().map_err(LtError::from_swen)?;
		write_reg!(self, Command, command.set_swen_req(true));

		D::delay(160_u64.micros()).await;
		Ok(())
	}

	// Takes a voltage in Volts. Will ramp if the difference is larger than a defined threshold
	pub async fn set_voltage(&mut self, target: f32) -> Result<(), <Self as Er>::Error> {
		// Eq. 8 in the datasheet:
		// Vout = 16 * SPIS_DAC * V2p5 * 2^-25
		// Rearragned, you get:
		// DAC = Vout / (2.5 * 2^(-25 + 4))
		// DAC = Vout * (2^21 / 2.5)
		const FACTOR: f32 = 2_u32.pow(21) as f32 / 2.5;
		let dac_target = i25::new((target * FACTOR) as i32);

		self.set_dac(dac_target).await
	}

	/// Sets the positive and negative current limits. Clamped to the ranges specified by the
	/// datasheet: ilimp 0-462 and ilimn 48-511.
	/// Returns the clamped current limits
	pub fn set_current_limit(
		&mut self,
		low: f32,
		high: f32,
	) -> Result<(f32, f32), <Self as Er>::Error> {
		let ilimp = u9::new(((6.8 - high) / 0.01328).clamp(0.0, 462.0) as u16);
		let ilimn = u9::new((low / -0.01328).clamp(48.0, 511.0) as u16);
		self.set_dac_ilimp(DACILimP::new(ilimp))?;
		self.set_dac_ilimn(DACILimN::new(ilimn))?;
		let ilimp = 6.8 - 0.01328 * ilimp.value() as f32;
		let ilimn = ilimn.value() as f32 * -0.01328;
		Ok((ilimn, ilimp))
	}

	// Sets the SPIS_DAC value, ramping if larger than the defined DAC_RAMP_STEP
	pub async fn set_dac(&mut self, target: i25) -> Result<(), <Self as Er>::Error> {
		let mut dac_diff = self.registers.dac.get() - target;
		if dac_diff == i25::ZERO {
			return Ok(());
		};

		let dac_sign = if dac_diff.is_negative() {
			i25::new(-1)
		} else {
			i25::new(1)
		};
		while dac_diff.value() * dac_sign.value() > DAC_RAMP_STEP.value() {
			#[cfg(test)]
			{
				println!(
					"Ramp: {:#08X} -= {1:#08X} ({1})",
					self.registers.dac.get(),
					DAC_RAMP_STEP * dac_sign
				);
			}
			write_reg!(
				self,
				DAC,
				dac.set(self.registers.dac.get() - DAC_RAMP_STEP * dac_sign,)
			);
			dac_diff -= DAC_RAMP_STEP * dac_sign;
			D::delay(DAC_STEP_DUR_US.micros()).await;
		}

		write_reg!(self, DAC, dac.set(target));

		Ok(())
	}

	pub fn get_status(&mut self) -> Result<Status, <Self as Er>::Error> {
		self.spi_transaction(SpiPacket::Status {
			addr: RegAddr::Status,
		})
		.map(|r| r.status)
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
				buf[1] = (addr as u8) << 1 & 0xFE;
				buf[2] = Self::CRC.checksum(&buf[0..2]);
				&mut buf[0..4]
			}
			SpiPacket::DataRead { addr } => {
				buf[0] = 0xF4;
				buf[1] = (addr as u8) << 1 & 0xFE;
				buf[2] = Self::CRC.checksum(&buf[0..2]);
				&mut buf[..]
			}
			SpiPacket::DataWrite { addr, data } => {
				buf[0] = 0xF2;
				buf[1] = (addr as u8) << 1 & 0xFE;
				buf[2..6].copy_from_slice(&data.to_be_bytes());
				buf[6] = Self::CRC.checksum(&buf[0..6]);
				&mut buf[..]
			}
		};

		#[cfg(debug_assertions)]
		defmt::debug!("{:#04X}", xfer);
		self.spi
			.transfer_in_place(xfer)
			.map_err(LtError::from_spi)?;
		#[cfg(debug_assertions)]
		defmt::debug!("{:#04X}", xfer);

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
	pub fn free(self) -> (SPI, SYNC, EN, SWEN) {
		(
			self.spi,
			self._sync_pin,
			self.enable,
			self.switch_enable,
		)
	}

	pub fn view_regs(&self) -> &Registers {
		&self.registers
	}

	/// WARNING: Does not keep internal state coherent, use macro instead
	pub fn write_register(
		&mut self,
		addr: RegAddr,
		data: u32,
	) -> Result<SpiResponse, <Self as Er>::Error> {
		self.spi_transaction(SpiPacket::DataWrite { addr, data })
	}

	pub fn read_register(&mut self, addr: RegAddr) -> Result<SpiResponse, <Self as Er>::Error> {
		self.spi_transaction(SpiPacket::DataRead { addr })
	}
}

// #[cfg(test)]
// mod test {
// 	use embedded_hal_mock::eh1::{delay, spi};
// 
// 	use super::*;
//     use std::sync::LazyLock;
// 
// 	const CRC: crc::Crc<u8> = crc::Crc::<u8>::new(&crc::CRC_8_SMBUS);
// 
//     static BASE: std::sync::LazyLock<std::time::Instant> = LazyLock::new(|| std::time::Instant::now());
//     struct MockDelay;
// 
//     impl Monotonic for MockDelay {
//         type Instant = fugit::Instant<u64, 1, 32768>;
//         type Duration = fugit::Duration<u64, 1, 32768>;
// 
//         fn now() -> Self::Instant {
//             fugit::Instant::<u64, 1, 32768>::from_ticks(0) + ((std::time::Instant::now() - *BASE).as_micros() as u64).micros()           
//         }
// 
//         async fn delay(duration: Self::Duration) {
//             todo!()
//         }
// 
//         async fn delay_until(instant: Self::Instant) {
//             todo!()
//         }
// 
//         async fn timeout_at<F: core::future::Future>(
//             instant: Self::Instant,
//             future: F,
//         ) -> Result<F::Output, rtic_time::TimeoutError> {
//             todo!()
//         }
// 
//         async fn timeout_after<F: core::future::Future>(
//             duration: Self::Duration,
//             future: F,
//         ) -> Result<F::Output, rtic_time::TimeoutError> {
//             todo!()
//         }
//     }
// 
// 	#[test]
// 	fn soft_start() {
// 		let mut expectations = vec![
// 			spi::Transaction::transaction_start(),
// 			spi::Transaction::transfer_in_place(
// 				vec![0xF2, 0x00, 0x00, 0x08, 0xA2, 0x15, 0x65, 0x00],
// 				vec![0x04, 0x90, 0xAD, 0x00, 0x00, 0x00, 0x00, 0xA5],
// 			),
// 			spi::Transaction::transaction_end(),
// 			spi::Transaction::transaction_start(),
// 			spi::Transaction::transfer_in_place(
// 				vec![0xF2, 0x04, 0xFF, 0x00, 0x00, 0x00, 0x33, 0x00],
// 				vec![0x04, 0x90, 0xAD, 0x00, 0x00, 0x00, 0x00, 0xA5],
// 			),
// 			spi::Transaction::transaction_end(),
// 			spi::Transaction::transaction_start(),
// 			spi::Transaction::transfer_in_place(
// 				vec![0xF2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00],
// 				vec![0x06, 0x90, 0x87, 0x00, 0x00, 0x00, 0x00, 0xA5],
// 			),
// 			spi::Transaction::transaction_end(),
// 		];
// 
// 		let mut dac = i25::MIN;
// 		for _ in 0..DAC_STEPS {
// 			dac = dac.saturating_add(DAC_RAMP_STEP);
// 			let mut packet: Vec<u8> = vec![0xF2, 0x04];
// 			packet.append(&mut dac.value().to_be_bytes().to_vec());
// 			packet.append(&mut vec![CRC.checksum(&packet[..]), 0x00]);
// 			expectations.append(&mut vec![
// 				spi::Transaction::transaction_start(),
// 				spi::Transaction::transfer_in_place(packet, vec![
// 					0x06, 0x90, 0x87, 0x00, 0x00, 0x00, 0x00, 0xA5,
// 				]),
// 				spi::Transaction::transaction_end(),
// 			]);
// 		}
// 
// 		expectations.append(&mut vec![
// 			spi::Transaction::transaction_start(),
// 			spi::Transaction::transfer_in_place(
// 				vec![0xF2, 0x04, 0x00, 0x00, 0x00, 0x00, 0xE2, 0x00],
// 				vec![0x06, 0x90, 0x87, 0x00, 0x00, 0x00, 0x00, 0xA5],
// 			),
// 			spi::Transaction::transaction_end(),
// 			spi::Transaction::transaction_start(),
// 			spi::Transaction::transfer_in_place(
// 				vec![0xF2, 0x00, 0x00, 0x08, 0xA2, 0x17, 0x6B, 0x00],
// 				vec![0x06, 0x90, 0x87, 0x00, 0x00, 0x00, 0x00, 0xA5],
// 			),
// 			spi::Transaction::transaction_end(),
// 		]);
// 
// 		let spi_mock = spi::Mock::new(&expectations);
// 
// 		let mut lt = LT8722::new(spi_mock, NoPin {}, NoPin {}, NoPin {}, MockDelay)
// 			.expect("Failed to construct");
// 
// 		lt.soft_start().await.expect("Failed to soft start");
// 
// 		let (mut spi_mock, ..) = lt.free();
// 		spi_mock.done()
// 	}
// }
