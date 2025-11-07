/// Writes a value to a register via SPI transaction and updates the status register.
///
/// This macro sets a register field value, performs an SPI write transaction,
/// and updates `self.registers.status` with the result.
///
/// # Arguments
/// * `$addr` - The `RegAddr` enum variant for the register address
/// * `$reg` - The register field name in `self.registers`
/// * `$method` - The setter method to call on the register
/// * `$args` - Arguments to pass to the setter method
///
/// # Example
/// write_reg!(Command, command.set_enable_req(true));
/// write_reg!(DAC, dac.set_val_0(0xFF000000));
/// 
#[macro_export]
macro_rules! write_reg {
    ($self:expr, $addr:ident, $reg:ident . $method:ident ( $($args:expr),* $(,)? )) => {{
        $self.registers.$reg.$method($($args),*);
        $self.registers.status = $self.spi_transaction(SpiPacket::DataWrite {
            addr: RegAddr::$addr,
            data: $self.registers.$reg.value.into(),
        })?.status;
    }};
}

#[macro_export]
macro_rules! impl_reg_field_access {
    {$name: ident, $regtype: ty, $addr: ident, $bits: ty} => {
        paste! {
            pub fn [<set_$name>](&mut self, $name: $regtype) -> Result<(), <Self as Er>::Error> {
                write_reg!(self, $addr, [<$addr:lower>].[<set_$name>]($name));
                Ok(())
            }

            pub fn [<get_$name>](&mut self) -> Result<$regtype, <Self as Er>::Error> {
                let read = self.spi_transaction(SpiPacket::DataRead {
                    addr: RegAddr::$addr,
                })?;
                self.registers.status = read.status;
                let _ = core::mem::replace(&mut self.registers.[<$addr:lower>], $addr::from($bits::from_u32(read.data.ok_or(LtError::Misc)?)));
                Ok(self.registers.[<$addr:lower>].$name())
            }
        }
    };
    {($($name: ident, $regtype: ty),+), $addr: ident, $bits: ty} => {
        $(impl_reg_field_access!{$name, $regtype, $addr, $bits})+
    }
}

#[macro_export]
macro_rules! impl_reg_access {
    {$addr: ident, $regtype: ty, $bits: ty} => {
        paste! {
            pub fn [<set_$addr:lower>](&mut self, [<$addr:lower>]: $regtype) -> Result<(), <Self as Er>::Error> {
                write_reg!(self, $addr, [<$addr:lower>].set_val_0([<$addr:lower>].val_0()));
                Ok(())
            }

            pub fn [<get_$addr:lower>](&mut self) -> Result<$regtype, <Self as Er>::Error> {
                let read = self.spi_transaction(SpiPacket::DataRead {
                    addr: RegAddr::$addr,
                })?;
                self.registers.status = read.status;
                self.registers.[<$addr:lower>].set_val_0($bits::from_u32(read.data.ok_or(LtError::Misc)?));
                Ok($regtype { value: self.registers.[<$addr:lower>].val_0() })
            }
        }
    };
}
