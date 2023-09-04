
use embedded_hal_async::delay::DelayUs;
use lorawan_device::{
    async_device::radio::PhyRxTx,
    Timings,
    radio::{
        Bandwidth, CodingRate, RfConfig, RxQuality, SpreadingFactor, TxConfig,
    }
};

use crate::mod_params::{BoardType, ChipType, RadioError};
use crate::mod_traits::RadioKind;
use crate::LoRa;

pub(crate) const DEFAULT_DBM: i8 = 14;

/// Convert the spreading factor for use in the external lora-phy crate
impl From<SpreadingFactor> for crate::mod_params::SpreadingFactor {
    fn from(sf: SpreadingFactor) -> Self {
        match sf {
            SpreadingFactor::_7 => crate::mod_params::SpreadingFactor::_7,
            SpreadingFactor::_8 => crate::mod_params::SpreadingFactor::_8,
            SpreadingFactor::_9 => crate::mod_params::SpreadingFactor::_9,
            SpreadingFactor::_10 => crate::mod_params::SpreadingFactor::_10,
            SpreadingFactor::_11 => crate::mod_params::SpreadingFactor::_11,
            SpreadingFactor::_12 => crate::mod_params::SpreadingFactor::_12,
        }
    }
}

/// Convert the bandwidth for use in the external lora-phy crate
impl From<Bandwidth> for crate::mod_params::Bandwidth {
    fn from(bw: Bandwidth) -> Self {
        match bw {
            Bandwidth::_125KHz => crate::mod_params::Bandwidth::_125KHz,
            Bandwidth::_250KHz => crate::mod_params::Bandwidth::_250KHz,
            Bandwidth::_500KHz => crate::mod_params::Bandwidth::_500KHz,
        }
    }
}

/// Convert the coding rate for use in the external lora-phy crate
impl From<CodingRate> for crate::mod_params::CodingRate {
    fn from(cr: CodingRate) -> Self {
        match cr {
            CodingRate::_4_5 => crate::mod_params::CodingRate::_4_5,
            CodingRate::_4_6 => crate::mod_params::CodingRate::_4_6,
            CodingRate::_4_7 => crate::mod_params::CodingRate::_4_7,
            CodingRate::_4_8 => crate::mod_params::CodingRate::_4_8,
        }
    }
}


/// Provide the timing values for boards supported by the external lora-phy crate
impl<RK, DLY> Timings for LoRa<RK, DLY>
    where
        RK: RadioKind + 'static,
        DLY: DelayUs,
{
    fn get_rx_window_offset_ms(&self) -> i32 {
        match self.get_board_type() {
            BoardType::Rak4631Sx1262 => -15,
            BoardType::Stm32l0Sx1276 => -15,
            BoardType::Stm32wlSx1262 => -50,
            _ => -50,
        }
    }
    fn get_rx_window_duration_ms(&self) -> u32 {
        match self.get_board_type() {
            BoardType::Rak4631Sx1262 => 1050,
            BoardType::Stm32l0Sx1276 => 1003,
            BoardType::Stm32wlSx1262 => 1050,
            _ => 1050,
        }
    }
}

/// Provide the LoRa physical layer rx/tx interface for boards supported by the external lora-phy
/// crate
impl<RK, DLY> PhyRxTx for LoRa<RK, DLY>
    where
        RK: RadioKind + 'static,
        DLY: DelayUs,
{
    type PhyError = RadioError;

    async fn tx(&mut self, config: TxConfig, buffer: &[u8]) -> Result<u32, Self::PhyError> {
        let mdltn_params = self.create_modulation_params(
            config.rf.spreading_factor.into(),
            config.rf.bandwidth.into(),
            config.rf.coding_rate.into(),
            config.rf.frequency,
        )?;
        let mut tx_pkt_params =
            self.create_tx_packet_params(8, false, true, false, &mdltn_params)?;
        let pw = match self.get_board_type().into() {
            ChipType::Sx1276 | ChipType::Sx1277 | ChipType::Sx1278 | ChipType::Sx1279 => {
                if config.pw > DEFAULT_DBM {
                    DEFAULT_DBM
                } else {
                    config.pw
                }
            }
            _ => config.pw,
        };
        self.prepare_for_tx(&mdltn_params, pw.into(), false).await?;
        self.tx(&mdltn_params, &mut tx_pkt_params, buffer, 0xffffff).await?;
        Ok(0)
    }

    async fn rx(
        &mut self,
        config: RfConfig,
        receiving_buffer: &mut [u8],
    ) -> Result<(usize, RxQuality), Self::PhyError> {
        let mdltn_params = self.create_modulation_params(
            config.spreading_factor.into(),
            config.bandwidth.into(),
            config.coding_rate.into(),
            config.frequency,
        )?;
        let rx_pkt_params = self.create_rx_packet_params(
            8,
            false,
            receiving_buffer.len() as u8,
            true,
            true,
            &mdltn_params,
        )?;
        self.prepare_for_rx(
                &mdltn_params,
                &rx_pkt_params,
                None,
                None,
                false,
            )
            .await?;
        match self.rx(&rx_pkt_params, receiving_buffer).await {
            Ok((received_len, rx_pkt_status)) => {
                Ok((
                    received_len as usize,
                    RxQuality::new(rx_pkt_status.rssi, rx_pkt_status.snr as i8), // downcast snr
                ))
            }
            Err(err) => Err(err),
        }
    }
}
