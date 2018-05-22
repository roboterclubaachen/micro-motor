// coding: utf-8
/*
 * Copyright (c) 2018, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_DRV832X_SPI_HPP
	#error	"Don't include this file directly, use 'drv832x_spi.hpp' instead!"
#endif
#include "drv832x_spi.hpp"

// ----------------------------------------------------------------------------
template < class SpiMaster, class Cs >
modm::Drv832xSpi<SpiMaster, Cs>::Drv832xSpi()
{
	this->attachConfigurationHandler([]() {
		SpiMaster::setDataMode(SpiMaster::DataMode::Mode1);
		SpiMaster::setDataOrder(SpiMaster::DataOrder::MsbFirst);
	});
	Cs::setOutput(modm::Gpio::High);
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::FaultStatus1_t>
modm::Drv832xSpi<SpiMaster, Cs>::readFaultStatus1()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::FaultStatus1));
	RF_END_RETURN(modm::drv832xSpi::FaultStatus1_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::VgsStatus2_t>
modm::Drv832xSpi<SpiMaster, Cs>::readVgsStatus2()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::VgsStatus2));
	RF_END_RETURN(modm::drv832xSpi::VgsStatus2_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::DriverControl_t>
modm::Drv832xSpi<SpiMaster, Cs>::readDriverControl()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::DriverControl));
	RF_END_RETURN(modm::drv832xSpi::DriverControl_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::GateDriveHS_t>
modm::Drv832xSpi<SpiMaster, Cs>::readGateDriveHS()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::GateDriveHS));
	RF_END_RETURN(modm::drv832xSpi::GateDriveHS_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::GateDriveLS_t>
modm::Drv832xSpi<SpiMaster, Cs>::readGateDriveLS()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::GateDriveLS));
	RF_END_RETURN(modm::drv832xSpi::GateDriveLS_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::OcpControl_t>
modm::Drv832xSpi<SpiMaster, Cs>::readOcpControl()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::OcpControl));
	RF_END_RETURN(modm::drv832xSpi::OcpControl_t(value));
}

template < class SpiMaster, class Cs >
modm::ResumableResult<modm::drv832xSpi::CsaControl_t>
modm::Drv832xSpi<SpiMaster, Cs>::readCsaControl()
{
	RF_BEGIN();
	value = RF_CALL(readData(Register::CsaControl));
	RF_END_RETURN(modm::drv832xSpi::CsaControl_t(value));
}


/*
template < class SpiMaster, class Cs >
modm::ResumableResult<void>
modm::Drv832xSpi<SpiMaster, Cs>::setDriverControl(modm::drv832xSpi::DriverControl_t data)
{

}
*/

template < class SpiMaster, class Cs >
modm::ResumableResult<void>
modm::Drv832xSpi<SpiMaster, Cs>::writeData(Register address, uint16_t data)
{
	RF_BEGIN();

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	inBuffer[0] = 0;
	inBuffer[1] = 0;

	static constexpr uint8_t writeBit = (0 << 7); // 0 = write

	outBuffer[0] = writeBit | (static_cast<uint8_t>(address) << 3) | ((data >> 8) & 0b111);
	outBuffer[1] = data & 0xff;

	RF_CALL(SpiMaster::transfer(outBuffer, inBuffer, 2));

	if (this->releaseMaster()) {
		Cs::set();
	}

	RF_END();
}

template < class SpiMaster, class Cs >
modm::ResumableResult<uint16_t>
modm::Drv832xSpi<SpiMaster, Cs>::readData(Register address)
{
	RF_BEGIN();

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	inBuffer[0] = 0x55;
	inBuffer[1] = 0x55;

	static constexpr uint8_t writeBit = (1 << 7); // 1 = read

	outBuffer[0] = writeBit | (static_cast<uint8_t>(address) << 3);
	outBuffer[1] = 0;

	RF_CALL(SpiMaster::transfer(outBuffer, inBuffer, 2));

	if (this->releaseMaster()) {
		Cs::set();
	}

	ret = static_cast<uint16_t>(inBuffer[1]) | (static_cast<uint16_t>(inBuffer[0] & 0b111) << 8);
	RF_END_RETURN(ret);
}
