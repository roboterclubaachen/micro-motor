#ifndef IDENTITY_PROTOCOL_HPP
#error "Do not include this file directly, use identity_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template<typename ObjectDictionary, const MotorState& state>
constexpr void
IdentityProtocol::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary>& map)
{
	map.template setReadHandler<IdentityObjects::DeviceType>(+[]() { return (uint32_t)412; });
	map.template setReadHandler<IdentityObjects::ErrorRegister>(+[]() { return (uint8_t)0; });
	map.template setReadHandler<IdentityObjects::IdentityObject>(+[]() { return (uint8_t)4; });
	map.template setReadHandler<IdentityObjects::VendorId>(+[]() { return (uint32_t)0xdeadbeef; });
	map.template setReadHandler<IdentityObjects::ProductCode>(+[]() { return (uint32_t)1; });
	map.template setReadHandler<IdentityObjects::RevisionId>(+[]() { return (uint32_t)1; });
	map.template setReadHandler<IdentityObjects::SerialNumber>(+[]() { return (uint32_t)1; });
}