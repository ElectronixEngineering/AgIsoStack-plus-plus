//================================================================================================
/// @file can_message.cpp
///
/// @brief An abstraction of a CAN message, could be > 8 data bytes.
/// @author Adrian Del Grosso
/// @author Daan Steenbergen
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#include "isobus/isobus/can_message.hpp"
#include <cassert>

namespace isobus
{
	CANMessage::CANMessage(std::uint8_t CANPort) :
	  CANPortIndex(CANPort)
	{
	}

	CANMessage::Type CANMessage::get_type() const
	{
		return messageType;
	}

	const std::vector<std::uint8_t> &CANMessage::get_data() const
	{
		return data;
	}

	std::size_t CANMessage::get_data_length() const
	{
		return data.size();
	}

	ControlFunction *CANMessage::get_source_control_function() const
	{
		return source;
	}

	ControlFunction *CANMessage::get_destination_control_function() const
	{
		return destination;
	}

	CANIdentifier CANMessage::get_identifier() const
	{
		return identifier;
	}

	std::uint8_t CANMessage::get_can_port_index() const
	{
		return CANPortIndex;
	}

	void CANMessage::set_data(const std::uint8_t *dataBuffer, std::size_t length)
	{
		if (nullptr != dataBuffer)
		{
			data.insert(data.end(), dataBuffer, dataBuffer + length);
		}
	}

	void CANMessage::set_data(std::uint8_t dataByte, const std::size_t insertPosition)
	{
		if (insertPosition < data.size())
		{
			data[insertPosition] = dataByte;
		}
	}

	void CANMessage::set_data_size(std::size_t length)
	{
		data.resize(length);
	}

	void CANMessage::set_source_control_function(ControlFunction *value)
	{
		source = value;
	}

	void CANMessage::set_destination_control_function(ControlFunction *value)
	{
		destination = value;
	}

	void CANMessage::set_identifier(CANIdentifier value)
	{
		identifier = value;
	}

	std::uint8_t CANMessage::get_uint8_at(const std::size_t index) const
	{
		return data.at(index);
	}

	std::uint16_t CANMessage::get_uint16_at(const std::size_t index, const ByteFormat format) const
	{
		std::uint16_t retVal;
		if (ByteFormat::LittleEndian == format)
		{
			retVal = data.at(index);
			retVal |= static_cast<std::uint16_t>(data.at(index + 1)) << 8;
		}
		else
		{
			retVal = static_cast<std::uint16_t>(data.at(index)) << 8;
			retVal |= data.at(index + 1);
		}
		return retVal;
	}

	std::uint32_t CANMessage::get_uint24_at(const std::size_t index, const ByteFormat format) const
	{
		std::uint32_t retVal;
		if (ByteFormat::LittleEndian == format)
		{
			retVal = data.at(index);
			retVal |= static_cast<std::uint32_t>(data.at(index + 1)) << 8;
			retVal |= static_cast<std::uint32_t>(data.at(index + 2)) << 16;
		}
		else
		{
			retVal = static_cast<std::uint32_t>(data.at(index + 2)) << 16;
			retVal |= static_cast<std::uint32_t>(data.at(index + 1)) << 8;
			retVal |= data.at(index + 2);
		}
		return retVal;
	}

	std::uint32_t CANMessage::get_uint32_at(const std::size_t index, const ByteFormat format) const
	{
		std::uint32_t retVal;
		if (ByteFormat::LittleEndian == format)
		{
			retVal = data.at(index);
			retVal |= static_cast<std::uint32_t>(data.at(index + 1)) << 8;
			retVal |= static_cast<std::uint32_t>(data.at(index + 2)) << 16;
			retVal |= static_cast<std::uint32_t>(data.at(index + 3)) << 24;
		}
		else
		{
			retVal = static_cast<std::uint32_t>(data.at(index)) << 24;
			retVal |= static_cast<std::uint32_t>(data.at(index + 1)) << 16;
			retVal |= static_cast<std::uint32_t>(data.at(index + 2)) << 8;
			retVal |= data.at(index + 3);
		}
		return retVal;
	}

	std::uint64_t CANMessage::get_uint64_at(const std::size_t index, const ByteFormat format) const
	{
		std::uint64_t retVal;
		if (ByteFormat::LittleEndian == format)
		{
			retVal = data.at(index);
			retVal |= static_cast<std::uint64_t>(data.at(index + 1)) << 8;
			retVal |= static_cast<std::uint64_t>(data.at(index + 2)) << 16;
			retVal |= static_cast<std::uint64_t>(data.at(index + 3)) << 24;
			retVal |= static_cast<std::uint64_t>(data.at(index + 4)) << 32;
			retVal |= static_cast<std::uint64_t>(data.at(index + 5)) << 40;
			retVal |= static_cast<std::uint64_t>(data.at(index + 6)) << 48;
			retVal |= static_cast<std::uint64_t>(data.at(index + 7)) << 56;
		}
		else
		{
			retVal = static_cast<std::uint64_t>(data.at(index)) << 56;
			retVal |= static_cast<std::uint64_t>(data.at(index + 1)) << 48;
			retVal |= static_cast<std::uint64_t>(data.at(index + 2)) << 40;
			retVal |= static_cast<std::uint64_t>(data.at(index + 3)) << 32;
			retVal |= static_cast<std::uint64_t>(data.at(index + 4)) << 24;
			retVal |= static_cast<std::uint64_t>(data.at(index + 5)) << 16;
			retVal |= static_cast<std::uint64_t>(data.at(index + 6)) << 8;
			retVal |= data.at(index + 7);
		}
		return retVal;
	}
	bool isobus::CANMessage::get_bool_at(const std::size_t byteIndex, const std::uint8_t bitIndex, const std::uint8_t length) const
	{
		assert(length <= 8 - bitIndex && "length must be less than or equal to 8 - bitIndex");
		std::uint8_t mask = ((1 << length) - 1) << bitIndex;
		return (get_uint8_at(byteIndex) & mask) == mask;
	}

} // namespace isobus
