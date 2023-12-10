#include <gtest/gtest.h>

#include "helpers/control_function_helpers.hpp"
#include "isobus/isobus/can_transport_protocol.hpp"
#include "isobus/utility/system_timing.hpp"

#include <cmath>
#include <future>

using namespace isobus;

// Test case for sending a broadcast message
TEST(TransportProtocolManagerTest, BroadcastMessageSending)
{
	constexpr std::uint32_t pgnToSent = 0xFEEC;
	constexpr std::array<std::uint8_t, 17> dataToSent = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11 };

	CANNetworkConfiguration configuration;
	auto originator = test_helpers::create_mock_control_function(0x01);
	std::size_t frameCount = 0;

	auto sendFrameCallback = [&](std::uint32_t parameterGroupNumber,
	                             CANDataSpan data,
	                             std::shared_ptr<InternalControlFunction> sourceControlFunction,
	                             std::shared_ptr<ControlFunction> destinationControlFunction,
	                             CANIdentifier::CANPriority priority) {
		EXPECT_EQ(data.size(), 8);
		EXPECT_EQ(sourceControlFunction, originator);
		EXPECT_EQ(destinationControlFunction, nullptr);
		EXPECT_EQ(priority, CANIdentifier::CANPriority::PriorityLowest7);

		switch (frameCount)
		{
			case 0:
				// First we expect broadcast announcement message (BAM)
				EXPECT_EQ(parameterGroupNumber, 0xEC00);
				EXPECT_EQ(data[0], 32); // BAM control byte
				EXPECT_EQ(data[1], dataToSent.size() & 0xFF);
				EXPECT_EQ(data[2], dataToSent.size() >> 8);
				EXPECT_EQ(data[3], 3); // Number of packets
				EXPECT_EQ(data[4], 0xFF); // Reserved
				EXPECT_EQ(data[5], pgnToSent & 0xFF);
				EXPECT_EQ(data[6], (pgnToSent >> 8) & 0xFF);
				EXPECT_EQ(data[7], (pgnToSent >> 16) & 0xFF);
				break;

			case 1:
				// Then we expect the first data frame
				EXPECT_EQ(parameterGroupNumber, 0xEB00);
				EXPECT_EQ(data[0], 1); // Sequence number
				EXPECT_EQ(data[1], dataToSent[0]);
				EXPECT_EQ(data[2], dataToSent[1]);
				EXPECT_EQ(data[3], dataToSent[2]);
				EXPECT_EQ(data[4], dataToSent[3]);
				EXPECT_EQ(data[5], dataToSent[4]);
				EXPECT_EQ(data[6], dataToSent[5]);
				EXPECT_EQ(data[7], dataToSent[6]);
				break;

			case 2:
				// Then we expect the second data frame
				EXPECT_EQ(parameterGroupNumber, 0xEB00);
				EXPECT_EQ(data[0], 2); // Sequence number
				EXPECT_EQ(data[1], dataToSent[7]);
				EXPECT_EQ(data[2], dataToSent[8]);
				EXPECT_EQ(data[3], dataToSent[9]);
				EXPECT_EQ(data[4], dataToSent[10]);
				EXPECT_EQ(data[5], dataToSent[11]);
				EXPECT_EQ(data[6], dataToSent[12]);
				EXPECT_EQ(data[7], dataToSent[13]);
				break;

			case 3:
				// Then we expect the third data frame
				EXPECT_EQ(parameterGroupNumber, 0xEB00);
				EXPECT_EQ(data[0], 3); // Sequence number
				EXPECT_EQ(data[1], dataToSent[14]);
				EXPECT_EQ(data[2], dataToSent[15]);
				EXPECT_EQ(data[3], dataToSent[16]);
				EXPECT_EQ(data[4], 0xFF);
				EXPECT_EQ(data[5], 0xFF);
				EXPECT_EQ(data[6], 0xFF);
				EXPECT_EQ(data[7], 0xFF);
				break;

			default:
				EXPECT_TRUE(false);
		}

		frameCount++;
		return true;
	};

	// Create the transport protocol manager
	TransportProtocolManager manager(sendFrameCallback, nullptr, &configuration);

	// Send the message
	std::unique_ptr<CANMessageData> data = std::make_unique<CANMessageDataView>(dataToSent.data(), dataToSent.size());
	ASSERT_TRUE(manager.protocol_transmit_message(pgnToSent, data, originator, nullptr, nullptr, nullptr));
	ASSERT_TRUE(manager.has_session(originator, nullptr));
	// We shouldn't be able to broadcast another message
	ASSERT_FALSE(manager.protocol_transmit_message(pgnToSent, data, originator, nullptr, nullptr, nullptr));
	// Also not a message with a different PGN
	ASSERT_FALSE(manager.protocol_transmit_message(pgnToSent + 1, data, originator, nullptr, nullptr, nullptr));

	// Wait for the transmission to finish (or timeout)
	std::uint32_t time = SystemTiming::get_timestamp_ms();
	while ((frameCount < 4) && (SystemTiming::get_time_elapsed_ms(time) < 3 * 200))
	{
		manager.update();
	}
	ASSERT_EQ(frameCount, 4);

	// We expect the transmission to take the minimum time between frames as we update continuously, plus some margin.
	EXPECT_NEAR(SystemTiming::get_time_elapsed_ms(time), 3 * configuration.get_minimum_time_between_transport_protocol_bam_frames(), 5);

	// After the transmission is finished, the session should be removed as indication that connection is closed
	ASSERT_FALSE(manager.has_session(originator, nullptr));
}
