#include <gtest/gtest.h>

#include "isobus/isobus/can_transport_protocol.hpp"
#include "isobus/utility/system_timing.hpp"

#include "helpers/control_function_helpers.hpp"
#include "helpers/messaging_helpers.hpp"

#include <cmath>
#include <future>

using namespace isobus;

// Test case for sending a broadcast message
TEST(TRANSPORT_PROTOCOL_TESTS, BroadcastMessageSending)
{
	constexpr std::uint32_t pgnToSent = 0xFEEC;
	constexpr std::array<std::uint8_t, 17> dataToSent = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11 };

	CANNetworkConfiguration defaultConfiguration;
	auto originator = test_helpers::create_mock_control_function(0x01);

	std::size_t frameCount = 0;
	std::uint32_t frameTime = 0;
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
				EXPECT_NEAR(SystemTiming::get_time_elapsed_ms(frameTime), 50, 5);
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
				EXPECT_NEAR(SystemTiming::get_time_elapsed_ms(frameTime), 50, 5);
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
				EXPECT_NEAR(SystemTiming::get_time_elapsed_ms(frameTime), 50, 5);
				break;

			default:
				EXPECT_TRUE(false);
		}

		frameCount++;
		frameTime = SystemTiming::get_timestamp_ms();
		return true;
	};

	// Create the transport protocol manager
	TransportProtocolManager manager(sendFrameCallback, nullptr, &defaultConfiguration);

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

	// We expect the transmission to take the minimum time between frames as we update continuously, plus some margin, by default that should be 50ms
	EXPECT_NEAR(SystemTiming::get_time_elapsed_ms(time), 3 * 50, 5);

	// After the transmission is finished, the session should be removed as indication that connection is closed
	ASSERT_FALSE(manager.has_session(originator, nullptr));
}

// Test case for receiving a broadcast message
TEST(TRANSPORT_PROTOCOL_TESTS, BroadcastMessageReceiving)
{
	constexpr std::uint32_t pgnToReceive = 0xFEEC;
	constexpr std::array<std::uint8_t, 17> dataToReceive = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11 };

	CANNetworkConfiguration defaultConfiguration;
	auto originator = test_helpers::create_mock_control_function(0x01);

	std::uint8_t messageCount = 0;
	auto receiveMessageCallback = [&](const CANMessage &message) {
		CANIdentifier identifier = message.get_identifier();
		EXPECT_EQ(identifier.get_parameter_group_number(), pgnToReceive);
		EXPECT_EQ(identifier.get_priority(), CANIdentifier::CANPriority::PriorityDefault6);
		EXPECT_EQ(message.get_source_control_function(), originator);
		EXPECT_TRUE(message.is_broadcast());
		EXPECT_EQ(message.get_data_length(), dataToReceive.size());
		for (std::size_t i = 0; i < dataToReceive.size(); i++)
		{
			EXPECT_EQ(message.get_data()[i], dataToReceive[i]);
		}
		messageCount++;
	};

	// Create the transport protocol manager
	TransportProtocolManager manager(nullptr, receiveMessageCallback, &defaultConfiguration);

	// Receive broadcast announcement message (BAM)
	manager.process_message(test_helpers::create_message_broadcast(
	  7,
	  0xEC00, // Transport Protocol Connection Management
	  originator,
	  {
	    0x20, // BAM Mux
	    17, // Data Length
	    0, // Data Length MSB
	    3, // Packet count
	    0xFF, // Reserved
	    0xEC, // PGN LSB
	    0xFE, // PGN middle byte
	    0x00, // PGN MSB
	  }));

	EXPECT_TRUE(manager.has_session(originator, nullptr));

	// Receive the first data frame
	manager.process_message(test_helpers::create_message_broadcast(
	  7,
	  0xEB00, // Transport Protocol Data Transfer
	  originator,
	  {
	    1, // Sequence number
	    dataToReceive[0],
	    dataToReceive[1],
	    dataToReceive[2],
	    dataToReceive[3],
	    dataToReceive[4],
	    dataToReceive[5],
	    dataToReceive[6],
	  }));

	// Receive the second data frame
	manager.process_message(test_helpers::create_message_broadcast(
	  7,
	  0xEB00, // Transport Protocol Data Transfer
	  originator,
	  {
	    2, // Sequence number
	    dataToReceive[7],
	    dataToReceive[8],
	    dataToReceive[9],
	    dataToReceive[10],
	    dataToReceive[11],
	    dataToReceive[12],
	    dataToReceive[13],
	  }));

	// Receive the third data frame
	manager.process_message(test_helpers::create_message_broadcast(
	  7,
	  0xEB00, // Transport Protocol Data Transfer
	  originator,
	  {
	    3, // Sequence number
	    dataToReceive[14],
	    dataToReceive[15],
	    dataToReceive[16],
	    0xFF,
	    0xFF,
	    0xFF,
	    0xFF,
	  }));

	// We now expect the message to be received
	ASSERT_EQ(messageCount, 1);

	// After the transmission is finished, the session should be removed as indication that connection is closed
	ASSERT_FALSE(manager.has_session(originator, nullptr));
}

// Test case for timeout when receiving broadcast message
TEST(TRANSPORT_PROTOCOL_TESTS, BroadcastMessageTimeout)
{
	CANNetworkConfiguration defaultConfiguration;
	auto originator = test_helpers::create_mock_control_function(0x01);

	std::uint8_t messageCount = 0;
	auto receiveMessageCallback = [&](const CANMessage &) {
		messageCount++;
	};

	// Create the transport protocol manager
	TransportProtocolManager manager(nullptr, receiveMessageCallback, &defaultConfiguration);

	// Receive broadcast announcement message (BAM)
	std::uint32_t sessionCreationTime = SystemTiming::get_timestamp_ms();
	manager.process_message(test_helpers::create_message_broadcast(
	  7,
	  0xEC00, // Transport Protocol Connection Management
	  originator,
	  {
	    0x20, // BAM Mux
	    17, // Data Length
	    0, // Data Length MSB
	    3, // Packet count
	    0xFF, // Reserved
	    0xEC, // PGN LSB
	    0xFE, // PGN middle byte
	    0x00, // PGN MSB
	  }));

	EXPECT_TRUE(manager.has_session(originator, nullptr));

	// We expect the session to exists for T1=750ms before timing out
	std::uint32_t sessionRemovalTime = 0;
	while (SystemTiming::get_time_elapsed_ms(sessionCreationTime) < 1000)
	{
		manager.update();
		if (!manager.has_session(originator, nullptr))
		{
			sessionRemovalTime = SystemTiming::get_timestamp_ms();
			break;
		}
	}
	EXPECT_EQ(messageCount, 0);
	EXPECT_NEAR(sessionRemovalTime - sessionCreationTime, 750, 5);
};

// Test case for multiple concurrent broadcast messages
TEST(TRANSPORT_PROTOCOL_TESTS, BroadcastConcurrentMessaging)
{
	// We setup five sources, two of them sending the same PGN and data, and the other three sending the different PGNs and data combinations
	constexpr std::uint32_t pgn1ToReceive = 0xFEEC;
	constexpr std::uint32_t pgn2ToReceive = 0xFEEB;
	constexpr std::array<std::uint8_t, 17> dataToReceive1 = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11 };
	constexpr std::array<std::uint8_t, 12> dataToReceive2 = { 0xAC, 0xAB, 0xAA, 0xA9, 0xA8, 0xA7, 0xA6, 0xA5, 0xA4, 0xA3, 0xA2, 0xA1 };

	CANNetworkConfiguration configuration;
	configuration.set_max_number_transport_protocol_sessions(5); // We need to increase the number of sessions to 5 for this test
	auto originator1 = test_helpers::create_mock_control_function(0x01);
	auto originator2 = test_helpers::create_mock_control_function(0x02);
	auto originator3 = test_helpers::create_mock_control_function(0x03);
	auto originator4 = test_helpers::create_mock_control_function(0x04);
	auto originator5 = test_helpers::create_mock_control_function(0x05);

	std::uint8_t messageCount = 0;
	auto receiveMessageCallback = [&](const CANMessage &message) {
		CANIdentifier identifier = message.get_identifier();
		ASSERT_EQ(identifier.get_priority(), CANIdentifier::CANPriority::PriorityDefault6);
		ASSERT_TRUE(message.is_broadcast());

		std::uint32_t pgnToCheck;
		const std::uint8_t *dataToCheck;
		std::size_t dataLengthToCheck;

		if ((message.get_source_control_function() == originator1) || (message.get_source_control_function() == originator2))
		{
			pgnToCheck = pgn1ToReceive;
			dataToCheck = dataToReceive1.data();
			dataLengthToCheck = dataToReceive1.size();
		}
		else if (message.get_source_control_function() == originator3)
		{
			pgnToCheck = pgn1ToReceive;
			dataToCheck = dataToReceive2.data();
			dataLengthToCheck = dataToReceive2.size();
		}
		else if (message.get_source_control_function() == originator4)
		{
			pgnToCheck = pgn2ToReceive;
			dataToCheck = dataToReceive1.data();
			dataLengthToCheck = dataToReceive1.size();
		}
		else if (message.get_source_control_function() == originator5)
		{
			pgnToCheck = pgn2ToReceive;
			dataToCheck = dataToReceive2.data();
			dataLengthToCheck = dataToReceive2.size();
		}
		else
		{
			// Unexpected source, fail the test
			ASSERT_TRUE(false);
		}

		ASSERT_EQ(identifier.get_parameter_group_number(), pgnToCheck);
		ASSERT_EQ(message.get_data_length(), dataLengthToCheck);
		for (std::size_t i = 0; i < dataLengthToCheck; i++)
		{
			ASSERT_EQ(message.get_data()[i], dataToCheck[i]);
		}
		messageCount++;
	};

	// Create the receiving transport protocol manager
	TransportProtocolManager rxManager(nullptr, receiveMessageCallback, &configuration);

	// Create the sending transport protocol manager
	auto sendFrameCallback = [&](std::uint32_t parameterGroupNumber,
	                             CANDataSpan data,
	                             std::shared_ptr<InternalControlFunction> sourceControlFunction,
	                             std::shared_ptr<ControlFunction> destinationControlFunction,
	                             CANIdentifier::CANPriority priority) {
		EXPECT_EQ(destinationControlFunction, nullptr);
		CANMessage message(0); //! TODO: hack for now, will be fixed when we remove CANNetwork Singleton
		std::uint32_t identifier = test_helpers::create_ext_can_id_broadcast(static_cast<std::uint8_t>(priority),
		                                                                     parameterGroupNumber,
		                                                                     sourceControlFunction);
		message.set_identifier(CANIdentifier(identifier));
		message.set_source_control_function(sourceControlFunction);
		message.set_data(data.begin(), data.size());
		rxManager.process_message(message);
		return true;
	};
	TransportProtocolManager txManager(sendFrameCallback, nullptr, &configuration);

	// Send the messages
	std::unique_ptr<CANMessageData> data = std::make_unique<CANMessageDataView>(dataToReceive1.data(), dataToReceive1.size());
	txManager.protocol_transmit_message(pgn1ToReceive, data, originator1, nullptr, nullptr, nullptr);
	data = std::make_unique<CANMessageDataView>(dataToReceive1.data(), dataToReceive1.size());
	txManager.protocol_transmit_message(pgn1ToReceive, data, originator2, nullptr, nullptr, nullptr);
	data = std::make_unique<CANMessageDataView>(dataToReceive2.data(), dataToReceive2.size());
	txManager.protocol_transmit_message(pgn1ToReceive, data, originator3, nullptr, nullptr, nullptr);
	data = std::make_unique<CANMessageDataView>(dataToReceive1.data(), dataToReceive1.size());
	txManager.protocol_transmit_message(pgn2ToReceive, data, originator4, nullptr, nullptr, nullptr);
	data = std::make_unique<CANMessageDataView>(dataToReceive2.data(), dataToReceive2.size());
	txManager.protocol_transmit_message(pgn2ToReceive, data, originator5, nullptr, nullptr, nullptr);

	ASSERT_TRUE(txManager.has_session(originator1, nullptr));
	ASSERT_TRUE(txManager.has_session(originator2, nullptr));
	ASSERT_TRUE(txManager.has_session(originator3, nullptr));
	ASSERT_TRUE(txManager.has_session(originator4, nullptr));
	ASSERT_TRUE(txManager.has_session(originator5, nullptr));

	// Wait for the transmissions to finish (or timeout)
	std::uint32_t time = SystemTiming::get_timestamp_ms();
	while ((messageCount < 5) && (SystemTiming::get_time_elapsed_ms(time) < 5 * 200))
	{
		txManager.update();
		rxManager.update();
	}

	ASSERT_FALSE(rxManager.has_session(originator1, nullptr));
	ASSERT_FALSE(rxManager.has_session(originator2, nullptr));
	ASSERT_FALSE(rxManager.has_session(originator3, nullptr));
	ASSERT_FALSE(rxManager.has_session(originator4, nullptr));
	ASSERT_FALSE(rxManager.has_session(originator5, nullptr));
	ASSERT_FALSE(txManager.has_session(originator1, nullptr));
	ASSERT_FALSE(txManager.has_session(originator2, nullptr));
	ASSERT_FALSE(txManager.has_session(originator3, nullptr));
	ASSERT_FALSE(txManager.has_session(originator4, nullptr));
	ASSERT_FALSE(txManager.has_session(originator5, nullptr));
	ASSERT_EQ(messageCount, 5);
}