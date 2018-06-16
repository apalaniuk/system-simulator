#include "stdafx.h"

#include "message_handler.h++"

#include <google/protobuf/message.h>

namespace sim {
	namespace networking {
		message_handler::message_handler() { }

		message_handler::~message_handler() { }

		void message_handler::process_message(sim::message::message_type msg_type, std::vector<uint8_t> packet_data) {
			std::cout << "process_message called: " << std::string(packet_data.begin(), packet_data.end()) << std::endl;
		}
	}
}
