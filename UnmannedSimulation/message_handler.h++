#pragma once

#include "message_types.h++"

namespace sim {
	namespace networking {
		class message_handler {
			public:
				message_handler();
				virtual ~message_handler();

				void process_message(sim::message::message_type msg_type, std::vector<uint8_t> packet_data);
		};
	}
}