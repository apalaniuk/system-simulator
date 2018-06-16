#pragma once

namespace sim {
	namespace message {

		///
		/// SIM message types
		///

		enum class message_type : uint32_t
		{
			MT_INVALID = 0,
			MT_SIM_ENTITY_INFO,
			MT_INVALID_OUT_OF_RANGE
		};

	}
}
