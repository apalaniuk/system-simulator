#pragma once

#include "stdafx.h"

#include <boost/asio.hpp>

#include "connection.h++"

namespace sim {
	namespace networking {
		class server {
			public:
				server(boost::asio::io_service& io_service);
				virtual ~server();

			protected:
				void accept();
				void accept_handler(boost::shared_ptr<tcp_connection> newConnection, const boost::system::error_code& err);

			private:
				boost::asio::ip::tcp::acceptor m_acceptor;
				boost::asio::ip::tcp::socket   m_tcpSocket;
		};
	}
}