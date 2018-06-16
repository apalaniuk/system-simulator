#include "stdafx.h"

#include "server.h++"

namespace sim {
	namespace networking {
		server::server(boost::asio::io_service& io_service) :
			m_tcpSocket(io_service),
			m_acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 2014)) {

			m_acceptor.listen();
			accept();
		}

		server::~server() { }

		void server::accept() {
			tcp_connection::pointer new_connection =
				tcp_connection::create(m_acceptor.get_io_service());

			m_acceptor.async_accept(new_connection->socket(),
				boost::bind(&server::accept_handler, this, new_connection, boost::asio::placeholders::error));
		}

		void server::accept_handler(boost::shared_ptr<tcp_connection> new_connection, const boost::system::error_code& err) {
			if (!err)
			{
				std::cout << "A new client has joined the fray." << std::endl;
				new_connection->start();
			}

			accept();
		}
	}
}