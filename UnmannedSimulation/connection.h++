#pragma once

#include <boost/asio/read.hpp>
#include <boost/asio/completion_condition.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "message_handler.h++"
#include "message_types.h++"

class tcp_connection :
	public boost::enable_shared_from_this<tcp_connection>
{
public:
	typedef boost::shared_ptr<tcp_connection> pointer;

	static pointer create(boost::asio::io_service& io_service)
	{
		return pointer(new tcp_connection(io_service));
	}

	boost::asio::ip::tcp::socket& socket()
	{
		return m_socket;
	}

	void start()
	{
		boost::asio::async_read(
			m_socket,
			m_message,
			boost::asio::transfer_at_least(1),
			boost::bind(&tcp_connection::handle_read, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
		);
	}

private:
	tcp_connection(boost::asio::io_service& io_service)
		: m_socket(io_service)
	{
	}

	void handle_write(const boost::system::error_code& /*error*/,
		size_t /*bytes_transferred*/)
	{
	}

	void handle_read(const boost::system::error_code& error, size_t bytes_transferred)
	{
		std::cout << "Handle Read of connection\n";

		if (!error)
		{
			std::vector<uint8_t> target(m_message.size());
			buffer_copy(boost::asio::buffer(target), m_message.data());

			m_message.consume(bytes_transferred);

			m_message_handler.process_message(sim::message::message_type::MT_SIM_ENTITY_INFO, target);

			start();
		}
		else
		{
			if (error.value() == boost::asio::error::connection_reset) {
				std::cout << "Connection closed." << std::endl;
			} else {
				std::cout << "Connection error: " << error.message() << std::endl;
			}

			m_socket.close();
		}
	}

	boost::asio::ip::tcp::socket m_socket;
	boost::asio::streambuf m_message;
	sim::networking::message_handler m_message_handler;
};