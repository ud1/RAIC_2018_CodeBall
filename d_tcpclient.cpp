//
// Created by denis on 03.12.18.
//

#include <boost/asio.hpp>
#include <thread>
#include "d_format.hpp"
#include <iostream>
#include "d_tcpclient.hpp"
#include <boost/endian/conversion.hpp>

using boost::asio::ip::tcp;

void TcpClient::run() {
    tcp::resolver resolver(io_service);
    tcp::resolver::query query("localhost", "8400");
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    connect(socket, endpoint_iterator);

    readTcpThread = std::thread([this](){
        try
        {
            for (;;)
            {
                boost::system::error_code error;

                uint32_t size;
                read(socket, boost::asio::buffer(&size, sizeof(size)), error);

                std::vector<char> data;
                if (!error)
                {
                    size = boost::endian::big_to_native(size);
                    data.resize(size);
                    read(socket, boost::asio::buffer(data.data(), size), error);
                }

                if (error == boost::asio::error::eof)
                {
                    LOG("EOF");
                    break; // Connection closed cleanly by peer.
                }
                else if (error)
                    throw boost::system::system_error(error); // Some other error.

                std::string command{data.data(), data.data() + data.size()};
                std::istringstream iss(command);
                Obj obj = readObj(iss);
                if (iss)
                    inMsgs.push(obj);
                else
                    LOG("ERROR parse obj");
            }
        }
        catch (std::exception& e)
        {
            inMsgs.push(Obj());
            std::cerr << e.what() << std::endl;
        }

        inMsgs.push(Obj());
        LOG("TERMINATE TCP");
    });
}

void TcpClient::sendCmd(std::string cmd)
{
    uint32_t size = cmd.size();

    //LOG("S " << size << " " << cmd[0]);
    size = boost::endian::native_to_big(size);
    write(socket, boost::asio::buffer(&size, sizeof(size)));
    write(socket, boost::asio::buffer(cmd, cmd.size()));
}

void TcpClient::sendObj(const Obj &obj)
{
    std::ostringstream oss;
    writeObj(oss, obj);
    sendCmd(oss.str());
}