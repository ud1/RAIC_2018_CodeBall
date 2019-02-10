#ifndef UD1_VIEWER_TCPCLIENT_HPP
#define UD1_VIEWER_TCPCLIENT_HPP

#include "d_format.hpp"

#include <thread>
#include <boost/asio.hpp>
#include "d_blockingqueue.hpp"

struct TcpClient {
    BlockingQueue<Obj> inMsgs;
    std::thread readTcpThread;
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;

    TcpClient() : socket(io_service) {}

    void sendCmd(std::string cmd);

    void run();

    void sendObj(const Obj &obj);

    ~TcpClient()
    {
        socket.close();
        io_service.stop();

        if (readTcpThread.joinable())
            readTcpThread.join();
    }
};

#endif //UD1_VIEWER_TCPCLIENT_HPP
