#pragma once
#pragma comment(lib, "ws2_32.lib")
#include <iostream>
#include <WinSock2.h>
#include <string>


namespace ws_tech
{
	class UDPSender
	{
	public:
		UDPSender(const std::string& ip, const int& port);
		~UDPSender();

		void Send(std::string data);

	private:
		WSADATA wsaData;
		SOCKET sendSocket;
		sockaddr_in recvAddr;
	};
}
