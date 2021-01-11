#pragma once

#include "Scenario.h"
#include <zmq.hpp>
#include "Functions.h"


class Server {
private:
	zmq::context_t ctx;
	zmq::socket_t socket;


	std::clock_t lastSentMessageTime = std::clock();


public:
	Scenario scenario;
	
	bool clientStarted = false;

	Server(unsigned int port);
	void checkRecvMessage();
	void checkSendMessage();
	//void checkClient();
};