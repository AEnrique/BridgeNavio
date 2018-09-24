/*
 * main.cpp
 *
 *  Created on: 15/3/2018
 *      Author: GRVC
 */
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <bridge_drivers_navio.h>


using namespace std;

int main(int argc, char * argv[]) {
	try
	{
		BridgeDriversNavio *navio2;
		navio2= new BridgeDriversNavio(argc, argv);

		boost::asio::io_service io;

		navio2->runThreads();

	    while (true) {
	    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
	    	navio2->setShm();
	    	//printf("2 SVO -> x: %f| y: %f| z: %f\n",navio2->_svo_data._shmmsg._x,navio2->_svo_data._shmmsg._y,navio2->_svo_data._shmmsg._z);
			t.wait();
	    }

	}catch(int error)
	{

		cout << "EXIT" << endl;
	}
}



