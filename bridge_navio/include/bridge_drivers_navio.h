//----------------------------------------------------------------------------------------------------------------------
// GRVC AUTOPILOT
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#ifndef BRIDGE_DRIVERS_NAVIO_H_
#define BRIDGE_DRIVERS_NAVIO_H_

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Util.h>
#include <shm_util.h>
#include <navio_types.h>
#include <shm_channels.h>
#include <vector>

#include "common.h"

/// Manage the Navio2 sensors data and others sensors connected to Raspberry pi, to communicate with Simulink external mode.
class BridgeDriversNavio{
public:
	BridgeDriversNavio(int argc, char * argv[]);
	BridgeDriversNavio();
	~BridgeDriversNavio();

	/// Launch all threads of sensors
	void runThreads();

	/// Access to the shared memory to write or read the sensors data.
	void setShm();

	/// Attributes
	std::vector<std::string> _sensor;
	bool _calib_rotors;
	int8_t _num_rotors;
	mpu9250_imu_str _mpu9250_data;
	lsm9ds1_imu_str _lsm9ds1_data;
	baro_str _barometer_data;
	rcio_str _rcio_data;
	adc_str _adc_data;
	sf11c_str _sf11c_data;
	totalStation_str _totalstation_data;
	gps_str _gps_data;
	px4flow_str _px4flow_data;
	svo_str _svo_data;
	vicon_str _vicon_data;
	teraranger_str _teraranger_data;
	status_str _status_data;

private:

	/// Unlink, close and remove the addresses of the shared memory and the semaphores associated.
	static void close(int sig);


};



#endif /* BRIDGE_DRIVERS_NAVIO_H_ */
