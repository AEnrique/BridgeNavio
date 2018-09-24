/*
 * serial_comm.cpp
 *
 *  Created on: 13 jul. 2018
 *      Author: ae-grvc
 */
#include <serial_comm.h>
#include <boost/bind.hpp>
// ROS includes
//#include <px_comm/OpticalFlow.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/Image.h>


namespace px
{

SerialComm::SerialComm(const std::string& frameId)
 : m_port(m_uartService)
 , m_timer(m_uartService)
 , m_systemId(-1)
 , m_compId(0)
 , m_frameId(frameId)
 , m_timeout(false)
 , m_errorCount(0)
 , m_connected(false)
{
  m_optFlowMsg = {0.0,0,0,0.0,0.0,0};
}

SerialComm::~SerialComm()
{
    if (m_connected)
    {
        close();
    }
}

bool
SerialComm::open(const std::string& portStr, int baudrate)
{
    m_timeout = false;
    m_errorCount = 0;
    m_connected = false;

    // open port
    try
    {
        m_port.open(portStr);

        printf("Opened serial port %s.", portStr.c_str());
    }
    catch (boost::system::system_error const& e)
    {
    	printf("Could not open serial port %s. Reason: %s.", portStr.c_str(), e.what());

        return false;
    }

    // configure baud rate
    try
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        m_port.set_option(boost::asio::serial_port_base::character_size(8));

        printf("Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error const& e)
    {
    	printf("Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }

    //ros::NodeHandle nh("px4flow");

    // set up publishers
    //m_optFlowPub = nh.advertise<px_comm::OpticalFlow>("opt_flow", 5);

    //image_transport::ImageTransport it(nh);
    //m_imagePub = it.advertise("camera_image", 5);

    // set up thread to asynchronously read data from serial port
    readStart(1000);
    m_uartThread = boost::thread(boost::bind(static_cast<size_t (boost::asio::io_service::*)()>(&boost::asio::io_service::run), &m_uartService));

    //m_syncTimer = nh.createTimer(ros::Duration(2.0), boost::bind(&SerialComm::syncCallback, this, _1));

    m_connected = true;

    return true;
}

void
SerialComm::close(void)
{
    if (!m_connected)
    {
        return;
    }
    m_uartService.post(boost::bind(static_cast<size_t (boost::asio::deadline_timer::*)()>(&boost::asio::deadline_timer::cancel), &m_timer));
    m_uartService.post(boost::bind(static_cast<void (boost::asio::serial_port::*)()>(&boost::asio::serial_port::close), &m_port));

    //m_uartService.post(boost::bind(&boost::asio::serial_port::close, &m_port));

    m_uartThread.join();
}

void
SerialComm::readCallback(const boost::system::error_code& error, size_t bytesTransferred)
{
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (m_timeout)
            {
                m_timeout = false;
                readStart(1000);

                return;
            }
        }

        printf("Read error: %s", error.message().c_str());

        if (m_errorCount < 10)
        {
            readStart(1000);
        }
        else
        {
        	printf("# read errors exceeded 10. Aborting...");
            //ros::shutdown();
        }

        ++m_errorCount;

        return;
    }

    m_timer.cancel();

    mavlink_message_t message;
    mavlink_status_t status;

    for (size_t i = 0; i < bytesTransferred; i++) {
        bool msgReceived = mavlink_parse_char(MAVLINK_COMM_1, m_buffer[i], &message, &status);

        if (msgReceived)
        {
            m_systemId = message.sysid;

            switch (message.msgid)
            {
				case MAVLINK_MSG_ID_OPTICAL_FLOW:
				{
					// decode message
					mavlink_optical_flow_t flow;
					mavlink_msg_optical_flow_decode(&message, &flow);



					m_optFlowMsg._ground_distance = flow.ground_distance;
					m_optFlowMsg._flow_x = flow.flow_x;
					m_optFlowMsg._flow_y = flow.flow_y;
					m_optFlowMsg._velocity_x = flow.flow_comp_m_x;
					m_optFlowMsg._velocity_y = flow.flow_comp_m_y;
					m_optFlowMsg._quality = flow.quality;

					//printf("%d\n", optFlowMsg.flow_y);

					break;
				}
            }
        }
    }

    readStart(1000);
}

void
SerialComm::readStart(uint32_t timeout_ms)
{
    m_port.async_read_some(boost::asio::buffer(m_buffer, sizeof(m_buffer)),
                            boost::bind(&SerialComm::readCallback, this, boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));

    if (timeout_ms != 0)
    {
        m_timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
        m_timer.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}


void
SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        m_port.cancel();
        m_timeout = true;
        printf("WARNING:Serial connection timed out.");
    }
}

}


