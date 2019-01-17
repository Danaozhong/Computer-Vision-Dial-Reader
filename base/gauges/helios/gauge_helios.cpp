/** \file gauge_helios.cpp
\brief Implementation of gauge_helios.h.
    

Created on:  05/28/2015 at 06:06 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#include <boost/lexical_cast.hpp>
#include "base/gauges/helios/gauge_helios.h"



#include <thread>
#include <iostream>
#include <stdio.h>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace grBase;

GaugeHeliosCOM* GaugeHeliosCOM::doClone() const
{
	return new GaugeHeliosCOM(*this);
}

GaugeHeliosCOM* GaugeHeliosCOM::doCreate() const
{
	return new GaugeHeliosCOM("COM0");
}

GaugeHeliosCOM::GaugeHeliosCOM(const std::string &com_id)
: com_port_id(com_id), threadDataAcquisition(nullptr), dataAcquisitionInProgress(false), 
requestDataAcquisition(false), isConnected(false), terminate(false)
{
	// spawn the data acquisition thread.
	this->threadDataAcquisition = new tbb::tbb_thread(GaugeHeliosCOM::DataAcquisitionThread, this);
}

GaugeHeliosCOM::GaugeHeliosCOM(const GaugeHeliosCOM &other)
: com_port_id(other.com_port_id), threadDataAcquisition(nullptr), dataAcquisitionInProgress(false),
requestDataAcquisition(false), isConnected(false), terminate(false)
{
	// spawn the data acquisition thread.
	this->threadDataAcquisition = new tbb::tbb_thread(GaugeHeliosCOM::DataAcquisitionThread, this);
}

GaugeHeliosCOM::~GaugeHeliosCOM()
{
	this->terminate = true;

	// wait for the thread to terminate.
	if (nullptr != this->threadDataAcquisition)
	{
		this->threadDataAcquisition->join();
		delete(this->threadDataAcquisition);
		this->threadDataAcquisition = 0;

	}
}

GaugeType GaugeHeliosCOM::GetType() const
{
	return GAUGE_TYPE_HELIOS_COM;
}

std::string GaugeHeliosCOM::GetCOMPort() const
{
	return this->com_port_id;
}

bool GaugeHeliosCOM::DataAcquisitionInProgress() const
{
	return this->dataAcquisitionInProgress;
}

bool GaugeHeliosCOM::IsConnected() const
{
	return this->isConnected;
}

int GaugeHeliosCOM::Acquire()
{
	this->requestDataAcquisition = true;
	return 0;
}


int GaugeHeliosCOM::GetValue(double &value)
{
	
	value = this->gaugeValue;
	if (true == this->isConnected)
	{
		return 0;
	}
	return -1;
}

int GaugeHeliosCOM::WaitUntilValueHasBeenReceived()
{
	auto beginTimeStamp = boost::posix_time::microsec_clock::local_time();

	do
	{
		auto interval = boost::posix_time::microsec_clock::local_time() - beginTimeStamp;

		if (interval.total_milliseconds() > 2000)
		{
			// timeout!
			return -1;
		}
			

	} while (this->dataAcquisitionInProgress);

	if (this->isConnected)
	{
		return 0;
	}
	return -1;
}




int GaugeHeliosCOM::Acquire(void*& hComm, double &value)
{
	if (INVALID_HANDLE_VALUE == hComm)
	{
		// connection terminated or not established.
		//if (0 != this->Connect())
		//{
			// connection failed.
			return -1;
		//}
	}

	// wait at least 300 ms with DTR set to high.
	std::this_thread::sleep_for(std::chrono::milliseconds(300));


	// Set DTR to 0
	if (0 == EscapeCommFunction(hComm, CLRDTR))
	{
		Disconnect(hComm);
		return -5;
	}

	// wait 150 ms
	std::this_thread::sleep_for(std::chrono::milliseconds(150));

	// reenable DTR
	if (0 == EscapeCommFunction(hComm, SETDTR))
	{
		Disconnect(hComm);
		return -2;
	}

	// wait and try to receive data. 
	const DWORD num_of_bytes_to_read = 9;
	DWORD num_of_bytes_read = 0;

	char buffer[num_of_bytes_to_read];

	if (0 == ReadFile(hComm, buffer, num_of_bytes_to_read, &num_of_bytes_read, NULL))
	{
		// reading from the COM port failed.
		Disconnect(hComm);
		return -3;
	}

	// set the last byte manually to avoid buffer overruns
	buffer[num_of_bytes_to_read - 1] = '\0';

	if (num_of_bytes_to_read == num_of_bytes_read)
	{
		// reading was successful! Now, first, get away from that ugly C and write clean C++ code again.
		try 
		{
			value = boost::lexical_cast<double>(buffer);
		}
		catch (boost::bad_lexical_cast const&) 
		{
			// conversion failed.
			Disconnect(hComm);
			return -5;
		}
		return 0;
	}
	return -7;
}



int GaugeHeliosCOM::Connect(void*& hComm, const std::string &com_port_id)
{
	// Oldschool-C-Code used to be able to set DTR on the RS232 port manually.
	hComm = CreateFile(std::wstring(com_port_id.begin(), com_port_id.end()).c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);
	if (INVALID_HANDLE_VALUE == hComm)
	{
		// error opening port; abort
		return -1;
	}

	// set the parameters
	DCB serial_params;
	memset(&serial_params, 0, sizeof(serial_params));
	serial_params.DCBlength = sizeof(serial_params);

	if (0 == GetCommState(hComm, &serial_params))
	{
		// error getting COM state
		Disconnect(hComm);
		return -2;
	}

	serial_params.BaudRate = CBR_4800;
	serial_params.fParity = TRUE;
	serial_params.ByteSize = 7;
	serial_params.StopBits = TWOSTOPBITS;
	serial_params.Parity = EVENPARITY;

	if (0 == SetCommState(hComm, &serial_params))
	{
		// failed to set the params.
		Disconnect(hComm);
		return -3;
	}

	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 300;
	timeouts.ReadTotalTimeoutConstant = 300;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (0 == SetCommTimeouts(hComm, &timeouts))
	{
		// failed to set timeouts.
		Disconnect(hComm);
		return -4;
	}
	return 0;
}


int GaugeHeliosCOM::Disconnect(void*& hComm)
{
	if (INVALID_HANDLE_VALUE == hComm)
	{
		// handle closed already.
		return 0;
	}

	if (0 == CloseHandle(hComm))
	{
		// function failed.
		return -1;
	}
	hComm = INVALID_HANDLE_VALUE;
	return 0;
}

int GaugeHeliosCOM::DataAcquisitionThread(GaugeHeliosCOM* _this)
{
	/// The handle used for the COM port communication.
	HANDLE hComm;

	GaugeHeliosCOM::Connect(hComm, _this->com_port_id);

	while (false == _this->terminate)
	{
		if (true == _this->requestDataAcquisition)
		{
			_this->requestDataAcquisition = false;
			_this->dataAcquisitionInProgress = true;
			double readValue = 0.0;
			if (0 == GaugeHeliosCOM::Acquire(hComm, readValue))
			{
				// data acquisition successful!
				// update the value in the main thread.
				_this->gaugeValue = readValue;
				_this->isConnected = true;
			}
			else
			{
				// data acquisition failed.
				_this->isConnected = false;
			}


			_this->dataAcquisitionInProgress = false;
		}
	}

	// disconnect...
	GaugeHeliosCOM::Disconnect(hComm);

	//...and terminate the thread!
	return 0;
}
