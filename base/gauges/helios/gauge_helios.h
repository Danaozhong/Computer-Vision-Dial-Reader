/** \file gauge_helios.h
\brief Class to read a gauge manufactured by HELIOS via the COM interface.
    

Created on:  05/28/2015 at 06:04 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef __base_gauge_helios_H__
#define __base_gauge_helios_H__

#include "base\gauges\gauge.h"


#include <string>
#include <atomic>
#include <tbb/tbb.h>

namespace grBase
{
	class GaugeHeliosCOM : public Gauge
	{
	protected:
		/// The string of the COM port.
		const std::string com_port_id;

		/// The thread handeling the data acquisition.
		tbb::tbb_thread* threadDataAcquisition;

		std::atomic<bool> isConnected;

		/// flag to indicate that the data thread should acquire a new value.
		std::atomic<bool> requestDataAcquisition;
		
		/// flag to indicate that the data thread is busy with acquiring data.
		std::atomic<bool> dataAcquisitionInProgress;

		/// flag to indicate that the object is being deleted (thread must be terminated!)
		std::atomic<bool> terminate;

		/// the data value read from the gauge.
		std::atomic<double> gaugeValue;

		virtual GaugeHeliosCOM* doClone() const;
		virtual GaugeHeliosCOM* doCreate() const;
	public:

		/**
		Constructor.
		*/
		GaugeHeliosCOM(const std::string &com_id);

		/**
		Copy constructor.
		*/
		GaugeHeliosCOM(const GaugeHeliosCOM &other);

		/**
		Destructor.
		*/
		virtual ~GaugeHeliosCOM();

		virtual GaugeType GetType() const;

		std::string GetCOMPort() const;

		virtual bool DataAcquisitionInProgress() const;

		virtual bool IsConnected() const;

		/**
		Tries to read a new value from the gauge.
		*/
		virtual int Acquire();

		/**
		read the last value.
		*/
		virtual int GetValue(double &value);

		virtual int WaitUntilValueHasBeenReceived();

		/**
		Acquire data.
		*/
		static int Acquire(void*& hComm, double &value);


		static int Connect(void*& hComm, const std::string &com_port_id);
		
		static int Disconnect(void*& hComm);

		static int DataAcquisitionThread(GaugeHeliosCOM* _this);

	};
}

#endif
