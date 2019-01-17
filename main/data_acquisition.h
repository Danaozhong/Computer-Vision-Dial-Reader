/** \file gauge.h
\brief Class to read from a HELIOS gauge (connected via COM)
    
Pure virtual class to read data from a generic gauge.

Created on:  05/28/2015 at 05:13 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef __data_acquisition_H__
#define __data_acquisition_H__

#include "main/data/data_acquisition_data.h"
#include "main/data/data_acquisition_configuration.h"

namespace grMain
{
	class DataAcquisition
	{
	public:
		/// The collected data.
		grMain::DataCollection data;

		/// The data acquisition data.
		grMain::DataAcquisitionConfiguration acquisitionConfiguration;

		/// The current Interval.
		size_t currentInterval;

		bool completed;

		boost::posix_time::ptime lastTimeStamp;

		boost::posix_time::ptime initialTime;
	};
}

#endif
