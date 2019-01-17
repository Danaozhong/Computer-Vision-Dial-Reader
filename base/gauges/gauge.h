/** \file gauge.h
\brief Provides the class used to read a gauge.
    
Pure virtual class to read data from a generic gauge.

Created on:  05/28/2015 at 05:13 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef __base_gauge_H__
#define __base_gauge_H__

#include <string>
#include <memory>

namespace grBase
{

	enum GaugeType
	{
		GAUGE_TYPE_HELIOS_COM,
		GAUGE_TYPE_CAMERA,
		GAUGE_TYPE_TCPIP
	};

	std::string ToString(GaugeType type);

	class Gauge
	{
	protected:
		double min;
		double max;

		
		std::string identifier;

		virtual Gauge* doClone() const = 0;
		virtual Gauge* doCreate() const = 0;
	public:

		/**
		Constructor.
		*/
		Gauge();

		/**
		Destructor.
		*/
		virtual ~Gauge() = 0;

		std::unique_ptr<Gauge> Clone() const { return std::unique_ptr<Gauge>(this->doClone()); }
		std::unique_ptr<Gauge> Create() const { return std::unique_ptr<Gauge>(this->doCreate()); }

		virtual int SetIdentifier(const std::string &identifier);
		virtual std::string GetIdentifier() const;

		virtual GaugeType GetType() const = 0;

		virtual bool DataAcquisitionInProgress() const;


		virtual bool IsConnected() const;
		
		/**
		Tries to read a new value from the gauge. 
		*/
		virtual int Acquire();

		/**
		read the last value.
		*/
		virtual int GetValue(double &value) = 0;


		virtual int WaitUntilValueHasBeenReceived();

		// acquires a value and waits until it is received.
		virtual int AcquireAndRead(double &value);
	};


	namespace GaugeHelper
	{
		Gauge* GaugeFactory(GaugeType type);
	}
}

#endif
