/** \file gauge.cpp
\brief Implementation of gauge.h.


Created on:  05/28/2015 at 09:06 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#include "base/gauges/gauge.h"
#include "base/gauges/helios/gauge_helios.h"

using namespace grBase;



std::string grBase::ToString(GaugeType type)
{
	if (GAUGE_TYPE_HELIOS_COM == type)
	{
		return "Helios Gauge (COM)";
	}
	return "unknown";
}


Gauge::Gauge() {}

Gauge::~Gauge() {}


int Gauge::SetIdentifier(const std::string &identifier)
{
	this->identifier = identifier;
	return 0;
}

std::string Gauge::GetIdentifier() const
{
	return this->identifier;
}

GaugeType Gauge::GetType() const
{
	return GAUGE_TYPE_HELIOS_COM;
}

bool Gauge::DataAcquisitionInProgress() const
{
	return false;
}

bool Gauge::IsConnected() const
{
	return true;
}

int Gauge::Acquire()
{
	return 0;
}

int Gauge::WaitUntilValueHasBeenReceived()
{
	return 0;
}


int Gauge::AcquireAndRead(double &value)
{
	this->Acquire();
	// wait for the thread to acquire the values
	if (0 == WaitUntilValueHasBeenReceived())
	{
		return this->GetValue(value);
	}

	// probably timeout
	return -1;
}

Gauge* GaugeHelper::GaugeFactory(GaugeType type)
{
	switch (type)
	{
	case GAUGE_TYPE_HELIOS_COM:
		return new GaugeHeliosCOM("COMX");
	}
}