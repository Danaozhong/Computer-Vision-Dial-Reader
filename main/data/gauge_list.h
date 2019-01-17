/** \file gauge_list.h
\brief Provides the class used to read a gauge.
    
Pure virtual class to read data from a generic gauge.

Created on:  05/28/2015 at 05:13 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef __main_gauge_list_H__
#define __main_gauge_list_H__

#include "base/gauges/gauge.h"

#include <memory>
#include <vector>

namespace grMain
{
	class GaugeListItem
	{
	public:
		std::shared_ptr<grBase::Gauge> gauge;

		/**
		Constructor.
		*/
		GaugeListItem(const std::shared_ptr<grBase::Gauge> &gauge);


	};
	class GaugeList
	{
	public:
		std::vector<GaugeListItem> gauges;
	};
}

#endif
