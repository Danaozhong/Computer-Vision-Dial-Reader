/** \file gauge.cpp
\brief Implementation of gauge.h.


Created on:  05/29/2015 at 09:42 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#include "main/data/gauge_list.h"


using namespace grMain;

GaugeListItem::GaugeListItem(const std::shared_ptr<grBase::Gauge> &gauge)
: gauge(gauge)
{}
