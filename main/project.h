#ifndef __project_H__
#define __project_H__

#include "main/data/gauge_list.h"
#include "main/data/data_acquisition_configuration.h"

#include <tinyxml2.h>

namespace grMain
{
	class Project
	{
	public:
		/// the list of gauges.
		GaugeList gaugeList;

		/// the data acquisition configuration.
		DataAcquisitionConfiguration dataAcquisitionConfiguration;
	};

	namespace ProjectHelper
	{
		/**
		Reads a project from an XML node.
		*/
		Project LoadProjectFromXML(const tinyxml2::XMLElement& node);

		/**
		Saves a complete project in a XML node.
		*/
		tinyxml2::XMLElement* SaveToXML(const Project &project, tinyxml2::XMLDocument &document, const std::string &name);

	}
}
#endif