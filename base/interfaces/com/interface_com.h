/** \file interface_com.h
\brief Helper functions for the COM interface.
    

Created on:  05/29/2015 at 06:51 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#ifndef __file_interface_com_H__
#define __file_interface_com_H__


#include <string>
#include <vector>

namespace grBase
{
	namespace InterfaceCOM
	{

		std::vector<std::string> GetAllPorts();
	}
}

#endif
