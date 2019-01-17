/** \file gauge_helios.cpp
\brief Implementation of gauge_helios.h.
    

Created on:  05/29/2015 at 06:52 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/

#include "base/interfaces/com/interface_com.h"
#include <windows.h>
#include <sstream>


std::vector<std::string>  grBase::InterfaceCOM::GetAllPorts()
{

	std::vector<std::string> foundCOMPorts;

	HKEY registryKeyHandle;

	if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_ALL_ACCESS, &registryKeyHandle)) 
	{

		DWORD readEntry;
		int index = 0;
		do 
		{
			
			DWORD valueBufferSize = 50;
			TCHAR valueBuffer[50];
			DWORD dataBufferSize = 50;
			TCHAR dataBuffer[50];

			readEntry = RegEnumValue(registryKeyHandle, index, valueBuffer, &valueBufferSize, NULL, NULL, (LPBYTE)dataBuffer, &dataBufferSize);
			if (ERROR_SUCCESS == readEntry)
			{
				// convert to C++ string
				dataBuffer[49] = '\0';
				std::wstring regValueW(dataBuffer);
				std::string regValue(regValueW.begin(), regValueW.end());
				foundCOMPorts.push_back(regValue);
			}
			index++;
		} while (ERROR_SUCCESS == readEntry);
		RegCloseKey(registryKeyHandle);
	}

	return foundCOMPorts;

	/*
	HANDLE hCom = nullptr;

	size_t maxCOMPort = 20;
	std::vector<std::string> foundCOMPorts;

	for (size_t i = 1; i <= maxCOMPort; ++i)
	{
		std::ostringstream ss;
		ss << "COM" << i;

		std::string currentPort = ss.str();

		hCom = CreateFile(std::wstring(currentPort.begin(), currentPort.end()).c_str(),
			GENERIC_READ | GENERIC_WRITE, // desired access should be read&write  
			0,                          // COM port must be opened in non-sharing mode  
			NULL,                       // don't care about the security  
			OPEN_EXISTING,              // IMPORTANT: must use OPEN_EXISTING for a COM port  
			0,                          // usually overlapped but non-overlapped for existance test  
			NULL);                      // always NULL for a general purpose COM port  

		if (INVALID_HANDLE_VALUE != hCom)
		{   // COM port exist  
			foundCOMPorts.push_back(currentPort);
			CloseHandle(hCom);
		}
	}
	return foundCOMPorts;
	*/

}