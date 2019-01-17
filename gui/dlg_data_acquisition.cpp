#include "gui/dlg_data_acquisition.h"
#include "main/main_globals.h"
#include "common/misc/cmn_misc.h"

#include <iostream>
#include <fstream>

DialogDataAcquisition::DialogDataAcquisition(wxWindow* parent, DialogDataAcquisitionParent* data_parent)
:
dlgDataAcquisition(parent), dataParent(data_parent)
{

}

void DialogDataAcquisition::OnClose(wxCloseEvent& event)
{
	if (true == this->data.completed)
	{

	}
	else
	{
		if (false == grBase::MessageQuestionYesNo("The data acquisition has not yet been completed. Are you sure you want to cancel?"))
		{
			event.Veto();
		}
	}

	event.Skip();
}

void DialogDataAcquisition::OnBtnStopAcquisitionClick(wxCommandEvent& event)
{

}


int DialogDataAcquisition::ShowForm()
{
	if (this->IsVisible())
	{
		return -1;
	}
	if (grMain::PM_DEFAULT != editor->programMode)
	{
		// wrong program mode.
		return -2;
	}
	editor->programMode = grMain::PM_DATA_ACQUISITION;
	this->data.lastTimeStamp = boost::posix_time::second_clock::local_time();
	this->data.initialTime = this->data.lastTimeStamp;
	this->data.acquisitionConfiguration = editor->project->dataAcquisitionConfiguration;

	this->data.data = grMain::DataCollection();
	this->data.data.gauges = editor->project->gaugeList.gauges;
	this->data.completed = false;
	this->data.currentInterval = 0;
	this->Show();
	return 0;
}

int DialogDataAcquisition::HideForm()
{
	this->Close();
	return 0;
}

int DialogDataAcquisition::Process()
{
	if (false == this->data.completed)
	{
		// get the current time.
		auto currentTime = boost::posix_time::second_clock::local_time();

		// get the duration:
		auto passedTime = currentTime - this->data.lastTimeStamp;

		if (passedTime > this->data.acquisitionConfiguration.GetInterval())
		{
			// acquire data from all gauges
			this->data.data.CollectData();
			this->data.currentInterval++;
			this->data.lastTimeStamp = currentTime;

			if (this->data.currentInterval > this->data.acquisitionConfiguration.GetNumOfIntervals())
			{
				// data acquisition is completed!
				this->data.completed = true;
				
				// Export to file
				std::ofstream fileStream;
				fileStream.open(this->data.acquisitionConfiguration.GetDataFileName());
				if (false == fileStream.is_open())
				{
					this->statusLabel = "File could not be stored!";
				}
				else
				{
					if (0 == this->data.data.StoreInCSV(fileStream))
					{
						this->statusLabel = "Data successfully written in " + this->data.acquisitionConfiguration.GetDataFileName();
					}
					else
					{
						this->statusLabel = "Error storing data.";
					}
					fileStream.close();
				};
			}
			else
			{
				std::ostringstream ss;
				ss << "In progress... (" << this->data.currentInterval << " of " << this->data.acquisitionConfiguration.GetNumOfIntervals() << " data sets collected).";
				this->statusLabel = ss.str();
			}

			this->UpdateForm();
		}
	}
	return 0;
}

int DialogDataAcquisition::UpdateForm()
{
	this->lblDataAcquisitionStatusValue->SetLabel(this->statusLabel);
	
	if (false == this->data.completed)
	{
		this->lblDuration->SetLabel(std::to_string((boost::posix_time::second_clock::local_time() - this->data.initialTime).total_seconds()));
	}
	else
	{


	}

	return 0;
}
