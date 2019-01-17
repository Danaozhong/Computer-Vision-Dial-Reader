/** \file main.cpp
\brief Class to read a gauge manufactured by HELIOS via the COM interface.


Created on:  05/28/2015 at 06:04 PM by Clemens Zangl.
Copyright:   Texelography. All Rights Reserved.
*/



#include "main/main.h"
#include "main/main_globals.h"

#include <thread>
#include <iostream>

using namespace grMain;

IMPLEMENT_APP(GaugeReaderApp)




bool GaugeReaderApp::OnInit()
{
	// create editor control handle
	/*
	try
	{
		editor = new Editor("config.ini");
	}
	catch (...)
	{
		fatalError("Creation of the main program class failed");
		return false;
	}

	// load the additional panels
	if (0 != editor->LoadPanels())
	{
		fatalError("Loading of the panels failed.");
		delete editor;
		editor = 0;
		return false;
	}

	// the editor class has been loaded successfully. Now, we need to run some initialization functions to start it
	editor->Init();

	*/

	this->exitApplication = false;
	editor = this;

	this->Init();
	// connect the idle event to the render function
	Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(GaugeReaderApp::onIdle));

	return true;
}

int GaugeReaderApp::OnExit()
{
	// detach the render function from the idle event
	this->project->gaugeList.gauges.clear();



	Disconnect(wxEVT_IDLE, wxIdleEventHandler(GaugeReaderApp::onIdle));
	//editor->exitApplication = false;

	// destroy all objects
	//delete editor;

	return 0;
}


void GaugeReaderApp::onIdle(wxIdleEvent& evt)
{
	//editor->Process();
	//ProcessSystemEvents();
	this->Process();

	evt.RequestMore(); // render continuously, not only once on idle
}

std::string GaugeReaderApp::GetVersionInfo()
{
	return "0.01";
}

int GaugeReaderApp::LoadPanels()
{
	// load panels
	this->formMain = new FormMain(0);


	// check if the panel creation was successful.
	if (nullptr == this->formMain)
	{
		return -1;
	}
	return 0;
}

int GaugeReaderApp::Init()
{
	auto myGauge = std::shared_ptr<grBase::Gauge>(new grBase::GaugeHeliosCOM("COM6"));

	this->project = std::shared_ptr<Project>(new Project());
	this->project->gaugeList.gauges.push_back(GaugeListItem(myGauge));

	this->LoadPanels();

	// set the title 
	this->formMain->SetTitle("Gauge Reader Software " + GaugeReaderApp::GetVersionInfo());

	this->programMode = grMain::PM_DEFAULT;

	// show the main panel
	this->formMain->Show();


	return 0;
}

int GaugeReaderApp::Process()
{
	if (this->exitApplication)
	{
		return 0;
	}

	if (grMain::PM_DEFAULT == editor->programMode)
	{
		double value;

		for (auto itr = this->project->gaugeList.gauges.begin(); itr != this->project->gaugeList.gauges.end(); itr++)
		{
			itr->gauge->Acquire();
		}

		this->formMain->UpdateGaugeList(this->project->gaugeList);
	}
	else if (grMain::PM_DATA_ACQUISITION == editor->programMode)
	{
		this->formMain->dialogDataAcquisition->Process();
	}
	return 0;
}