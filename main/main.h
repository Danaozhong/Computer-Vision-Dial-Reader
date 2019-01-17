//==============================================================================
//
// Title:       main.h
// Purpose:     Test file to add functions used by wxWidgets

#ifndef __main_H__
#define __main_H__

#include "base/gauges/helios/gauge_helios.h"
#include "main/data/gauge_list.h"
#include "main/data/data_acquisition_configuration.h"
#include "main/project.h"
#include "gui/frm_main.h"

#include <wx/wx.h>
#include <memory>

/**
Forward declarations.
*/
class FormMain;

namespace grMain
{
	enum ProgramMode
	{
		PM_DEFAULT,
		PM_ADD_GAUGE,
		PM_EDIT_GAUGE,
		PM_CONFIGURE_DATA_ACQUISITION,
		PM_DATA_ACQUISITION
	};



	class GaugeReaderApp : public wxApp
	{
	protected:
		friend class FormMain;
		friend class DialogDataAcquisition;
		friend class DialogDataAcquisitionConfiguration;
		friend class DataCollection;

		bool exitApplication;

		ProgramMode programMode;

		/// The main panel.
		FormMain* formMain;

		/// the current project
		std::shared_ptr<grMain::Project> project;

	public:
		virtual bool OnInit();
		virtual int OnExit();
		void onIdle(wxIdleEvent& evt);

		static std::string GetVersionInfo();

		int LoadPanels();
		int Init();
		int Process();

	};

}
#endif