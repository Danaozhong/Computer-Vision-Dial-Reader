///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun  5 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GUI_BASE_H__
#define __GUI_BASE_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/grid.h>
#include <wx/sizer.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/radiobut.h>
#include <wx/choice.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/spinctrl.h>
#include <wx/gauge.h>

///////////////////////////////////////////////////////////////////////////

#define ID_SAVE 1000
#define ID_QUIT 1001
#define ID_ADD 1002
#define ID_MODIFY 1003
#define ID_DELETE 1004
#define ID_CONFIGURE_DATA_ACQUISITION 1005
#define ID_START_DATA_ACQUISITION 1006

///////////////////////////////////////////////////////////////////////////////
/// Class frmMain
///////////////////////////////////////////////////////////////////////////////
class frmMain : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* m_menubar1;
		wxMenu* file;
		wxMenu* edit;
		wxMenu* gauges;
		wxGrid* grdGauges;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnMnuFileSaveClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuFileQuitClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuEditGaugesAddClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuEditGaugesEditClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuEditGaugesDeleteClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuEditConfigureDataAcquisitionClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMnuEditStartDataAcquisitionClick( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		frmMain( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Gauge Reader v 0.1"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 724,370 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~frmMain();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class dlgGaugeConfiguration
///////////////////////////////////////////////////////////////////////////////
class dlgGaugeConfiguration : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* lblGaugeName;
		wxTextCtrl* txtGaugeName;
		wxStaticText* m_staticText2;
		wxRadioButton* rdbCOM;
		wxChoice* drpCOM;
		wxRadioButton* rdbCamera;
		wxRadioButton* rdbTCPIP;
		wxPanel* m_panel1;
		wxButton* btnCancel;
		wxButton* btnOK;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnBtnCancelClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBtnOKClick( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		dlgGaugeConfiguration( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Gauge Configuration"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 454,213 ), long style = wxDEFAULT_DIALOG_STYLE ); 
		~dlgGaugeConfiguration();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class pnlUSB
///////////////////////////////////////////////////////////////////////////////
class pnlUSB : public wxPanel 
{
	private:
	
	protected:
	
	public:
		
		pnlUSB( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 227,118 ), long style = wxTAB_TRAVERSAL ); 
		~pnlUSB();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class dlgDataAcquisitionConfiguration
///////////////////////////////////////////////////////////////////////////////
class dlgDataAcquisitionConfiguration : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* lblGaugeName;
		wxSpinCtrl* numIntervalHours;
		wxStaticText* m_staticText20;
		wxSpinCtrl* numIntervalMinutes;
		wxStaticText* m_staticText21;
		wxSpinCtrl* numIntervalSeconds;
		wxStaticText* m_staticText22;
		wxStaticText* lblNumIntervals;
		wxSpinCtrl* numIntervals;
		wxStaticText* m_staticText16;
		wxStaticText* lblDuration;
		wxStaticText* m_staticText18;
		wxStaticText* lblFilePath;
		wxButton* btnSelectFilePath;
		wxStaticText* lblFileTypeDescription;
		wxStaticText* lblFileType;
		wxButton* btnCancel;
		wxButton* btnOK;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnBtnSelectFilePathClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBtnCancelClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBtnOKClick( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		dlgDataAcquisitionConfiguration( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Data Acquisition Configuration"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 454,303 ), long style = wxDEFAULT_DIALOG_STYLE ); 
		~dlgDataAcquisitionConfiguration();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class dlgDataAcquisition
///////////////////////////////////////////////////////////////////////////////
class dlgDataAcquisition : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* lblDataAcquistionStatus;
		wxStaticText* lblDataAcquisitionStatusValue;
		wxStaticText* lblNumOfDataSets;
		wxStaticText* m_staticText42;
		wxStaticText* m_staticText16;
		wxStaticText* lblDuration;
		wxStaticText* m_staticText18;
		wxStaticText* m_staticText19;
		wxGauge* prgProgress;
		wxButton* btnStop;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnBtnStopAcquisitionClick( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		dlgDataAcquisition( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Data Acquisition Configuration"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 454,218 ), long style = wxDEFAULT_DIALOG_STYLE ); 
		~dlgDataAcquisition();
	
};

#endif //__GUI_BASE_H__
