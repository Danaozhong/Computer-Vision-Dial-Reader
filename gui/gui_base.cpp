///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun  5 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gui\gui_base.h"

///////////////////////////////////////////////////////////////////////////

frmMain::frmMain( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	m_menubar1 = new wxMenuBar( 0 );
	file = new wxMenu();
	wxMenuItem* mnuFileSave;
	mnuFileSave = new wxMenuItem( file, ID_SAVE, wxString( wxT("Save") ) , wxEmptyString, wxITEM_NORMAL );
	file->Append( mnuFileSave );
	
	wxMenuItem* numFileQuit;
	numFileQuit = new wxMenuItem( file, ID_QUIT, wxString( wxT("Quit") ) , wxEmptyString, wxITEM_NORMAL );
	file->Append( numFileQuit );
	
	m_menubar1->Append( file, wxT("File") ); 
	
	edit = new wxMenu();
	gauges = new wxMenu();
	wxMenuItem* gaugesItem = new wxMenuItem( edit, wxID_ANY, wxT("Gauges"), wxEmptyString, wxITEM_NORMAL, gauges );
	wxMenuItem* mnuEditGaugesAdd;
	mnuEditGaugesAdd = new wxMenuItem( gauges, ID_ADD, wxString( wxT("Add...") ) , wxEmptyString, wxITEM_NORMAL );
	gauges->Append( mnuEditGaugesAdd );
	
	wxMenuItem* mnuEditGaugesModify;
	mnuEditGaugesModify = new wxMenuItem( gauges, ID_MODIFY, wxString( wxT("Modify...") ) , wxEmptyString, wxITEM_NORMAL );
	gauges->Append( mnuEditGaugesModify );
	
	wxMenuItem* mnuEditGaugesDelete;
	mnuEditGaugesDelete = new wxMenuItem( gauges, ID_DELETE, wxString( wxT("Delete") ) , wxEmptyString, wxITEM_NORMAL );
	gauges->Append( mnuEditGaugesDelete );
	
	edit->Append( gaugesItem );
	
	wxMenuItem* mnuEditConfigureDataAcquisition;
	mnuEditConfigureDataAcquisition = new wxMenuItem( edit, ID_CONFIGURE_DATA_ACQUISITION, wxString( wxT("Configure Data Acquisition") ) , wxEmptyString, wxITEM_NORMAL );
	edit->Append( mnuEditConfigureDataAcquisition );
	
	wxMenuItem* mnuEditStartDataAcquisition;
	mnuEditStartDataAcquisition = new wxMenuItem( edit, ID_START_DATA_ACQUISITION, wxString( wxT("Start Data Acquisition") ) , wxEmptyString, wxITEM_NORMAL );
	edit->Append( mnuEditStartDataAcquisition );
	
	m_menubar1->Append( edit, wxT("Edit") ); 
	
	this->SetMenuBar( m_menubar1 );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	grdGauges = new wxGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	
	// Grid
	grdGauges->CreateGrid( 0, 5 );
	grdGauges->EnableEditing( false );
	grdGauges->EnableGridLines( true );
	grdGauges->EnableDragGridSize( false );
	grdGauges->SetMargins( 0, 0 );
	
	// Columns
	grdGauges->EnableDragColMove( true );
	grdGauges->EnableDragColSize( true );
	grdGauges->SetColLabelSize( 30 );
	grdGauges->SetColLabelValue( 0, wxT("ID") );
	grdGauges->SetColLabelValue( 1, wxT("Name") );
	grdGauges->SetColLabelValue( 2, wxT("Value") );
	grdGauges->SetColLabelValue( 3, wxT("Connection") );
	grdGauges->SetColLabelValue( 4, wxT("Attributes") );
	grdGauges->SetColLabelAlignment( wxALIGN_CENTRE, wxALIGN_CENTRE );
	
	// Rows
	grdGauges->EnableDragRowSize( false );
	grdGauges->SetRowLabelSize( 80 );
	grdGauges->SetRowLabelAlignment( wxALIGN_CENTRE, wxALIGN_CENTRE );
	
	// Label Appearance
	
	// Cell Defaults
	grdGauges->SetDefaultCellAlignment( wxALIGN_LEFT, wxALIGN_TOP );
	bSizer1->Add( grdGauges, 1, wxALL|wxEXPAND, 5 );
	
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( frmMain::OnClose ) );
	this->Connect( mnuFileSave->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuFileSaveClick ) );
	this->Connect( numFileQuit->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuFileQuitClick ) );
	this->Connect( mnuEditGaugesAdd->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesAddClick ) );
	this->Connect( mnuEditGaugesModify->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesEditClick ) );
	this->Connect( mnuEditGaugesDelete->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesDeleteClick ) );
	this->Connect( mnuEditConfigureDataAcquisition->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditConfigureDataAcquisitionClick ) );
	this->Connect( mnuEditStartDataAcquisition->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditStartDataAcquisitionClick ) );
}

frmMain::~frmMain()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( frmMain::OnClose ) );
	this->Disconnect( ID_SAVE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuFileSaveClick ) );
	this->Disconnect( ID_QUIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuFileQuitClick ) );
	this->Disconnect( ID_ADD, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesAddClick ) );
	this->Disconnect( ID_MODIFY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesEditClick ) );
	this->Disconnect( ID_DELETE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditGaugesDeleteClick ) );
	this->Disconnect( ID_CONFIGURE_DATA_ACQUISITION, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditConfigureDataAcquisitionClick ) );
	this->Disconnect( ID_START_DATA_ACQUISITION, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( frmMain::OnMnuEditStartDataAcquisitionClick ) );
	
}

dlgGaugeConfiguration::dlgGaugeConfiguration( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer1->SetFlexibleDirection( wxBOTH );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	lblGaugeName = new wxStaticText( this, wxID_ANY, wxT("Gauge Name:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblGaugeName->Wrap( -1 );
	fgSizer1->Add( lblGaugeName, 0, wxALL, 5 );
	
	txtGaugeName = new wxTextCtrl( this, wxID_ANY, wxT("Gauge XY"), wxDefaultPosition, wxDefaultSize, 0 );
	fgSizer1->Add( txtGaugeName, 0, wxALL, 5 );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Gauge Connection:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	fgSizer1->Add( m_staticText2, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	rdbCOM = new wxRadioButton( this, wxID_ANY, wxT("COM"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer12->Add( rdbCOM, 0, wxALL, 5 );
	
	wxArrayString drpCOMChoices;
	drpCOM = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, drpCOMChoices, 0 );
	drpCOM->SetSelection( 0 );
	bSizer12->Add( drpCOM, 0, wxALL, 5 );
	
	
	bSizer7->Add( bSizer12, 1, wxEXPAND, 5 );
	
	rdbCamera = new wxRadioButton( this, wxID_ANY, wxT("Identification via camera"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( rdbCamera, 0, wxALL, 5 );
	
	rdbTCPIP = new wxRadioButton( this, wxID_ANY, wxT("TCP/IP"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( rdbTCPIP, 0, wxALL, 5 );
	
	
	fgSizer1->Add( bSizer7, 1, wxEXPAND, 5 );
	
	
	bSizer5->Add( fgSizer1, 1, wxEXPAND, 5 );
	
	m_panel1 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer5->Add( m_panel1, 1, wxEXPAND | wxALL, 5 );
	
	
	bSizer5->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer6->Add( 0, 0, 1, wxEXPAND, 5 );
	
	btnCancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( btnCancel, 0, wxALL, 5 );
	
	btnOK = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( btnOK, 0, wxALL, 5 );
	
	
	bSizer5->Add( bSizer6, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( bSizer5 );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	btnCancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgGaugeConfiguration::OnBtnCancelClick ), NULL, this );
	btnOK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgGaugeConfiguration::OnBtnOKClick ), NULL, this );
}

dlgGaugeConfiguration::~dlgGaugeConfiguration()
{
	// Disconnect Events
	btnCancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgGaugeConfiguration::OnBtnCancelClick ), NULL, this );
	btnOK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgGaugeConfiguration::OnBtnOKClick ), NULL, this );
	
}

pnlUSB::pnlUSB( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
}

pnlUSB::~pnlUSB()
{
}

dlgDataAcquisitionConfiguration::dlgDataAcquisitionConfiguration( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer1->SetFlexibleDirection( wxBOTH );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	lblGaugeName = new wxStaticText( this, wxID_ANY, wxT("Interval:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblGaugeName->Wrap( -1 );
	fgSizer1->Add( lblGaugeName, 0, wxALL, 5 );
	
	wxFlexGridSizer* fgSizer8;
	fgSizer8 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer8->SetFlexibleDirection( wxBOTH );
	fgSizer8->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	numIntervalHours = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	fgSizer8->Add( numIntervalHours, 0, wxALL, 5 );
	
	m_staticText20 = new wxStaticText( this, wxID_ANY, wxT("Hours"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText20->Wrap( -1 );
	fgSizer8->Add( m_staticText20, 0, wxALL, 5 );
	
	numIntervalMinutes = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 60, 0 );
	fgSizer8->Add( numIntervalMinutes, 0, wxALL, 5 );
	
	m_staticText21 = new wxStaticText( this, wxID_ANY, wxT("Minutes"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	fgSizer8->Add( m_staticText21, 0, wxALL, 5 );
	
	numIntervalSeconds = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 60, 0 );
	fgSizer8->Add( numIntervalSeconds, 0, wxALL, 5 );
	
	m_staticText22 = new wxStaticText( this, wxID_ANY, wxT("Seconds"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText22->Wrap( -1 );
	fgSizer8->Add( m_staticText22, 0, wxALL, 5 );
	
	
	fgSizer1->Add( fgSizer8, 1, wxEXPAND, 5 );
	
	lblNumIntervals = new wxStaticText( this, wxID_ANY, wxT("Number of intervals:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblNumIntervals->Wrap( -1 );
	fgSizer1->Add( lblNumIntervals, 0, wxALL, 5 );
	
	numIntervals = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 2, 1000, 10 );
	fgSizer1->Add( numIntervals, 0, wxALL, 5 );
	
	m_staticText16 = new wxStaticText( this, wxID_ANY, wxT("Total Duration:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText16->Wrap( -1 );
	fgSizer1->Add( m_staticText16, 0, wxALL, 5 );
	
	lblDuration = new wxStaticText( this, wxID_ANY, wxT("---"), wxDefaultPosition, wxDefaultSize, 0 );
	lblDuration->Wrap( -1 );
	fgSizer1->Add( lblDuration, 0, wxALL, 5 );
	
	m_staticText18 = new wxStaticText( this, wxID_ANY, wxT("File Name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText18->Wrap( -1 );
	fgSizer1->Add( m_staticText18, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxVERTICAL );
	
	lblFilePath = new wxStaticText( this, wxID_ANY, wxT("--"), wxDefaultPosition, wxDefaultSize, 0 );
	lblFilePath->Wrap( -1 );
	bSizer13->Add( lblFilePath, 0, wxALL, 5 );
	
	btnSelectFilePath = new wxButton( this, wxID_ANY, wxT("..."), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer13->Add( btnSelectFilePath, 0, wxALL, 5 );
	
	
	fgSizer1->Add( bSizer13, 1, wxEXPAND, 5 );
	
	lblFileTypeDescription = new wxStaticText( this, wxID_ANY, wxT("File Type:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblFileTypeDescription->Wrap( -1 );
	fgSizer1->Add( lblFileTypeDescription, 0, wxALL, 5 );
	
	lblFileType = new wxStaticText( this, wxID_ANY, wxT("Excel Text File (.???)"), wxDefaultPosition, wxDefaultSize, 0 );
	lblFileType->Wrap( -1 );
	fgSizer1->Add( lblFileType, 0, wxALL, 5 );
	
	
	bSizer5->Add( fgSizer1, 1, wxEXPAND, 5 );
	
	
	bSizer5->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer6->Add( 0, 0, 1, wxEXPAND, 5 );
	
	btnCancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( btnCancel, 0, wxALL, 5 );
	
	btnOK = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( btnOK, 0, wxALL, 5 );
	
	
	bSizer5->Add( bSizer6, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( bSizer5 );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	btnSelectFilePath->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnSelectFilePathClick ), NULL, this );
	btnCancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnCancelClick ), NULL, this );
	btnOK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnOKClick ), NULL, this );
}

dlgDataAcquisitionConfiguration::~dlgDataAcquisitionConfiguration()
{
	// Disconnect Events
	btnSelectFilePath->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnSelectFilePathClick ), NULL, this );
	btnCancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnCancelClick ), NULL, this );
	btnOK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisitionConfiguration::OnBtnOKClick ), NULL, this );
	
}

dlgDataAcquisition::dlgDataAcquisition( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer1->SetFlexibleDirection( wxBOTH );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	lblDataAcquistionStatus = new wxStaticText( this, wxID_ANY, wxT("Interval:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblDataAcquistionStatus->Wrap( -1 );
	fgSizer1->Add( lblDataAcquistionStatus, 0, wxALL, 5 );
	
	lblDataAcquisitionStatusValue = new wxStaticText( this, wxID_ANY, wxT("In Progress...."), wxDefaultPosition, wxDefaultSize, 0 );
	lblDataAcquisitionStatusValue->Wrap( -1 );
	fgSizer1->Add( lblDataAcquisitionStatusValue, 0, wxALL, 5 );
	
	lblNumOfDataSets = new wxStaticText( this, wxID_ANY, wxT("Number of acquired data sets:"), wxDefaultPosition, wxDefaultSize, 0 );
	lblNumOfDataSets->Wrap( -1 );
	fgSizer1->Add( lblNumOfDataSets, 0, wxALL, 5 );
	
	m_staticText42 = new wxStaticText( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText42->Wrap( -1 );
	fgSizer1->Add( m_staticText42, 0, wxALL, 5 );
	
	m_staticText16 = new wxStaticText( this, wxID_ANY, wxT("Duration:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText16->Wrap( -1 );
	fgSizer1->Add( m_staticText16, 0, wxALL, 5 );
	
	lblDuration = new wxStaticText( this, wxID_ANY, wxT("---"), wxDefaultPosition, wxDefaultSize, 0 );
	lblDuration->Wrap( -1 );
	fgSizer1->Add( lblDuration, 0, wxALL, 5 );
	
	m_staticText18 = new wxStaticText( this, wxID_ANY, wxT("Gauge status:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText18->Wrap( -1 );
	fgSizer1->Add( m_staticText18, 0, wxALL, 5 );
	
	m_staticText19 = new wxStaticText( this, wxID_ANY, wxT("10/10 online."), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText19->Wrap( -1 );
	fgSizer1->Add( m_staticText19, 0, wxALL, 5 );
	
	
	bSizer5->Add( fgSizer1, 1, wxEXPAND, 5 );
	
	
	bSizer5->Add( 0, 0, 1, wxEXPAND, 5 );
	
	prgProgress = new wxGauge( this, wxID_ANY, 100, wxDefaultPosition, wxDefaultSize, wxGA_HORIZONTAL );
	prgProgress->SetValue( 0 ); 
	bSizer5->Add( prgProgress, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer6->Add( 0, 0, 1, wxEXPAND, 5 );
	
	btnStop = new wxButton( this, wxID_ANY, wxT("Stop Acquistion"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( btnStop, 0, wxALL, 5 );
	
	
	bSizer5->Add( bSizer6, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( bSizer5 );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( dlgDataAcquisition::OnClose ) );
	btnStop->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisition::OnBtnStopAcquisitionClick ), NULL, this );
}

dlgDataAcquisition::~dlgDataAcquisition()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( dlgDataAcquisition::OnClose ) );
	btnStop->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( dlgDataAcquisition::OnBtnStopAcquisitionClick ), NULL, this );
	
}
