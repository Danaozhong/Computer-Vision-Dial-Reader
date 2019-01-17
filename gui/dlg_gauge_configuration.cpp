#include "base\gauges\gauge.h"
#include "base/gauges/helios/gauge_helios.h"
#include "base/interfaces/com/interface_com.h"
#include "dlg_gauge_configuration.h"

DialogGaugeConfiguration::DialogGaugeConfiguration(wxWindow* parent, DialogGaugeConfigurationParent* data_parent)
:
dlgGaugeConfiguration(parent), dataParent(data_parent)
{}

void DialogGaugeConfiguration::OnBtnCancelClick( wxCommandEvent& event )
{
	this->HideForm(false);
}

void DialogGaugeConfiguration::OnBtnOKClick( wxCommandEvent& event )
{
	this->HideForm(true);
}

grBase::GaugeType DialogGaugeConfiguration::TypeFromRadioButtons() const
{
	if (true == this->rdbCOM->GetValue())
	{
		return grBase::GAUGE_TYPE_HELIOS_COM;
	}
	else
	{
		return grBase::GAUGE_TYPE_HELIOS_COM;
	}


}

int DialogGaugeConfiguration::ShowForm(const std::shared_ptr<grBase::Gauge>& data)
{
	assert(false == this->IsVisible());

	//create a clone of the gauge data.
	this->data = data->Clone();
	this->Show();

	// populate the list with all COM Ports
	this->drpCOM->Clear();
	auto listCOMPorts = grBase::InterfaceCOM::GetAllPorts();

	for (auto itr = listCOMPorts.begin(); itr != listCOMPorts.end(); itr++)
	{
		this->drpCOM->AppendString(*itr);
	}

	UpdateForm();
	return 0;
}

int DialogGaugeConfiguration::HideForm(bool submit)
{
	assert(this->IsVisible());
	UpdateData();
	this->dataParent->OnDialogGaugeConfigurationClose(submit, this->data, 0);
	this->Hide();
	return 0;
}

int DialogGaugeConfiguration::Process()
{
	return 0;
}

int DialogGaugeConfiguration::UpdateForm()
{
	// set the identifier
	this->txtGaugeName->SetValue(this->data->GetIdentifier());

	// set the type-specific things 
	switch (this->data->GetType())
	{
		case grBase::GAUGE_TYPE_CAMERA:
			this->rdbCamera->SetValue(true);
			break;
		case grBase::GAUGE_TYPE_HELIOS_COM:
		{
			this->rdbCOM->SetValue(true);

			// set the right COM port.
			auto heliosGauge = std::dynamic_pointer_cast<grBase::GaugeHeliosCOM>(this->data);
			assert(nullptr != heliosGauge);

			int index = this->drpCOM->FindString(heliosGauge->GetCOMPort());
			if (wxNOT_FOUND == index)
			{
				assert(false);
			}
			else
			{
				this->drpCOM->SetSelection(index);
			}
			break;
		}
		case grBase::GAUGE_TYPE_TCPIP:
			this->rdbTCPIP->SetValue(true);
			break;
	}



	return 0;
}

int DialogGaugeConfiguration::UpdateData()
{
	// depending on the type, create a new object
	auto gaugeType = this->TypeFromRadioButtons();
	//this->data = std::shared_ptr<grBase::Gauge>(grBase::GaugeHelper::GaugeFactory(gaugeType));
	

	switch (gaugeType)
	{
	case grBase::GAUGE_TYPE_CAMERA:
		break;
	case grBase::GAUGE_TYPE_HELIOS_COM:
		{
			std::string COMPort = this->drpCOM->GetStringSelection();
			auto heliosGauge = std::shared_ptr<grBase::GaugeHeliosCOM>(new grBase::GaugeHeliosCOM(COMPort));
			this->data = heliosGauge;
			break;
		}
	case grBase::GAUGE_TYPE_TCPIP:
		break;
	}

	this->data->SetIdentifier(this->txtGaugeName->GetValue().ToStdString());

	return 0;
}