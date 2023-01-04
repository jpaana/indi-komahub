/*
 KomaHub INDI driver

 Copyright 2018-2023 Jarno Paananen

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <memory>
#include <cstring>
#include <deque>

#include "config.h"

#include "komahub.h"

static class Loader
{
        std::deque<std::unique_ptr<KOMAHUB>> hubs;
    public:
        Loader()
        {
            hid_device_info *devices = hid_enumerate(KOMAHUB::vendorID, KOMAHUB::productID);
            int hubCount = 0;

            hid_device_info *current = devices;
            while (current)
            {
                // The hub shows up as to interfaces, only use 0
                if (current->interface_number == 0)
                {
                    hid_device *dev = hid_open_path(current->path);
                    if (dev)
                    {
                        std::string name = "KomaHub";
                        hubCount++;
                        if (hubCount > 1)
                            name += " " + std::to_string(hubCount);
                        hubs.push_back(std::unique_ptr<KOMAHUB>(new KOMAHUB(dev, name.c_str())));
                    }
                }
                current = current->next;
            }
            hid_free_enumeration(devices);
        }
} loader;

KOMAHUB::KOMAHUB(hid_device *device, const char* name)
{
    this->device     = device;
    snprintf(this->name, 32, "%s", name);
    setDeviceName(this->name);

    setVersion(KOMAHUB_VERSION_MAJOR, KOMAHUB_VERSION_MINOR);
}

KOMAHUB::~KOMAHUB()
{
    hid_close(device);
}

const char *KOMAHUB::getDefaultName()
{
    return "KomaHub";
}

bool KOMAHUB::readFactorySettings()
{
    if (!device)
    {
        LOG_ERROR("KomaHub not connected.");
        return false;
    }
    unsigned char buf[2] = { magic, GETFACTORYSETTINGS };
    GetFactorySettingsResponse settings;

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("ReadFactorySettings: hid_write( { %d, %d } ) -> %d", buf[0], buf[1], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    rc = hid_read(device, reinterpret_cast<unsigned char *>(&settings), sizeof(GetFactorySettingsResponse));
    if (rc != sizeof(GetFactorySettingsResponse))
    {
        LOG_ERROR("Failed to read from KomaHub.");
        return false;
    }

    char str[32];
    snprintf(str, 32, "%d.%d", settings.boardRevision >> 4, settings.boardRevision & 0xf);
    VersionsTP[0].setText(str);

    snprintf(str, 32, "%d.%d", settings.firmwareVersion >> 8, settings.firmwareVersion & 0xff);
    VersionsTP[1].setText(str);

    snprintf(str, 32, "%d", settings.serialNumber);
    VersionsTP[2].setText(str);

    VersionsTP.setState(IPS_OK);
    VersionsTP.apply();
    return true;
}

bool KOMAHUB::readStatus()
{
    if (!device)
    {
        LOG_ERROR("KomaHub not connected.");
        return false;
    }
    unsigned char buf[2] = { magic, GETSTATUS };
    GetStatusResponse status;

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("ReadStatus: hid_write( { %d, %d } ) -> %d", buf[0], buf[1], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    rc = hid_read(device, reinterpret_cast<unsigned char *>(&status), sizeof(GetStatusResponse));
    if (rc != sizeof(GetStatusResponse))
    {
        LOG_ERROR("Failed to read from KomaHub.");
        return false;
    }

    InputVoltageNP[0].setValue(status.inputVoltage / 10.0);
    InputVoltageNP.setState((InputVoltageNP[0].getValue() > 11.0 && InputVoltageNP[0].getValue() < 14) ? IPS_OK : IPS_ALERT);
    InputVoltageNP.apply();

    for (unsigned int p = 0; p < numPorts; ++p)
    {
        Ports[p].CurrentNP[0].setValue(status.outputPower[p] / 10.0);
        Ports[p].CurrentNP.setState((status.fuseIsBlownBits >> p) & 1 ? IPS_ALERT : Ports[p].ModeSP[0].getState() == ISS_ON ?
                                    IPS_IDLE : IPS_OK);
        Ports[p].CurrentNP.apply();

        Ports[p].DutyCycleNP[0].setValue(status.pwmPercentages[p]);
        Ports[p].DutyCycleNP.setState(Ports[p].ModeSP[2].getState() == ISS_ON ? IPS_OK : IPS_IDLE);
        Ports[p].DutyCycleNP.apply();

        Ports[p].EnableSP.reset();
        Ports[p].EnableSP[(status.relayIsOpenBits >> p) & 1].setState(ISS_ON);
        Ports[p].EnableSP.setState((status.fuseIsBlownBits >> p) & 1 ? IPS_ALERT : (status.relayIsOpenBits >> p) & 1 ? IPS_OK :
                                   IPS_IDLE);
        Ports[p].EnableSP.apply();
    }

    numTemperatureProbes  = status.numberOfTemperatureProbes;
    pthPresent            = (status.pthpresent == 1);
    skyqualityPresent     = (status.skyqualitypresent == 1);
    skytemperaturePresent = (status.skytemperaturepresent == 1);

    if (numTemperatureProbes > 0)
    {
        for (unsigned t = 0; t < numTemperatureProbes; ++t)
        {
            TemperaturesNP[t].setValue(status.temperatureProbes[t] / 10);
        }
        TemperaturesNP.setState(IPS_OK);
        TemperaturesNP.apply();
    }

    if (pthPresent)
    {
        TemperatureNP[0].setValue(status.temperature / 10);
        TemperatureNP.setState(IPS_OK);
        TemperatureNP.apply();

        HumidityNP[0].setValue(status.humidity);
        HumidityNP.setState(IPS_OK);
        HumidityNP.apply();

        PressureNP[0].setValue(status.pressure / 10);
        PressureNP.setState(IPS_OK);
        PressureNP.apply();

        DewpointNP[0].setValue(status.dewpoint / 10);
        DewpointNP.setState(IPS_OK);
        DewpointNP.apply();
    }

    if (skyqualityPresent)
    {
        SkyQualityNP[0].setValue(status.skyquality / 10);
        SkyQualityNP[1].setValue(status.skyqualityfreq / 10);
        SkyQualityNP.setState(IPS_OK);
        SkyQualityNP.apply();
    }

    if (skytemperaturePresent)
    {
        SkyTemperatureNP[0].setValue(status.skytemperature / 10);
        SkyTemperatureNP[1].setValue(status.skyambienttemperature / 10);
        SkyTemperatureNP.setState(IPS_OK);
        SkyTemperatureNP.apply();
    }
    return true;
}

bool KOMAHUB::readOutputSettings()
{
    if (!device)
    {
        LOG_ERROR("KomaHub not connected.");
        return false;
    }

    for (unsigned char p = 0; p < numPorts; ++p)
    {
        unsigned char buf[3]              = { magic, GETOUTPUTSETTINGS, p };
        GetOutputSettingsResponse *status = &Ports[p].settings;

        int rc = hid_write(device, buf, sizeof(buf));
        LOGF_DEBUG("ReadOutputSettings: hid_write( { %d, %d, %d } ) -> %d", buf[0], buf[1], buf[2], rc);
        if (rc != sizeof(buf))
        {
            LOG_ERROR("Failed to write to KomaHub");
            return false;
        }
        rc = hid_read(device, reinterpret_cast<unsigned char *>(status), sizeof(GetOutputSettingsResponse));
        if (rc != sizeof(GetOutputSettingsResponse))
        {
            LOG_ERROR("Failed to read from KomaHub.");
            return false;
        }

        char str[MAXINDILABEL];
        snprintf(str, MAXINDILABEL, "%s enable", status->name);
        Ports[p].EnableSP.setLabel(str);
        Ports[p].EnableSP.apply();

        snprintf(str, MAXINDILABEL, "%s fuse", status->name);
        Ports[p].FuseNP.setLabel(str);
        Ports[p].FuseNP.apply();
        snprintf(str, MAXINDILABEL, "%s type", status->name);
        Ports[p].ModeSP.setLabel(str);
        Ports[p].ModeSP.apply();

        snprintf(str, MAXINDILABEL, "%s current", status->name);
        Ports[p].CurrentNP.setLabel(str);
        Ports[p].CurrentNP.apply();

        snprintf(str, MAXINDILABEL, "%s PWM", status->name);
        Ports[p].DutyCycleNP.setLabel(str);
        Ports[p].DutyCycleNP.apply();

        Ports[p].NameTP[0].setText(status->name);
        Ports[p].NameTP.setState(IPS_OK);
        Ports[p].NameTP.apply();

        Ports[p].FuseNP[0].setValue(status->fuseCurrent / 10.0);
        Ports[p].FuseNP.setState(IPS_OK);
        Ports[p].FuseNP.apply();

        if (status->type <= OUTPUT_PWM)
        {
            Ports[p].ModeSP.reset();
            Ports[p].ModeSP[status->type].setState(ISS_ON);
            Ports[p].ModeSP.setState(IPS_OK);
            Ports[p].ModeSP.apply();
        }
    }

    return true;
}

bool KOMAHUB::setRelay(unsigned char port, bool enabled)
{
    unsigned char buf[4] = { magic, SETRELAY, port, enabled };

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("SetRelay: hid_write( { %d, %d, %d, %d } ) -> %d", buf[0], buf[1], buf[2], buf[3], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    return true;
}

bool KOMAHUB::setPwmDuty(unsigned char port, unsigned char pwm)
{
    unsigned char buf[4] = { magic, SETPWMDUTY, port, pwm };

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("SetPWM: hid_write( { %d, %d, %d, %d } ) -> %d", buf[0], buf[1], buf[2], buf[3], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    return true;
}

bool KOMAHUB::resetFuse(unsigned char port)
{
    unsigned char buf[4] = { magic, RESETFUSE, port };

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("ResetFuse: hid_write( { %d, %d, %d } ) -> %d", buf[0], buf[1], buf[2], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    return true;
}

bool KOMAHUB::configureOutput(unsigned char port)
{
    unsigned char buf[2 + sizeof(ConfigureOutputCommand)] = { magic, CONFIGUREOUTPUT };
    ConfigureOutputCommand *cmd                           = reinterpret_cast<ConfigureOutputCommand *>(&buf[2]);

    GetOutputSettingsResponse &settings = Ports[port].settings;

    cmd->outputNumber = port;
    memcpy(cmd->name, settings.name, sizeof(settings.name));
    cmd->fuseCurrent = settings.fuseCurrent;
    cmd->outputType  = settings.type;

    int rc = hid_write(device, buf, sizeof(buf));
    LOGF_DEBUG("ConfigureOutput: hid_write( { %d, %d } ) -> %d", buf[0], buf[1], rc);
    if (rc != sizeof(buf))
    {
        LOG_ERROR("Failed to write to KomaHub");
        return false;
    }
    return true;
}

bool KOMAHUB::initProperties()
{
    // Init parent properties first
    INDI::DefaultDevice::initProperties();

    addConfigurationControl();
    addDebugControl();
    setDefaultPollingPeriod(10000);
    addPollPeriodControl();

    VersionsTP[0].fill("BOARD", "Board", "");
    VersionsTP[1].fill("FIRMWARE", "Firmware", "");
    VersionsTP[2].fill("SERIAL", "Serial", "");
    VersionsTP.fill(getDeviceName(), "VERSIONS", "Versions", INFO_TAB, IP_RO, 60, IPS_IDLE);

    InputVoltageNP[0].fill("VALUE", "V", "%2.1f", 0.0, 15.0, 1.0, 0.0);
    InputVoltageNP.fill(getDeviceName(), "INPUT_VOLTAGE", "Input voltage",
                        MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    for (unsigned int p = 0; p < numPorts; ++p)
    {
        char str[32];
        char label[32];

        Ports[p].NameTP[0].fill("NAME", "Name", "");
        snprintf(str, 32, "OUTPUT_NAME_%d", p);
        snprintf(label, 32, "Output %d name", p + 1);
        Ports[p].NameTP.fill(getDeviceName(), str, label, OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

        Ports[p].EnableSP[0].fill("OFF", "Off", ISS_OFF);
        Ports[p].EnableSP[1].fill("ON", "On", ISS_OFF);
        Ports[p].EnableSP[2].fill("RESETFUSE", "Reset fuse", ISS_OFF);
        snprintf(str, 32, "OUTPUT_ENABLE_%d", p);
        snprintf(label, 32, "Output %d enable", p + 1);
        Ports[p].EnableSP.fill(getDeviceName(), str, label, MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

        Ports[p].ModeSP[0].fill("OFF", "Off", ISS_OFF);
        Ports[p].ModeSP[1].fill("DC", "DC", ISS_OFF);
        Ports[p].ModeSP[2].fill("PWM", "PWM", ISS_OFF);
        snprintf(str, 32, "OUTPUT_MODE_%d", p);
        snprintf(label, 32, "Output %d mode", p + 1);
        Ports[p].ModeSP.fill(getDeviceName(), str, label, OPTIONS_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

        Ports[p].FuseNP[0].fill("VALUE", "A", "%3.0f", 0.0, 10.0, 0.1, 0.0);
        snprintf(str, 32, "OUTPUT_FUSE_%d", p);
        snprintf(label, 32, "Output %d fuse", p + 1);
        Ports[p].FuseNP.fill(getDeviceName(), str, label, OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

        Ports[p].CurrentNP[0].fill("VALUE", "A", "%2.1f", 0.0, 15.0, 1.0, 0.0);
        snprintf(str, 32, "OUTPUT_CURRENT_%d", p);
        snprintf(label, 32, "Output %d current", p + 1);
        Ports[p].CurrentNP.fill(getDeviceName(), str, label, MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

        Ports[p].DutyCycleNP[0].fill("POWER", "%", "%3.0f", 0.0, 100.0, 1.0, 0.0);
        snprintf(str, 32, "OUTPUT_PWM_%d", p);
        snprintf(label, 32, "Output %d PWM", p + 1);
        Ports[p].DutyCycleNP.fill(getDeviceName(), str, label, MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);
    }

    // DS18B20 probes, vector initialized later
    for (unsigned int t = 0; t < 4; ++t)
    {
        char str[32];
        char label[32];

        snprintf(str, 32, "TEMP_%d", t);
        snprintf(label, 32, "Temperature %d", t + 1);
        TemperaturesNP[t].fill(str, label, "%3.2f", -100.0, 100.0, 0, 0.0);
    }

    // BME280
    HumidityNP[0].fill("VALUE", "%", "%3.0f", 0, 100, 0, 0);
    HumidityNP.fill(getDeviceName(), "HUMIDITY", "Humidity", INFO_TAB, IP_RO, 60, IPS_IDLE);

    PressureNP[0].fill("VALUE", "hPa", "%4.0f", 900, 1100, 0, 0);
    PressureNP.fill(getDeviceName(), "PRESSURE", "Pressure", INFO_TAB, IP_RO, 60, IPS_IDLE);

    TemperatureNP[0].fill("VALUE", "C", "%3.0f", -100, 100, 0, 0);
    TemperatureNP.fill(getDeviceName(), "TEMPERATURE", "Temperature", INFO_TAB, IP_RO, 60, IPS_IDLE);

    DewpointNP[0].fill("VALUE", "C", "%3.0f", -100, 100, 0, 0);
    DewpointNP.fill(getDeviceName(), "DEWPOINT", "Dewpoint", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // TSL237
    SkyQualityNP[0].fill("SKY_BRIGHTNESS", "Mag/Arcsec^2", "%3.2f", -20, 30, 0, 0); // Property snooped by INDICCD
    SkyQualityNP[1].fill("SENSOR_FREQUENCY", "Hz", "%6.2f", 0, 5000000, 0, 0);
    SkyQualityNP.fill(getDeviceName(), "SKY_QUALITY", "Sky Quality", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // PLX90614
    SkyTemperatureNP[0].fill("SKY_TEMPERATURE", "C", "%3.2f", -100, 100, 0, 0);
    SkyTemperatureNP[1].fill("AMBIENT", "C", "%3.2f", -100, 100, 0, 0);
    SkyTemperatureNP.fill(getDeviceName(), "SKY_TEMPERATURE", "Sky Temperature", INFO_TAB, IP_RO, 60, IPS_IDLE);

    return true;
}

void KOMAHUB::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);
}

bool KOMAHUB::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        readFactorySettings();
        readOutputSettings();
        readStatus();

        defineProperty(VersionsTP);
        defineProperty(InputVoltageNP);

        for (unsigned int p = 0; p < numPorts; ++p)
        {
            defineProperty(Ports[p].NameTP);
            defineProperty(Ports[p].EnableSP);
            defineProperty(Ports[p].ModeSP);
            defineProperty(Ports[p].FuseNP);
            defineProperty(Ports[p].DutyCycleNP);
            defineProperty(Ports[p].CurrentNP);
        }

        // Configure sensors according to status
        // DS18B20 temperature probes
        if (numTemperatureProbes > 0)
        {
            TemperaturesNP.resize(numTemperatureProbes);
            TemperaturesNP.fill(getDeviceName(), "TEMPERATURES", "Temperatures", INFO_TAB, IP_RO, 60, IPS_IDLE);

            defineProperty(TemperaturesNP);
        }

        // BM280 pressure-temperature-humidity sensor
        if (pthPresent)
        {
            defineProperty(PressureNP);
            defineProperty(HumidityNP);
            defineProperty(TemperatureNP);
            defineProperty(DewpointNP);
        }

        // TSL237 sky quality sensor
        if (skyqualityPresent)
        {
            defineProperty(SkyQualityNP);
        }

        // PLX90614 sky temperature sensor
        if (skytemperaturePresent)
        {
            defineProperty(SkyTemperatureNP);
        }
        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(VersionsTP);
        deleteProperty(InputVoltageNP);
        for (unsigned int p = 0; p < numPorts; ++p)
        {
            deleteProperty(Ports[p].NameTP);
            deleteProperty(Ports[p].EnableSP);
            deleteProperty(Ports[p].ModeSP);
            deleteProperty(Ports[p].FuseNP);
            deleteProperty(Ports[p].DutyCycleNP);
            deleteProperty(Ports[p].CurrentNP);
        }

        if (numTemperatureProbes > 0)
        {
            deleteProperty(TemperaturesNP);
        }
        if (pthPresent)
        {
            deleteProperty(PressureNP);
            deleteProperty(HumidityNP);
            deleteProperty(TemperatureNP);
            deleteProperty(DewpointNP);
        }
        // TSL237 sky quality sensor
        if (skyqualityPresent)
        {
            deleteProperty(SkyQualityNP);
        }

        // PLX90614 sky temperature sensor
        if (skytemperaturePresent)
        {
            deleteProperty(SkyTemperatureNP);
        }
    }
    return true;
}

bool KOMAHUB::Connect()
{
    LOG_INFO("Attempting to find KomaHubs...");
    return true;
}

bool KOMAHUB::Disconnect()
{
    LOG_INFO("KomaHub is offline.");
    return true;
}

void KOMAHUB::TimerHit()
{
    if (isConnected() == false)
        return; //  No need to reset timer if we are not connected anymore

    readStatus();

    SetTimer(getCurrentPollingPeriod());
}

bool KOMAHUB::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev == nullptr || strcmp(dev, getDeviceName()) != 0)
    {
        return false;
    }
    for (unsigned int p = 0; p < numPorts; ++p)
    {
        if (Ports[p].NameTP.isNameMatch(name))
        {
            Ports[p].NameTP.update(texts, names, n);
            if (strlen(Ports[p].NameTP[0].getText()) >= 16)
            {
                Ports[p].NameTP.setState(IPS_ALERT);
                Ports[p].NameTP.apply();
                LOG_ERROR("Too long port name, maximum of 15 characters supported!");
                return false;
            }

            strncpy(Ports[p].settings.name, Ports[p].NameTP[0].getText(), sizeof(Ports[p].settings.name));
            if (configureOutput(p))
            {
                if (readOutputSettings())
                {
                    Ports[p].NameTP.setState(IPS_OK);
                    Ports[p].NameTP.apply();
                    return true;
                }
            }
            return false;
        }
    }
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool KOMAHUB::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev == nullptr || strcmp(dev, getDeviceName()) != 0)
    {
        return false;
    }
    for (unsigned int p = 0; p < numPorts; ++p)
    {
        if (Ports[p].DutyCycleNP.isNameMatch(name))
        {
            Ports[p].DutyCycleNP.update(values, names, n);

            if (setPwmDuty(p, static_cast<unsigned char>(Ports[p].DutyCycleNP[0].getValue())))
            {
                Ports[p].DutyCycleNP.setState(IPS_OK);
                Ports[p].DutyCycleNP.apply();
                readStatus();
                return true;
            }
            return false;
        }
        if (Ports[p].FuseNP.isNameMatch(name))
        {
            Ports[p].FuseNP.update(values, names, n);
            Ports[p].settings.fuseCurrent = static_cast<unsigned char>(Ports[p].FuseNP[0].getValue() * 10.0);
            if (configureOutput(p))
            {
                if (readOutputSettings())
                {
                    Ports[p].FuseNP.setState(IPS_OK);
                    Ports[p].FuseNP.apply();
                    return true;
                }
            }
            return false;
        }
    }

    return DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool KOMAHUB::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev == nullptr || strcmp(dev, getDeviceName()) != 0)
    {
        return false;
    }

    for (unsigned int p = 0; p < numPorts; ++p)
    {
        if (Ports[p].ModeSP.isNameMatch(name))
        {
            Ports[p].ModeSP.update(states, names, n);
            int mode = Ports[p].ModeSP.findOnSwitchIndex();
            if (mode >= OUTPUT_OFF && mode <= OUTPUT_PWM)
            {
                Ports[p].settings.type = static_cast<unsigned char>(mode);
                if (configureOutput(p))
                {
                    if (readOutputSettings())
                    {
                        Ports[p].ModeSP.setState(IPS_OK);
                        Ports[p].ModeSP.apply();
                        return true;
                    }
                }
            }
            return false;
        }
        if (Ports[p].EnableSP.isNameMatch(name))
        {
            Ports[p].EnableSP.update(states, names, n);
            int mode = Ports[p].EnableSP.findOnSwitchIndex();
            switch (mode)
            {
                case 0:
                    if (setRelay(p, false))
                    {
                        Ports[p].EnableSP.setState(IPS_OK);
                        Ports[p].EnableSP.apply();
                        readStatus();
                        return true;
                    }
                    break;
                case 1:
                    if (setRelay(p, true))
                    {
                        Ports[p].EnableSP.setState(IPS_OK);
                        Ports[p].EnableSP.apply();
                        readStatus();
                        return true;
                    }
                    break;
                case 2:
                    if (resetFuse(p))
                    {
                        Ports[p].EnableSP.setState(IPS_OK);
                        Ports[p].EnableSP.apply();
                        readStatus();
                        return true;
                    }
                    break;
            }
        }
    }
    return DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}
