/*
 KomaHub INDI driver

 Copyright 2018 Jarno Paananen

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

#include "config.h"

#include "komahub.h"

#define MAX_DEVICES 4 /* Max device hubCount */

static int hubCount;
static KOMAHUB *hubs[MAX_DEVICES];

static void cleanup()
{
    for (int i = 0; i < hubCount; i++)
    {
        delete hubs[i];
    }
}

void ISInit()
{
    static bool isInit = false;

    if (!isInit)
    {
        hid_device_info *devices = hid_enumerate(KOMAHUB::vendorID, KOMAHUB::productID);
        hubCount                 = 0;

        hid_device_info *current = devices;
        while (current)
        {
            // The hub shows up as to interfaces, only use 0
            if (current->interface_number == 0)
            {
                hid_device *dev = hid_open_path(current->path);
                if (dev)
                {
                    hubs[hubCount++] = new KOMAHUB(dev);
                }
            }
            current = current->next;
        }
        hid_free_enumeration(devices);
        atexit(cleanup);
        isInit = true;
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();
    for (int i = 0; i < hubCount; i++)
    {
        KOMAHUB *hub = hubs[i];
        if (dev == NULL || !strcmp(dev, hub->name))
        {
            hub->ISGetProperties(dev);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    ISInit();
    for (int i = 0; i < hubCount; i++)
    {
        KOMAHUB *hub = hubs[i];
        if (dev == NULL || !strcmp(dev, hub->name))
        {
            hub->ISNewSwitch(dev, name, states, names, num);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < hubCount; i++)
    {
        KOMAHUB *hub = hubs[i];
        if (dev == NULL || !strcmp(dev, hub->name))
        {
            hub->ISNewText(dev, name, texts, names, num);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < hubCount; i++)
    {
        KOMAHUB *hub = hubs[i];
        if (dev == NULL || !strcmp(dev, hub->name))
        {
            hub->ISNewNumber(dev, name, values, names, num);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    ISInit();

    for (int i = 0; i < hubCount; i++)
    {
        KOMAHUB *hub = hubs[i];
        hub->ISSnoopDevice(root);
    }
}

KOMAHUB::KOMAHUB(hid_device *device)
{
    this->device     = device;
    const char *name = "KomaHub";
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
    IUSaveText(&VersionsT[0], str);

    snprintf(str, 32, "%d.%d", settings.firmwareVersion >> 8, settings.firmwareVersion & 0xff);
    IUSaveText(&VersionsT[1], str);

    snprintf(str, 32, "%d", settings.serialNumber);
    IUSaveText(&VersionsT[2], str);

    VersionsTP.s = IPS_OK;
    IDSetText(&VersionsTP, nullptr);
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

    InputVoltageN.value = status.inputVoltage / 10.0;
    InputVoltageNP.s    = (InputVoltageN.value > 11.0 && InputVoltageN.value < 14) ? IPS_OK : IPS_ALERT;
    IDSetNumber(&InputVoltageNP, nullptr);

    for (unsigned int p = 0; p < numPorts; p++)
    {
        Ports[p].CurrentN.value = status.outputPower[p] / 10.0;
        Ports[p].CurrentNP.s =
            (status.fuseIsBlownBits >> p) & 1 ? IPS_ALERT : Ports[p].ModeS[0].s == ISS_ON ? IPS_IDLE : IPS_OK;
        IDSetNumber(&Ports[p].CurrentNP, nullptr);

        Ports[p].DutyCycleN.value = status.pwmPercentages[p];
        Ports[p].DutyCycleNP.s    = Ports[p].ModeS[2].s == ISS_ON ? IPS_OK : IPS_IDLE;
        IDSetNumber(&Ports[p].DutyCycleNP, nullptr);

        IUResetSwitch(&Ports[p].EnableSP);
        Ports[p].EnableS[(status.relayIsOpenBits >> p) & 1].s = ISS_ON;
        Ports[p].EnableSP.s =
            (status.fuseIsBlownBits >> p) & 1 ? IPS_ALERT : (status.relayIsOpenBits >> p) & 1 ? IPS_OK : IPS_IDLE;
        IDSetSwitch(&Ports[p].EnableSP, nullptr);
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

    for (unsigned char p = 0; p < numPorts; p++)
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

        snprintf(Ports[p].EnableSP.label, MAXINDILABEL, "%s enable", status->name);
        IDSetSwitch(&Ports[p].EnableSP, nullptr);

        snprintf(Ports[p].FuseNP.label, MAXINDILABEL, "%s fuse", status->name);
        snprintf(Ports[p].ModeSP.label, MAXINDILABEL, "%s type", status->name);

        snprintf(Ports[p].CurrentNP.label, MAXINDILABEL, "%s current", status->name);
        IDSetNumber(&Ports[p].CurrentNP, nullptr);
        snprintf(Ports[p].DutyCycleNP.label, MAXINDILABEL, "%s PWM", status->name);
        IDSetNumber(&Ports[p].DutyCycleNP, nullptr);

        IUSaveText(&Ports[p].NameT, status->name);
        Ports[p].NameTP.s = IPS_OK;
        IDSetText(&Ports[p].NameTP, nullptr);

        Ports[p].FuseN.value = status->fuseCurrent / 10.0;
        Ports[p].FuseNP.s    = IPS_OK;
        IDSetNumber(&Ports[p].FuseNP, nullptr);

        if (status->type <= OUTPUT_PWM)
        {
            IUResetSwitch(&Ports[p].ModeSP);
            Ports[p].ModeS[status->type].s = ISS_ON;
            Ports[p].ModeSP.s              = IPS_OK;
            IDSetSwitch(&Ports[p].ModeSP, nullptr);
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

    IUFillText(&VersionsT[0], "BOARD", "Board", "");
    IUFillText(&VersionsT[1], "FIRMWARE", "Firmware", "");
    IUFillText(&VersionsT[2], "SERIAL", "Serial", "");
    IUFillTextVector(&VersionsTP, VersionsT, 3, getDeviceName(), "VERSIONS", "Versions", INFO_TAB, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&InputVoltageN, "VALUE", "V", "%2.1f", 0.0, 15.0, 1.0, 0.0);
    IUFillNumberVector(&InputVoltageNP, &InputVoltageN, 1, getDeviceName(), "INPUT_VOLTAGE", "Input voltage",
                       MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    for (unsigned int p = 0; p < numPorts; p++)
    {
        char str[32];
        char label[32];

        IUFillText(&Ports[p].NameT, "NAME", "Name", "");
        snprintf(str, 32, "OUTPUTNAME_%d", p);
        snprintf(label, 32, "Output %d name", p + 1);
        IUFillTextVector(&Ports[p].NameTP, &Ports[p].NameT, 1, getDeviceName(), str, label, OPTIONS_TAB, IP_RW, 60,
                         IPS_IDLE);

        IUFillSwitch(&Ports[p].EnableS[0], "OFF", "Off", ISS_OFF);
        IUFillSwitch(&Ports[p].EnableS[1], "ON", "On", ISS_OFF);
        IUFillSwitch(&Ports[p].EnableS[2], "RESETFUSE", "Reset fuse", ISS_OFF);
        snprintf(str, 32, "OUTPUTENABLE_%d", p);
        snprintf(label, 32, "Output %d enable", p + 1);
        IUFillSwitchVector(&Ports[p].EnableSP, Ports[p].EnableS, 3, getDeviceName(), str, label, MAIN_CONTROL_TAB,
                           IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

        IUFillSwitch(&Ports[p].ModeS[0], "OFF", "Off", ISS_OFF);
        IUFillSwitch(&Ports[p].ModeS[1], "DC", "DC", ISS_OFF);
        IUFillSwitch(&Ports[p].ModeS[2], "PWM", "PWM", ISS_OFF);
        snprintf(str, 32, "OUTPUTMODE_%d", p);
        snprintf(label, 32, "Output %d mode", p + 1);
        IUFillSwitchVector(&Ports[p].ModeSP, Ports[p].ModeS, 3, getDeviceName(), str, label, OPTIONS_TAB, IP_RW,
                           ISR_ATMOST1, 0, IPS_IDLE);

        IUFillNumber(&Ports[p].FuseN, "VALUE", "A", "%3.0f", 0.0, 10.0, 0.1, 0.0);
        snprintf(str, 32, "OUTPUTFUSE_%d", p);
        snprintf(label, 32, "Output %d fuse", p + 1);
        IUFillNumberVector(&Ports[p].FuseNP, &Ports[p].FuseN, 1, getDeviceName(), str, label, OPTIONS_TAB, IP_RW, 60,
                           IPS_IDLE);

        IUFillNumber(&Ports[p].CurrentN, "VALUE", "A", "%2.1f", 0.0, 15.0, 1.0, 0.0);
        snprintf(str, 32, "OUTPUTCURRENT_%d", p);
        snprintf(label, 32, "Output %d current", p + 1);
        IUFillNumberVector(&Ports[p].CurrentNP, &Ports[p].CurrentN, 1, getDeviceName(), str, label, MAIN_CONTROL_TAB,
                           IP_RO, 60, IPS_IDLE);

        IUFillNumber(&Ports[p].DutyCycleN, "POWER", "%", "%3.0f", 0.0, 100.0, 1.0, 0.0);
        snprintf(str, 32, "OUTPUTPWM_%d", p);
        snprintf(label, 32, "Output %d PWM", p + 1);
        IUFillNumberVector(&Ports[p].DutyCycleNP, &Ports[p].DutyCycleN, 1, getDeviceName(), str, label,
                           MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);
    }
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

        defineText(&VersionsTP);
        defineNumber(&InputVoltageNP);

        for (unsigned int p = 0; p < numPorts; p++)
        {
            defineText(&Ports[p].NameTP);
            defineSwitch(&Ports[p].EnableSP);
            defineSwitch(&Ports[p].ModeSP);
            defineNumber(&Ports[p].FuseNP);
            defineNumber(&Ports[p].DutyCycleNP);
            defineNumber(&Ports[p].CurrentNP);
        }
        SetTimer(POLLMS);
    }
    else
    {
        deleteProperty(VersionsTP.name);
        deleteProperty(InputVoltageNP.name);
        for (unsigned int p = 0; p < numPorts; p++)
        {
            deleteProperty(Ports[p].NameTP.name);
            deleteProperty(Ports[p].EnableSP.name);
            deleteProperty(Ports[p].ModeSP.name);
            deleteProperty(Ports[p].FuseNP.name);
            deleteProperty(Ports[p].DutyCycleNP.name);
            deleteProperty(Ports[p].CurrentNP.name);
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

    SetTimer(POLLMS);
}

bool KOMAHUB::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        for (unsigned int p = 0; p < numPorts; p++)
        {
            if (!strcmp(Ports[p].NameTP.name, name))
            {
                IUUpdateText(&Ports[p].NameTP, texts, names, n);
                if (strlen(Ports[p].NameT.text) >= 16)
                {
                    Ports[p].NameTP.s = IPS_ALERT;
                    IDSetText(&Ports[p].NameTP, nullptr);
                    LOG_ERROR("Too long port name, maximum of 15 characters supported!");
                    return false;
                }

                strncpy(Ports[p].settings.name, Ports[p].NameT.text, sizeof(Ports[p].settings.name));
                if (configureOutput(p))
                {
                    if (readOutputSettings())
                    {
                        Ports[p].NameTP.s = IPS_OK;
                        IDSetText(&Ports[p].NameTP, nullptr);
                        return true;
                    }
                }
                return false;
            }
        }
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool KOMAHUB::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        for (unsigned int p = 0; p < numPorts; p++)
        {
            if (!strcmp(Ports[p].DutyCycleNP.name, name))
            {
                IUUpdateNumber(&Ports[p].DutyCycleNP, values, names, n);

                if (setPwmDuty(p, static_cast<unsigned char>(Ports[p].DutyCycleN.value)))
                {
                    Ports[p].DutyCycleNP.s = IPS_OK;
                    IDSetNumber(&Ports[p].DutyCycleNP, nullptr);
                    readStatus();
                    return true;
                }
                return false;
            }
            if (!strcmp(Ports[p].FuseNP.name, name))
            {
                IUUpdateNumber(&Ports[p].FuseNP, values, names, n);
                Ports[p].settings.fuseCurrent = static_cast<unsigned char>(Ports[p].FuseN.value * 10.0);
                if (configureOutput(p))
                {
                    if (readOutputSettings())
                    {
                        Ports[p].FuseNP.s = IPS_OK;
                        IDSetNumber(&Ports[p].FuseNP, nullptr);
                        return true;
                    }
                }
                return false;
            }
        }
    }

    return DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool KOMAHUB::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        for (unsigned int p = 0; p < numPorts; p++)
        {
            if (!strcmp(Ports[p].ModeSP.name, name))
            {
                IUUpdateSwitch(&Ports[p].ModeSP, states, names, n);
                int mode = IUFindOnSwitchIndex(&Ports[p].ModeSP);
                if (mode >= OUTPUT_OFF && mode <= OUTPUT_PWM)
                {
                    Ports[p].settings.type = static_cast<unsigned char>(mode);
                    if (configureOutput(p))
                    {
                        if (readOutputSettings())
                        {
                            Ports[p].ModeSP.s = IPS_OK;
                            IDSetSwitch(&Ports[p].ModeSP, nullptr);
                            return true;
                        }
                    }
                }
                return false;
            }
            if (!strcmp(Ports[p].EnableSP.name, name))
            {
                IUUpdateSwitch(&Ports[p].EnableSP, states, names, n);
                int mode = IUFindOnSwitchIndex(&Ports[p].EnableSP);
                switch (mode)
                {
                    case 0:
                        if (setRelay(p, false))
                        {
                            Ports[p].EnableSP.s = IPS_OK;
                            IDSetSwitch(&Ports[p].EnableSP, nullptr);
                            readStatus();
                            return true;
                        }
                        break;
                    case 1:
                        if (setRelay(p, true))
                        {
                            Ports[p].EnableSP.s = IPS_OK;
                            IDSetSwitch(&Ports[p].EnableSP, nullptr);
                            readStatus();
                            return true;
                        }
                        break;
                    case 2:
                        if (resetFuse(p))
                        {
                            Ports[p].EnableSP.s = IPS_OK;
                            IDSetSwitch(&Ports[p].EnableSP, nullptr);
                            readStatus();
                            return true;
                        }
                        break;
                }
            }
        }
    }

    return DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}
