/*
 KomaHub driver

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

#pragma once

#include <defaultdevice.h>
#include <hidapi.h>

using namespace std;

class KOMAHUB : public INDI::DefaultDevice
{
  public:
    KOMAHUB(hid_device *device);
    virtual ~KOMAHUB();

    const char *getDefaultName();

    virtual bool Connect() override;
    virtual bool Disconnect() override;

    virtual bool initProperties();
    virtual bool updateProperties();

    virtual void ISGetProperties(const char *dev);
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

    // http://pid.codes/1209/4242/
    static const unsigned short vendorID  = 0x1209;
    static const unsigned short productID = 0x4242;

    static const unsigned int numPorts = 6;

  protected:
    void TimerHit();

  private:
    hid_device *device;

    char name[32];

    friend void ::ISGetProperties(const char *dev);
    friend void ::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num);
    friend void ::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num);
    friend void ::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num);
    friend void ::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                            char *formats[], char *names[], int n);

    static const unsigned char magic = 'K';

    enum OutputType
    {
        OUTPUT_OFF = 0,
        OUTPUT_DC  = 1,
        OUTPUT_PWM = 2,
        //            OUTPUT_PWM_PID_HEAT = 3,
        //            OUTPUT_PWM_PID_COOL = 4,
        //            OUTPUT_PWM_FAST     = 5
    };

#include "USBCommands.h"

    bool readFactorySettings();
    bool readStatus();
    bool readOutputSettings();
    bool configureOutput(unsigned char output);
    bool setPwmDuty(unsigned char output, unsigned char duty);
    bool setRelay(unsigned char output, bool enabled);
    bool resetFuse(unsigned char output);

    IText VersionsT[3]{};
    ITextVectorProperty VersionsTP;

    INumber InputVoltageN;
    INumberVectorProperty InputVoltageNP;

    typedef struct
    {
        GetOutputSettingsResponse settings;

        IText NameT;
        ITextVectorProperty NameTP;

        ISwitch EnableS[3];
        ISwitchVectorProperty EnableSP;

        ISwitch ModeS[3];
        ISwitchVectorProperty ModeSP;

        INumber FuseN;
        INumberVectorProperty FuseNP;

        INumber DutyCycleN;
        INumberVectorProperty DutyCycleNP;

        INumber CurrentN;
        INumberVectorProperty CurrentNP;
    } PortStruct;

    PortStruct Ports[numPorts]{};
};
