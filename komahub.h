/*
 KomaHub driver

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

#pragma once

#include <defaultdevice.h>
#include <hidapi.h>

#include "indipropertyswitch.h"
#include "indipropertynumber.h"
#include "indipropertytext.h"

class KOMAHUB : public INDI::DefaultDevice
{
    public:
        KOMAHUB(hid_device *device, const char* name);
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

        static const unsigned char magic = 'K';

        enum OutputType
        {
            OUTPUT_OFF = 0,
            OUTPUT_DC  = 1,
            OUTPUT_PWM = 2,
            // OUTPUT_PWM_PID_HEAT = 3,
            // OUTPUT_PWM_PID_COOL = 4,
            // OUTPUT_PWM_FAST     = 5
        };

#include "USBCommands.h"

        bool readFactorySettings();
        bool readStatus();
        bool readOutputSettings();
        bool configureOutput(unsigned char output);
        bool setPwmDuty(unsigned char output, unsigned char duty);
        bool setRelay(unsigned char output, bool enabled);
        bool resetFuse(unsigned char output);

        INDI::PropertyText VersionsTP{3};

        INDI::PropertyNumber InputVoltageNP{1};

        typedef struct
        {
            GetOutputSettingsResponse settings;

            INDI::PropertyText NameTP{1};
            INDI::PropertySwitch EnableSP{3};
            INDI::PropertySwitch ModeSP{3};
            INDI::PropertyNumber FuseNP{1};
            INDI::PropertyNumber DutyCycleNP{1};
            INDI::PropertyNumber CurrentNP{1};
        } PortStruct;

        PortStruct Ports[numPorts] {};

        // DS18B20 temperature probes, up to 4
        unsigned int numTemperatureProbes{ 0 };
        INDI::PropertyNumber TemperaturesNP{4};

        // BME280 Pressure-Temperature-Humidity sensor
        bool pthPresent{ false };
        INDI::PropertyNumber HumidityNP{1};
        INDI::PropertyNumber PressureNP{1};
        INDI::PropertyNumber TemperatureNP{1};
        INDI::PropertyNumber DewpointNP{1};

        // TSL237 sky quality sensor
        bool skyqualityPresent{ false };
        INDI::PropertyNumber SkyQualityNP{2}; // Quality and raw frequency

        // MLX90614 sky temperature sensor
        bool skytemperaturePresent{ false };
        INDI::PropertyNumber SkyTemperatureNP{2}; // Ambient and sky temperatures
};
