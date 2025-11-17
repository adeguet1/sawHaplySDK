/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsHaply_h
#define _mtsHaply_h

#include <cisstMultiTask/mtsTaskContinuous.h>

#include <sawHaplySDK/sawHaplySDKExport.h>  // always include last

class mtsHaplyDevice;

class CISST_EXPORT mtsHaply: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    inline mtsHaply(const std::string & componentName):
        mtsTaskContinuous(componentName, 256) {
        Init();
    }

    inline mtsHaply(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg) {
        Init();
    }

    ~mtsHaply(void) {};

    void Configure(const std::string & filename = "");
    void GetDeviceNames(std::list<std::string> & result) const;
    void GetButtonNames(const std::string & deviceName,
                        std::list<std::string> & result) const;
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    enum ControlModeType {UNDEFINED, SERVO_CP, SERVO_CF, MOVE_CP};

 protected:

    void Init(void);

    bool mConfigured;

    struct {
        int Major;
        int Minor;
        int Release;
        int Revision;
    } mSDKVersion;

    int mNumberOfDevices;

    typedef std::list<mtsHaplyDevice *> DevicesType;
    DevicesType mDevices;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsHaply);

#endif  // _mtsHaply_h
