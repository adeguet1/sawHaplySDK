/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-11-10

  (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawHaplySDK/mtsHaply.h>

#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <thread>
#include <chrono>

class mtsHaplySocket {
public:
    typedef websocketpp::client<websocketpp::config::asio_client> ws_client_t;
    ws_client_t m_ws_client;
    websocketpp::connection_hdl m_ws_hdl;
    std::string m_ws_uri;
    bool m_ws_connected = false;
    std::string m_ws_response;

    mtsHaplySocket() {}
    ~mtsHaplySocket() {}

    void Configure(const std::string & uri) {
        m_ws_uri = uri;
        m_ws_client.init_asio();
        m_ws_client.set_message_handler([this](websocketpp::connection_hdl, ws_client_t::message_ptr msg) {
            m_ws_response = msg->get_payload();
        });
        websocketpp::lib::error_code ec;
        ws_client_t::connection_ptr con = m_ws_client.get_connection(m_ws_uri, ec);
        if (ec) {
            std::cout << "WebSocket configure error: " << ec.message() << std::endl;
            return;
        }
        m_ws_client.connect(con);
        m_ws_hdl = con->get_handle();
        m_ws_connected = true;
    }

    void SendRequest(const std::string & request) {
        if (m_ws_connected) {
            websocketpp::lib::error_code ec;
            m_ws_client.send(m_ws_hdl, request, websocketpp::frame::opcode::text, ec);
            if (ec) {
                std::cout << "WebSocket send error: " << ec.message() << std::endl;
                return;
            }
            m_ws_client.poll();
            if (!m_ws_response.empty()) {
                std::cout << "WebSocket response: " << m_ws_response << std::endl;
                m_ws_response.clear();
            }
        }
    }

    void Poll() {
        if (m_ws_connected) {
            m_ws_client.poll();
        }
    }
};

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsHaply, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

class mtsHaplyDevice
{
public:
    typedef std::list<mtsInterfaceProvided *> ButtonInterfaces;

    mtsHaplyDevice(const int deviceId,
                   const std::string & name,
                   mtsStateTable * stateTable,
                   mtsInterfaceProvided * interfaceProvided,
                   const ButtonInterfaces & buttonInterfaces);
    void Startup(void);
    void Run(void);
    void Cleanup(void);
    inline const std::string & Name(void) const {
        return m_name;
    }

    void GetButtonNames(std::list<std::string> & result) const;

protected:
    void GetRobotData(void);
    void SetControlMode(const mtsHaply::ControlModeType & mode);

    // crtk state
    void state_command(const std::string & command);
    prmOperatingState m_operating_state;
    mtsFunctionWrite m_operating_state_event;

    void servo_cp(const prmPositionCartesianSet & newPosition);
    void body_servo_cf(const prmForceCartesianSet & newForce);
    void move_cp(const prmPositionCartesianSet & newPosition);
    void use_gravity_compensation(const bool & gravityCompensation);
    void hold(void);
    void free(void);

    std::string m_device_id_string;
    std::string m_name;
    mtsStateTable * m_state_table;
    mtsInterfaceProvided * m_interface;

    uint mPreviousButtonMask;
    struct ButtonData {
        std::string Name;
        mtsFunctionWrite Function;
        bool Pressed;
    };
    typedef std::list<ButtonData *> ButtonsData;

    ButtonsData mButtonCallbacks;

    prmPositionCartesianGet m_measured_cp, m_setpoint_cp;
    prmVelocityCartesianGet m_measured_cv;
    prmForceCartesianGet m_body_measured_cf;

    vctMatRot3 m_orientation;

    mtsHaply::ControlModeType mControlMode;

    bool m_new_servo_cp;
    prmPositionCartesianSet m_servo_cp;
    prmForceCartesianSet m_body_servo_cf;
};


mtsHaplyDevice::mtsHaplyDevice(const int deviceId,
                               const std::string & name,
                               mtsStateTable * stateTable,
                               mtsInterfaceProvided * interfaceProvided,
                               const mtsHaplyDevice::ButtonInterfaces & buttonInterfaces):
    /* m_device(nullptr), */
    m_name(name),
    m_state_table(stateTable),
    m_interface(interfaceProvided)
{
    std::stringstream idString;
    idString << deviceId;
    m_device_id_string = idString.str();

    m_operating_state.IsBusy() = false;
    m_operating_state.Valid() = true;

    m_new_servo_cp = false;
    mControlMode = mtsHaply::UNDEFINED;

    m_body_servo_cf.Force().SetAll(0.0);

    m_measured_cp.SetReferenceFrame(m_name + "_base");
    m_measured_cp.SetMovingFrame(m_name);
    m_setpoint_cp.SetReferenceFrame(m_name + "_base");
    m_setpoint_cp.SetMovingFrame(m_name);

    m_state_table->SetAutomaticAdvance(false);
    m_state_table->AddData(m_operating_state, "operating_state");
    m_state_table->AddData(m_measured_cp, "measured_cp");
    m_state_table->AddData(m_measured_cv, "measured_cv");
    m_state_table->AddData(m_body_measured_cf, "body/measured_cf");
    m_state_table->AddData(m_setpoint_cp, "setpoint_cp");

    if (m_interface) {
        // system messages
        m_interface->AddMessageEvents();
        // state
        m_interface->AddCommandReadState(*m_state_table, m_measured_cp,
                                         "measured_cp");
        m_interface->AddCommandReadState(*m_state_table, m_measured_cv,
                                         "measured_cv");
        m_interface->AddCommandReadState(*m_state_table, m_body_measured_cf,
                                         "body/measured_cf");
        m_interface->AddCommandReadState(*m_state_table, m_setpoint_cp,
                                         "setpoint_cp");
        // commands
        m_interface->AddCommandWrite(&mtsHaplyDevice::servo_cp,
                                     this, "servo_cp");
        m_interface->AddCommandWrite(&mtsHaplyDevice::body_servo_cf,
                                     this, "body/servo_cf");
        m_interface->AddCommandWrite(&mtsHaplyDevice::move_cp,
                                     this, "move_cp");
        m_interface->AddCommandWrite(&mtsHaplyDevice::use_gravity_compensation,
                                     this, "use_gravity_compensation");
        m_interface->AddCommandVoid(&mtsHaplyDevice::hold,
                                    this, "hold");
        m_interface->AddCommandVoid(&mtsHaplyDevice::free,
                                    this, "free");
        // configuration
        m_interface->AddCommandRead(&mtsHaplyDevice::GetButtonNames,
                                    this, "get_button_names");
        // robot State
        m_interface->AddCommandWrite(&mtsHaplyDevice::state_command,
                                     this, "state_command", std::string(""));
        m_interface->AddCommandReadState(*m_state_table, m_operating_state,
                                         "operating_state");
        m_interface->AddEventWrite(m_operating_state_event, "operating_state",
                                   prmOperatingState());
        // stats
        m_interface->AddCommandReadState(*m_state_table, m_state_table->PeriodStats,
                                         "period_statistics");
    }

    // buttons
    const ButtonInterfaces::const_iterator endButtons = buttonInterfaces.end();
    ButtonInterfaces::const_iterator buttonInterface;
    for (buttonInterface = buttonInterfaces.begin();
         buttonInterface != endButtons;
         ++buttonInterface) {
        ButtonData * data = new ButtonData;
        mButtonCallbacks.push_back(data);
        data->Name = (*buttonInterface)->GetName();
        data->Pressed = false;
        (*buttonInterface)->AddEventWrite(data->Function, "Button", prmEventButton());
    }
}


void mtsHaplyDevice::Startup(void)
{
    std::string first_port;

    // Here we use the `Haply::HardwareAPI::DeviceDetection::DetectInverse3s`
    // static function to list all the Inverse3 devices currently connected. The
    // return value is a vector of string representing the serial COM ports that
    // can be used to communicate with the device.
    {
        // auto list = API::Devices::DeviceDetection::DetectInverse3s();
        // for (const auto & port : list) {
        //     m_interface->SendStatus(m_name + ": found device on port " + port);
        // }
        // 
        // if (list.empty()) {
        //     m_interface->SendError(m_name + ":no inverse3 detected");
        // }
        // 
        // first_port = list[0];
        // fprintf(stdout, "using the first inverse3 found: %s\n",
        //         first_port.c_str());
    }

    // handle
    char* portName;
    
    std::string portNames[256];
    // std::vector<std::string> ports =
    //     Haply::HardwareAPI::Devices::DeviceDetection::DetectHandles();
    // int portCount = static_cast<int>(ports.size());
    // for (int i = 0; i < portCount; i++) {
    //     portNames[i] = ports[i];
    // }
    // if (portCount > 0) {
    //     int index = portCount - 1;
    //     portName = strdup(portNames[index].c_str());
    //     std::cerr << "Using handle " << portName << std::endl;
    // } else {
    //     std::cout << "No Handle found" << std::endl;
    // }
    // m_device_stream = new API::IO::SerialStream(first_port.c_str());

    // Using the `API::IO::SerialStream` object we can initialize the
    // Inverse3 object which will encapsulates all the logic needed to
    // interact with an Inverse3 device.
    // m_device = new API::Devices::Inverse3(m_device_stream);

    // To start using the device, we first need to wake it up using the
    // `DeviceWakeup` function. Once awake, the LED colour on the device
    // should change to indicate that it's ready to receive commands. The
    // method returns a struct of type `Inverse3::DeviceInfoResponse` which
    // contains the device's general information.
    // auto info = m_device->DeviceWakeup();
    // std::fprintf(stdout,
    //              "info: id=%u, model=%u, version={hardware:%u, firmware:%u}\n",
    //              info.device_id, info.device_model_number,
    //              info.hardware_version, info.firmware_version);


    // m_handle_stream = new API::IO::SerialStream(portName);
    // m_handle = new API::Devices::Handle(m_handle_stream);

    m_interface->SendStatus(m_name + ": properly initialized");
    m_operating_state.IsHomed() = true;
    m_operating_state.State() = prmOperatingState::ENABLED;
    m_operating_state_event(m_operating_state);

    // update current state
    m_state_table->Start();
    GetRobotData();
    SetControlMode(mtsHaply::SERVO_CF);
    m_state_table->Advance();
}

void mtsHaplyDevice::Run(void)
{
    m_state_table->Start();
    // process mts commands
    m_interface->ProcessMailBoxes();
    // ws_client.poll(); // Non-blocking event loop step
    GetRobotData();

    // control mode
    switch (mControlMode) {
    case mtsHaply::SERVO_CF:
        /*
        dhdSetForceAndTorqueAndGripperForce(m_body_servo_cf.Force()[0],
                                            m_body_servo_cf.Force()[1],
                                            m_body_servo_cf.Force()[2],
                                            m_body_servo_cf.Force()[3],
                                            m_body_servo_cf.Force()[4],
                                            m_body_servo_cf.Force()[5],
                                            m_gripper_direction * m_gripper_servo_jf,
                                            m_device_id);
        m_setpoint_cp.Position().Assign(m_measured_cp.Position());
        m_setpoint_cp.Valid() = m_measured_cp.Valid();
        */
        break;
    case mtsHaply::SERVO_CP:
        /*
        if (m_new_servo_cp) {
            drdTrackPos(m_servo_cp.Goal().Translation().X(),
                        m_servo_cp.Goal().Translation().Y(),
                        m_servo_cp.Goal().Translation().Z(),
                        m_device_id);
            m_new_servo_cp = false;
            m_setpoint_cp.Position().Assign(m_servo_cp.Goal());
            m_setpoint_cp.Valid() = true;
        }
        */
        break;
    case mtsHaply::MOVE_CP:
        /*
        // check if the arm is still moving
        if (m_operating_state.IsBusy()
            && (!drdIsMoving(m_device_id))) {
            m_operating_state.IsBusy() = false;
            m_operating_state_event(m_operating_state);
            // save last setpoint using goal for move
            m_setpoint_cp.Position().Assign(m_servo_cp.Goal());
            m_setpoint_cp.Valid() = true;
        } else {
            // still moving, we update the setpoint from measured,
            // maybe there's a method in SDK to retrieve last setpoint
            // but I couldn't find it
            m_setpoint_cp.Position().Assign(m_measured_cp.Position());
            m_setpoint_cp.Valid() = m_measured_cp.Valid();
        }
        */
        break;
    default:
        break;
    }
    m_state_table->Advance();
}

void mtsHaplyDevice::Cleanup(void)
{
    // dhdClose(m_device_id);
}

void mtsHaplyDevice::GetButtonNames(std::list<std::string> & result) const
{
    result.clear();
    const ButtonsData::const_iterator end = mButtonCallbacks.end();
    ButtonsData::const_iterator button;
    for (button = mButtonCallbacks.begin();
         button != end;
         ++button) {
        result.push_back((*button)->Name);
    }
}

void mtsHaplyDevice::GetRobotData(void)
{
    // Send request and poll for response using WebSocket++
    if (ws_connected) {
        std::string request = "robot_data_request";
        websocketpp::lib::error_code ec;
        ws_client.send(ws_hdl, request, websocketpp::frame::opcode::text, ec);
        if (ec) {
            std::cout << "WebSocket send error: " << ec.message() << std::endl;
            return;
        }
        // Poll for events (non-blocking, single step)
        ws_client.poll();
        // Optionally, check ws_response for new data
        if (!ws_response.empty()) {
            std::cout << "WebSocket response: " << ws_response << std::endl;
            ws_response.clear();
        }
    }

    /*
    double rotation[3][3];
    // position
    dhdGetPositionAndOrientationFrame(&m_measured_cp.Position().Translation().X(),
                                      &m_measured_cp.Position().Translation().Y(),
                                      &m_measured_cp.Position().Translation().Z(),
                                      rotation,
                                      m_device_id);
    m_orientation.Row(0).Assign(rotation[0]);
    m_orientation.Row(1).Assign(rotation[1]);
    m_orientation.Row(2).Assign(rotation[2]);
    // apply rotation offset
    m_measured_cp.Position().Rotation() = m_orientation;
    */
    m_measured_cp.Valid() = true;
    /*
    // velocity
    dhdGetLinearVelocity(&m_measured_cv.VelocityLinear().X(),
                         &m_measured_cv.VelocityLinear().Y(),
                         &m_measured_cv.VelocityLinear().Z(),
                         m_device_id);
    dhdGetAngularVelocityRad(&m_measured_cv.VelocityAngular().X(),
                             &m_measured_cv.VelocityAngular().Y(),
                             &m_measured_cv.VelocityAngular().Z(),
                             m_device_id);
    m_measured_cv.SetValid(true);

    // force
    dhdGetForceAndTorqueAndGripperForce(&m_body_measured_cf.Force()[0],
                                        &m_body_measured_cf.Force()[1],
                                        &m_body_measured_cf.Force()[2],
                                        &m_body_measured_cf.Force()[3],
                                        &m_body_measured_cf.Force()[4],
                                        &m_body_measured_cf.Force()[5],
                                        &m_gripper_measured_js.Effort().at(0),
                                        m_device_id);
    m_body_measured_cf.SetValid(true);

    // buttons
    uint currentButtonMask = dhdGetButtonMask(m_device_id);
    // if any button pressed
    if (mPreviousButtonMask != currentButtonMask) {
        mPreviousButtonMask = currentButtonMask;
        uint index = 0;
        const ButtonsData::iterator buttonsEnd = mButtonCallbacks.end();
        ButtonsData::iterator button;
        for (button = mButtonCallbacks.begin();
             button != buttonsEnd;
             ++button) {
            const bool current =  (currentButtonMask & (1 << index));
            index++;
            if (current != (*button)->Pressed) {
                (*button)->Pressed = current;
                // generate event
                prmEventButton event;
                event.SetValid(true);
                if (current) {
                    event.SetType(prmEventButton::PRESSED);
                } else {
                    event.SetType(prmEventButton::RELEASED);
                }
                (*button)->Function(event);
            }
        }
    }
    */
}

void mtsHaplyDevice::state_command(const std::string & command)
{
    std::string humanReadableMessage;
    prmOperatingState::StateType newOperatingState;
    try {
        if (m_operating_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                           newOperatingState, humanReadableMessage)) {
            if (command == "enable") {
                m_operating_state.State() = prmOperatingState::ENABLED;
            } else if (command == "disable") {
                m_operating_state.State() = prmOperatingState::DISABLED;
            } else {
                m_interface->SendStatus(this->m_name + ": state command \""
                                        + command + "\" is not supported yet");
            }
            // always emit event with current device state
            m_interface->SendStatus(this->m_name
                                    + ": current state is \""
                                    + prmOperatingState::StateTypeToString(m_operating_state.State()) + "\"");
            m_operating_state.Valid() = true;
            m_operating_state_event(m_operating_state);
        } else {
            m_interface->SendWarning(this->m_name + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        m_interface->SendWarning(this->m_name + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}

void mtsHaplyDevice::SetControlMode(const mtsHaply::ControlModeType & mode)
{
    // return if we are already in this mode
    if (mode == mControlMode) {
        return;
    }
    // transition to new mode
    switch (mode) {
    case mtsHaply::SERVO_CP:
    case mtsHaply::MOVE_CP:
        /*
        m_new_servo_cp = false;
        drdRegulatePos(true, m_device_id);
        drdRegulateRot(false, m_device_id);
        drdRegulateGrip(false, m_device_id);
        if (drdStart(m_device_id) < 0) {
            m_interface->SendError(m_name + ": failed to start control loop, "
                                   + dhdErrorGetLastStr() + " [id:" + m_device_id_string + "]");
        }
        // start from current position
        m_servo_cp.Goal().Assign(m_measured_cp.Position());
        m_new_servo_cp = true;
        */
        break;
    case mtsHaply::SERVO_CF:
        /*
        drdRegulatePos(false, m_device_id);
        drdStop(true, m_device_id);
        // start with 0 forces
        m_body_servo_cf.Force().SetAll(0.0);
        */
        break;
    default:
        break;
    }
    // assign mode
    mControlMode = mode;
}

void mtsHaplyDevice::body_servo_cf(const prmForceCartesianSet & wrench)
{
    SetControlMode(mtsHaply::SERVO_CF);
    m_body_servo_cf = wrench;
}

void mtsHaplyDevice::servo_cp(const prmPositionCartesianSet & position)
{
    SetControlMode(mtsHaply::SERVO_CP);
    m_servo_cp = position;
    m_new_servo_cp = true;
}

void mtsHaplyDevice::move_cp(const prmPositionCartesianSet & position)
{
    SetControlMode(mtsHaply::MOVE_CP);
    /*
    m_servo_cp = position;
    drdMoveToPos(m_servo_cp.Goal().Translation().X(),
                 m_servo_cp.Goal().Translation().Y(),
                 m_servo_cp.Goal().Translation().Z(),
                 false,
                 m_device_id);
    m_operating_state.IsBusy() = true;
    m_operating_state_event(m_operating_state);
    */
}

void mtsHaplyDevice::hold(void)
{
    SetControlMode(mtsHaply::SERVO_CP);
    m_servo_cp.Goal().Assign(m_measured_cp.Position());
}

void mtsHaplyDevice::free(void)
{
    SetControlMode(mtsHaply::SERVO_CF);
    use_gravity_compensation(true);
}

void mtsHaplyDevice::use_gravity_compensation(const bool & gravity)
{
    if (gravity) {
        // dhdSetGravityCompensation(DHD_ON, m_device_id);
    } else {
        // dhdSetGravityCompensation(DHD_OFF, m_device_id);
    }
}

void mtsHaply::Init(void)
{
    mConfigured = false;
}

void mtsHaply::Configure(const std::string & filename)
{
    /*
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    if (mConfigured) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: already configured" << std::endl;
        return;
    }

    typedef std::list<std::string> NoSerialDevicesType;
    NoSerialDevicesType noSerialDevices;
    typedef std::map<int, std::string> SerialDevicesType;
    SerialDevicesType serialDevices;

    if (filename != "") {
        // read JSON file passed as param, see configAtracsysFusionTrack.json for an example
        std::ifstream jsonStream;
        jsonStream.open(filename.c_str());

        Json::Value jsonConfig, jsonValue;
        Json::Reader jsonReader;
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        const Json::Value jsonDevices = jsonConfig["devices"];
        for (unsigned int index = 0; index < jsonDevices.size(); ++index) {
            jsonValue = jsonDevices[index];
            std::string deviceName = jsonValue["name"].asString();
            // make sure toolName is valid
            if (deviceName == "") {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                         << index << "] name found in "
                                         << filename << std::endl;
                return;
            }
            if (jsonValue["serial"].empty() || !jsonValue["serial"].isInt()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                         << index << "] serial found in "
                                         << filename << " (must be an integer)"
                                         << std::endl;
                return;
            }
            int deviceSerial = jsonValue["serial"].asInt();
            if (deviceSerial == 0) {
                noSerialDevices.push_back(deviceName);
            } else {
                // check that this serial number hasn't already been assigned
                SerialDevicesType::const_iterator found = serialDevices.find(deviceSerial);
                if (found == serialDevices.end()) {
                    serialDevices[deviceSerial] = deviceName;
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                             << index << "], serial number "
                                             << deviceSerial << " already used in "
                                             << filename << std::endl;
                    return;
                }
            }
        }
    }

    // required to change asynchronous operation mode
    dhdEnableExpertMode();

    // sdk version number
    dhdGetSDKVersion (&(mSDKVersion.Major),
                      &(mSDKVersion.Minor),
                      &(mSDKVersion.Release),
                      &(mSDKVersion.Revision));
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using SDK "
                               << mSDKVersion.Major << "."
                               << mSDKVersion.Minor << "."
                               << mSDKVersion.Release << "."
                               << mSDKVersion.Revision << std::endl;

    mNumberOfDevices = dhdGetDeviceCount();
    if (mNumberOfDevices < 1) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find any device.  Make sure you have the correct permissions to access the USB devices.  See also README.md for sawHaplySDK" << std::endl;
        return;
    }

    // identify and name each device
    for (int i = 0;
         i < mNumberOfDevices; i++) {
        const int deviceId = drdOpenID(i);
        if (deviceId == -1) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to open device "
                                     << i << ", id: " << deviceId << std::endl;

        }
        std::string systemName = dhdGetSystemName(deviceId);
        bool hasSerial = true;
        ushort serialUShort = 0;
        if (dhdGetSerialNumber(&serialUShort, deviceId) != 0) {
            // error while getting serial number
            hasSerial = false;
            CMN_LOG_CLASS_INIT_WARNING << "Configure: can't retrieve serial number for device "
                                       << i << ", id: " << deviceId << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Configure: found device with serial number: " << serialUShort << std::endl;
        }
        // for some reason, FALCON serial number is always 65535
        if (dhdGetSystemType(deviceId) == DHD_DEVICE_FALCON) {
            hasSerial = false;
        }
        // find name of device based on serial number in configuration file
        std::string deviceName = "";
        if (hasSerial) {
            // look for name on map from configuration file
            SerialDevicesType::const_iterator found = serialDevices.find(serialUShort);
            if (found != serialDevices.end()) {
                deviceName = found->second;
            }
        }
        // if we still don't have a name
        if (deviceName == "") {
            // look in the list of names from configuration file
            if (!noSerialDevices.empty()) {
                deviceName = *(noSerialDevices.begin());
                noSerialDevices.pop_front();
            } else {
                // now we just make up a name
                std::stringstream tempName;
                tempName << systemName << std::setfill('0') << std::setw(2) << deviceId;
                deviceName = tempName.str();
            }
            } */

    // ugly hard code to test
    std::string deviceName = "Test";
    int deviceId = 0;
    int i = 0;
    
    // create list of buttons based on device type
    std::list<mtsInterfaceProvided *> buttonInterfaces;
    // if (dhdGetSystemType(deviceId) == DHD_DEVICE_FALCON) {
    buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "/center"));
    buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "/left"));
    buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "/top"));
    buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "/right"));
    // }
    
    // create the device data and add to list of devices
    mtsStateTable * stateTable
        = new mtsStateTable(StateTable.GetHistoryLength(),
                            deviceName);
    mtsInterfaceProvided * interfaceProvided
        = this->AddInterfaceProvided(deviceName);
    if (!interfaceProvided) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: can't create interface provided with name \""
                                 << deviceName << "\".  Device "
                                 << i << ", id: " << deviceId << std::endl;
        return;
    }
    mtsHaplyDevice * device =
        new mtsHaplyDevice(deviceId, deviceName,
                           stateTable, interfaceProvided,
                           buttonInterfaces);
    mDevices.push_back(device);
}


void mtsHaply::Startup(void)
{
    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Startup();
    }
}


void mtsHaply::GetDeviceNames(std::list<std::string> & result) const
{
    result.clear();
    const DevicesType::const_iterator end = mDevices.end();
    DevicesType::const_iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        result.push_back((*device)->Name());
    }
}

void mtsHaply::GetButtonNames(const std::string & deviceName,
                                       std::list<std::string> & result) const
{
    result.clear();
    const DevicesType::const_iterator end = mDevices.end();
    DevicesType::const_iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        if ((*device)->Name() == deviceName) {
            (*device)->GetButtonNames(result);
            return;
        }
    }
}

void mtsHaply::Run(void)
{
    // process mts commands using interface->ProcessMailBoxes
    // DO NOT USE ProcessQueuedCommands() !!!

    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Run();
    }
}

void mtsHaply::Cleanup(void)
{
    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Cleanup();
    }
    // drdClose();
}
