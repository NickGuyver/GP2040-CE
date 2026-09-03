/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2024 OpenStickCommunity (gp2040-ce.info)
 */

#ifndef _PS3_DRIVER_H_
#define _PS3_DRIVER_H_

#include "gpdriver.h"
#include "drivers/ps3/PS3Descriptors.h"

class PS3Driver : public GPDriver {
public:
    virtual void initialize();
    virtual bool process(Gamepad * gamepad);
    virtual void initializeAux() {}
    virtual void processAux() {}
    virtual uint16_t get_report(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen);
    virtual void set_report(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize);
    virtual bool vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
    virtual const uint16_t * get_descriptor_string_cb(uint8_t index, uint16_t langid);
    virtual const uint8_t * get_descriptor_device_cb();
    virtual const uint8_t * get_hid_descriptor_report_cb(uint8_t itf) ;
    virtual const uint8_t * get_descriptor_configuration_cb(uint8_t index);
    virtual const uint8_t * get_descriptor_device_qualifier_cb();
    virtual uint16_t GetJoystickMidValue();
    virtual USBListener * get_usb_auth_listener() { return nullptr; }
private:
    uint8_t last_report[CFG_TUD_ENDPOINT0_SIZE] = { };
    PS3Report ps3Report{};
    PS3ReportAlt ps3ReportAlt{};
    PS3Features ps3Features{};
    uint8_t lastFeatures[PS3_FEATURES_SIZE] = { };
    PS3BTInfo ps3BTInfo{};

    // this is an identification byte from the H2D 0xEF feature report that needs to be the same 
    // in multiple D2H reports for the controller to function
    uint8_t efByte = 0;

    InputModeDeviceType deviceType{};
    uint8_t deviceDescriptor[sizeof(ps3_device_descriptor)]{};

    GamepadButtonMapping *buttonFretGreen = nullptr;
    GamepadButtonMapping *buttonFretRed = nullptr;
    GamepadButtonMapping *buttonFretYellow = nullptr;
    GamepadButtonMapping *buttonFretBlue = nullptr;
    GamepadButtonMapping *buttonFretOrange = nullptr;
    GamepadButtonMapping *buttonWhammy = nullptr;
    GamepadButtonMapping *buttonPickup = nullptr;
    GamepadButtonMapping *buttonTilt = nullptr;

    GamepadButtonMapping *buttonDrumPadRed = nullptr;
    GamepadButtonMapping *buttonDrumPadBlue = nullptr;
    GamepadButtonMapping *buttonDrumPadYellow = nullptr;
    GamepadButtonMapping *buttonDrumPadGreen = nullptr;
    GamepadButtonMapping *buttonCymbalYellow = nullptr;
    GamepadButtonMapping *buttonCymbalBlue = nullptr;
    GamepadButtonMapping *buttonCymbalGreen = nullptr;

    GamepadButtonMapping *buttonShiftUp = nullptr;
    GamepadButtonMapping *buttonShiftDown = nullptr;
    GamepadButtonMapping *buttonGas = nullptr;
    GamepadButtonMapping *buttonBrake = nullptr;
    GamepadButtonMapping *buttonSteerLeft = nullptr;
    GamepadButtonMapping *buttonSteerRight = nullptr;
    GamepadButtonMapping *buttonPlus = nullptr;
    GamepadButtonMapping *buttonMinus = nullptr;
    GamepadButtonMapping *buttonDialDown = nullptr;
    GamepadButtonMapping *buttonDialUp = nullptr;
    GamepadButtonMapping *buttonDialEnter = nullptr;
};

#endif // _PS3_DRIVER_H_
