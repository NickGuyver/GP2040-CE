/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2024 OpenStickCommunity (gp2040-ce.info)
 */

#include "drivers/xinput/XInputDriver.h"
#include "drivers/shared/USBReportScheduler.h"
#include "storagemanager.h"

static bool isDigitalAxisAction(GpioAction action) {
    switch (action) {
        case ANALOG_DIRECTION_LS_X_NEG:
        case ANALOG_DIRECTION_LS_X_POS:
        case ANALOG_DIRECTION_LS_Y_NEG:
        case ANALOG_DIRECTION_LS_Y_POS:
        case ANALOG_DIRECTION_RS_X_NEG:
        case ANALOG_DIRECTION_RS_X_POS:
        case ANALOG_DIRECTION_RS_Y_NEG:
        case ANALOG_DIRECTION_RS_Y_POS:
            return true;
        default:
            return false;
    }
}

void XInputDriver::updateAnalogScheduling(uint32_t appliedProfileNumber) {
    USBReportScheduler::getInstance().resetInputTiming();
    Storage &storage = Storage::getInstance();
    analogSchedulingProfile = appliedProfileNumber;
    analogSchedulingAllowed = false;

    // Axis reports do not retain whether a value came from a stick or a
    // button. Keep ordinary FIFO submission when digital axis mappings could
    // otherwise be deferred or overwritten as disposable analog movement.
    // These driver mappings are initialized once, not on profile changes.
    if ((deviceType == INPUT_MODE_DEVICE_TYPE_WHEEL &&
         (buttonSteerLeft->pinMask || buttonSteerRight->pinMask)) ||
        (deviceType == INPUT_MODE_DEVICE_TYPE_GUITAR &&
         (buttonWhammy->pinMask || buttonTilt->pinMask))) {
        return;
    }

    GpioMappingInfo const *pinMappings = storage.getProfilePinMappings();
    bool dualDirectionalMapped = false;
    for (Pin_t pin = 0; pin < static_cast<Pin_t>(NUM_BANK0_GPIOS); ++pin) {
        GpioAction const action = pinMappings[pin].action;
        dualDirectionalMapped |= action == BUTTON_PRESS_DDI_UP || action == BUTTON_PRESS_DDI_DOWN ||
            action == BUTTON_PRESS_DDI_LEFT || action == BUTTON_PRESS_DDI_RIGHT;
        if (isDigitalAxisAction(action)) {
            return;
        }
    }

    AddonOptions const &addons = storage.getAddonOptions();
    if (dualDirectionalMapped && addons.dualDirectionalOptions.enabled &&
        addons.dualDirectionalOptions.dpadMode != DPAD_MODE_DIGITAL) {
        return;
    }
    if (addons.heTriggerOptions.enabled) {
        for (HETriggerInfo const &trigger : addons.heTriggerOptions.triggers) {
            if (isDigitalAxisAction(trigger.action)) {
                return;
            }
        }
    }

    if (!storage.getGamepadOptions().lockHotkeys) {
        HotkeyOptions const &hotkeys = storage.getHotkeyOptions();
        GamepadHotkey const actions[] = {
            hotkeys.hotkey01.action, hotkeys.hotkey02.action, hotkeys.hotkey03.action, hotkeys.hotkey04.action,
            hotkeys.hotkey05.action, hotkeys.hotkey06.action, hotkeys.hotkey07.action, hotkeys.hotkey08.action,
            hotkeys.hotkey09.action, hotkeys.hotkey10.action, hotkeys.hotkey11.action, hotkeys.hotkey12.action,
            hotkeys.hotkey13.action, hotkeys.hotkey14.action, hotkeys.hotkey15.action, hotkeys.hotkey16.action,
        };
        for (GamepadHotkey const action : actions) {
            switch (action) {
                case HOTKEY_LS_UP:
                case HOTKEY_LS_DOWN:
                case HOTKEY_LS_LEFT:
                case HOTKEY_LS_RIGHT:
                case HOTKEY_RS_UP:
                case HOTKEY_RS_DOWN:
                case HOTKEY_RS_LEFT:
                case HOTKEY_RS_RIGHT:
                    return;
                default:
                    break;
            }
        }
    }
    analogSchedulingAllowed = true;
}
