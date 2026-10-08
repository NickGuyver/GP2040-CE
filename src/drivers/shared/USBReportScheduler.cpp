/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2026 OpenStickCommunity (gp2040-ce.info)
 */

#include "drivers/shared/USBReportScheduler.h"

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "device/dcd.h"

#include "pico/platform.h"
#include "pico/time.h"

#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "hardware/sync.h"

#include <cstring>

// The RP USB controller uses full-speed frames. Endpoint bInterval controls
// host polling, not SOF cadence; never use it as a frame deadline.
static constexpr uint32_t USB_FULL_SPEED_FRAME_US = 1000;

// time_us_32() reports whole microseconds. Add one unit when publishing an
// observed duration so truncation cannot turn it into an unsafe lower bound.
static constexpr uint32_t TIME_US_32_RESOLUTION_US = 1;

static inline uint32_t boundedElapsedUs(uint32_t elapsedUs) {
    return elapsedUs + TIME_US_32_RESOLUTION_US;
}

USBReportScheduler &__not_in_flash_func(USBReportScheduler::getInstance)() {
    static USBReportScheduler instance;
    return instance;
}

void USBReportScheduler::configure(uint8_t endpoint) {
    this->endpoint = endpoint;
    reset();
}

void USBReportScheduler::start() {
    uint8_t const epNum = endpoint & 0x0f;
    if ((endpoint & 0x80) && epNum != 0 && epNum < USB_NUM_ENDPOINTS) {
        // Latch polls while unarmed without enabling another USB interrupt.
        usb_hw->ep_nak_stall_status = 1u << (2 * epNum);
        usb_dpram->ep_ctrl[epNum - 1].in |= EP_CTRL_INTERRUPT_ON_NAK;
        // The class SOF callback runs in the ISR; no queued application SOF event.
        dcd_sof_enable(0, true);
    }
}

void USBReportScheduler::reset() {
    uint32_t const irqState = save_and_disable_interrupts();
    lastSofUs = 0;
    sofEpoch = 0;
    sofTimingReady = false;
    reportPendingAtSof = false;
    lastSofFrame = 0;
    lastNakCheckSofEpoch = 0;
    resetPollTiming();
    restore_interrupts(irqState);

    resetInputTiming();
    replacementGuardUs = 0;
    completedSofEpoch = 0;
    pendingSofEpoch = 0;
    deliveredPriority = {};
    pendingPriority = {};
    pendingOwned = false;
    pendingPriorityUnchanged = false;
    replacementTimingReady = false;
    completedSofEpochValid = false;
    pendingSofEpochValid = false;
}

void USBReportScheduler::resetInputTiming() {
    inputStartUs = 0;
    maxInputIntervalUs = 0;
    maxInputProcessingUs = 0;
    inputTimingStarted = false;
    inputTimingReady = false;
}

void USBReportScheduler::resetPollTiming() {
    lastPollValid = false;
    pollTimingReady = false;
    pollIntervalFrames = 0;
}

void __not_in_flash_func(USBReportScheduler::noteHostPoll)(uint32_t frame) {
    if (lastPollValid) {
        uint32_t const interval = frame - lastPollSofEpoch;
        if (interval == 0) {
            return;
        }
        // Require two agreeing gaps; a changed cadence immediately falls back.
        pollTimingReady = interval == pollIntervalFrames;
        pollIntervalFrames = interval;
    }
    lastPollSofEpoch = frame;
    lastPollValid = true;
}

void __not_in_flash_func(USBReportScheduler::onSof)(uint8_t rhport, uint32_t frameCount) {
    (void)rhport;
    USBReportScheduler &scheduler = getInstance();
    uint32_t const now = time_us_32();
    // Also catch pauses hidden by the USB frame counter's 11-bit wrap.
    if (scheduler.sofTimingReady &&
        (((frameCount - scheduler.lastSofFrame) & USB_SOF_RD_BITS) != 1 ||
         now - scheduler.lastSofUs >= 2 * USB_FULL_SPEED_FRAME_US)) {
        scheduler.resetPollTiming();
    }
    scheduler.lastSofFrame = frameCount;
    uint8_t const epNum = scheduler.endpoint & 0x0f;
    scheduler.reportPendingAtSof = epNum != 0 && epNum < USB_NUM_ENDPOINTS &&
        (usb_dpram->ep_buf_ctrl[epNum].in & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL)) ==
            (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL);
    scheduler.lastSofUs = now;
    ++scheduler.sofEpoch;
    scheduler.sofTimingReady = true;
}

void __not_in_flash_func(USBReportScheduler::beginInputProcessing)() {
    if (endpoint == 0) {
        return;
    }

    uint32_t const now = time_us_32();
    uint32_t const irqState = save_and_disable_interrupts();
    uint32_t const nakMask = 1u << (2 * (endpoint & 0x0f));
    if (usb_hw->ep_nak_stall_status & nakMask) {
        usb_hw->ep_nak_stall_status = nakMask; // W1C: leave other endpoints alone.
        // Only attribute a NAK when both observations are in the same frame.
        if (sofTimingReady && lastNakCheckSofEpoch == sofEpoch &&
            !(usb_hw->ints & USB_INTS_DEV_SOF_BITS)) {
            noteHostPoll(sofEpoch);
        } else {
            resetPollTiming();
        }
    }
    lastNakCheckSofEpoch = sofEpoch;
    restore_interrupts(irqState);
    if (inputTimingStarted) {
        uint32_t const intervalUs = boundedElapsedUs(now - inputStartUs);
        if (intervalUs > maxInputIntervalUs) {
            maxInputIntervalUs = intervalUs;
        }
    }
    inputStartUs = now;
    inputTimingStarted = true;
}

void __not_in_flash_func(USBReportScheduler::endInputProcessing)() {
    if (!inputTimingStarted) {
        return;
    }

    uint32_t const processing = boundedElapsedUs(time_us_32() - inputStartUs);
    if (processing > maxInputProcessingUs) {
        maxInputProcessingUs = processing;
    }
    inputTimingReady = maxInputIntervalUs != 0;
}

bool __not_in_flash_func(USBReportScheduler::shouldDefer)(
        USBReportPriority const &priority) const {
    USBReportPriority const &baseline = pendingOwned ? pendingPriority : deliveredPriority;
    if (priority != baseline || !inputTimingReady) {
        return false;
    }

    uint32_t const now = time_us_32();
    uint32_t const irqState = save_and_disable_interrupts();
    uint32_t const sofUs = lastSofUs;
    bool const timingReady = sofTimingReady;
    bool const pollReady = pollTimingReady;
    uint32_t const framesSincePoll = sofEpoch - lastPollSofEpoch;
    uint32_t const pollFrames = pollIntervalFrames;
    restore_interrupts(irqState);

    if (!timingReady) {
        return false;
    }

    uint32_t const phaseUs = now - sofUs;
    if (phaseUs >= USB_FULL_SPEED_FRAME_US) {
        return false;
    }

    if (!pendingOwned && pollReady) {
        if (framesSincePoll >= pollFrames) {
            return false; // The prediction expired; never wait another period.
        }
        if (framesSincePoll + 1 < pollFrames) {
            return true; // Keep analog-only data in software until the final frame.
        }
    }

    uint32_t const currentProcessingUs = now - inputStartUs;
    uint32_t reportProcessingUs = maxInputProcessingUs;
    if (currentProcessingUs > reportProcessingUs) {
        reportProcessingUs = currentProcessingUs;
    }

    uint32_t replacementUs = replacementGuardUs;
    if (!replacementTimingReady) {
        replacementUs = reportProcessingUs;
    }

    uint32_t const remainingUs = USB_FULL_SPEED_FRAME_US - phaseUs;
    if (replacementUs >= remainingUs ||
        currentProcessingUs >= maxInputIntervalUs) {
        return false;
    }

    // Defer only when the measured runtime proves that another complete
    // acquisition-to-report opportunity fits before the replacement cutoff.
    // This accounts for the loop tail and tud_task(), rather than treating the
    // shorter input-processing duration as the whole loop interval.
    uint32_t const nextReportUs =
        maxInputIntervalUs - currentProcessingUs + reportProcessingUs;
    return nextReportUs < remainingUs - replacementUs;
}

bool __not_in_flash_func(USBReportScheduler::sendDirectReport)(
        void const *report, uint16_t len, USBReportPriority const &priority,
        bool allowAnalogScheduling) {
    if (allowAnalogScheduling && shouldDefer(priority)) {
        return false;
    }

    if (tud_ready() && endpoint != 0 && !usbd_edpt_busy(0, endpoint) &&
        usbd_edpt_claim(0, endpoint)) {
        uint32_t const irqState = save_and_disable_interrupts();
        uint32_t const queueSofEpoch = sofEpoch;
        bool const replaceable = allowAnalogScheduling &&
            sofTimingReady && completedSofEpochValid &&
            completedSofEpoch == queueSofEpoch;
        restore_interrupts(irqState);

        bool const queued = usbd_edpt_xfer(
            0, endpoint, const_cast<uint8_t *>(static_cast<uint8_t const *>(report)), len, false);
        usbd_edpt_release(0, endpoint);
        if (queued) {
            uint32_t const queuedIrqState = save_and_disable_interrupts();
            noteQueued(priority, queueSofEpoch,
                       replaceable && sofEpoch == queueSofEpoch);
            restore_interrupts(queuedIrqState);
            return true;
        }
    }

    return allowAnalogScheduling && tryReplace(report, len, priority);
}

bool __not_in_flash_func(USBReportScheduler::tryReplace)(
        void const *report, uint16_t len, USBReportPriority const &priority) {
    if (!pendingOwned || !tud_ready() || endpoint == 0 ||
        priority == pendingPriority) {
        return false;
    }

    bool const samePendingTriggers =
        pendingPriority.triggers == priority.triggers;
    bool const sameDeliveredTriggers = samePendingTriggers &&
        deliveredPriority.triggers == pendingPriority.triggers;
    bool const monotonicPress = !pendingPriorityUnchanged && samePendingTriggers &&
        (pendingPriority.digital & priority.digital) == pendingPriority.digital &&
        // Do not re-add a delivered bit that the pending report released.
        (priority.digital & deliveredPriority.digital & ~pendingPriority.digital) == 0;
    bool const monotonicRelease = !pendingPriorityUnchanged && !monotonicPress &&
        sameDeliveredTriggers &&
        (pendingPriority.digital & deliveredPriority.digital) == pendingPriority.digital &&
        (priority.digital & pendingPriority.digital) == priority.digital;

    if (!pendingPriorityUnchanged && !monotonicPress && !monotonicRelease) {
        return false;
    }

    bool const replaced = replacePendingInBuffer(report, len);
    if (replaced) {
        pendingPriority = priority;
        pendingPriorityUnchanged = false;
    }
    return replaced;
}

void __not_in_flash_func(USBReportScheduler::noteQueued)(
        USBReportPriority const &priority, uint32_t queuedSofEpoch,
        bool replaceable) {
    pendingPriority = priority;
    pendingPriorityUnchanged = priority == deliveredPriority;
    pendingOwned = true;
    pendingSofEpoch = queuedSofEpoch;
    pendingSofEpochValid = replaceable;
}

void USBReportScheduler::onReportComplete() {
    if (endpoint == 0) {
        return;
    }

    uint32_t const irqState = save_and_disable_interrupts();
    completedSofEpoch = sofEpoch;
    // tud_task() may dispatch a completion after its USB frame has ended.
    // A report queued earlier must still have belonged to hardware at this
    // frame's SOF; otherwise its actual completion frame is ambiguous.
    completedSofEpochValid = sofTimingReady && pendingOwned &&
        (pendingSofEpoch == sofEpoch || reportPendingAtSof) &&
        !(usb_hw->ints & USB_INTS_DEV_SOF_BITS);
    if (completedSofEpochValid) {
        noteHostPoll(sofEpoch);
    } else {
        resetPollTiming();
    }
    restore_interrupts(irqState);

    if (pendingOwned) {
        deliveredPriority = pendingPriority;
        pendingOwned = false;
        pendingPriorityUnchanged = false;
        pendingSofEpochValid = false;
    }
}

void USBReportScheduler::onReportFailed() {
    if (endpoint == 0) {
        return;
    }

    completedSofEpochValid = false;
    uint32_t const irqState = save_and_disable_interrupts();
    resetPollTiming();
    restore_interrupts(irqState);
    if (!pendingOwned) {
        return;
    }
    pendingPriority = deliveredPriority;
    pendingOwned = false;
    pendingPriorityUnchanged = false;
    pendingSofEpochValid = false;
}

bool __no_inline_not_in_flash_func(USBReportScheduler::replacePendingInBuffer)(
        void const *report, uint16_t len) {
    uint8_t const epNum = endpoint & 0x0f;
    if (rp2040_chip_version() < 2 || !(endpoint & 0x80) || epNum == 0 ||
        epNum >= USB_NUM_ENDPOINTS || len > USB_MAX_PACKET_SIZE) {
        return false;
    }

    uint32_t const irqState = save_and_disable_interrupts();
    uint32_t const attemptStartUs = time_us_32();
    uint32_t guardUs = replacementGuardUs;
    if (!replacementTimingReady) {
        guardUs = maxInputProcessingUs;
        uint32_t const currentProcessingUs = attemptStartUs - inputStartUs;
        if (currentProcessingUs > guardUs) {
            guardUs = currentProcessingUs;
        }
    }
    uint32_t const phaseUs = attemptStartUs - lastSofUs;
    bool const inWindow = inputTimingReady && sofTimingReady &&
        pendingSofEpochValid && completedSofEpochValid &&
        pendingSofEpoch == sofEpoch && completedSofEpoch == sofEpoch &&
        !(usb_hw->ints & USB_INTS_DEV_SOF_BITS) &&
        phaseUs < USB_FULL_SPEED_FRAME_US &&
        guardUs < USB_FULL_SPEED_FRAME_US - phaseUs;
    if (!inWindow) {
        restore_interrupts(irqState);
        return false;
    }

    uint32_t const abortMask = 1u << (2 * epNum);
    hw_set_bits(&usb_hw->abort, abortMask);
    while ((usb_hw->abort_done & abortMask) != abortMask) {}

    volatile uint32_t * const bufferControl = &usb_dpram->ep_buf_ctrl[epNum].in;
    uint32_t const savedControl = *bufferControl;
    bool const sofArrived = usb_hw->ints & USB_INTS_DEV_SOF_BITS;
    bool const pending = !sofArrived &&
        (savedControl & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL)) ==
            (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL) &&
        (savedControl & USB_BUF_CTRL_LEN_MASK) == len;
    if (pending) {
        uint32_t const endpointControl = usb_dpram->ep_ctrl[epNum - 1].in;
        uint8_t * const dpramBuffer = reinterpret_cast<uint8_t *>(usb_dpram) +
            (endpointControl & 0xffffu);
        *bufferControl = 0;
        memcpy(dpramBuffer, report, len);
        *bufferControl = savedControl & ~USB_BUF_CTRL_AVAIL;
        busy_wait_at_least_cycles(12);
        *bufferControl = savedControl;
    }

    hw_clear_bits(&usb_hw->abort_done, abortMask);
    hw_clear_bits(&usb_hw->abort, abortMask);
    bool const crossedSof = usb_hw->ints & USB_INTS_DEV_SOF_BITS;
    uint32_t const replacementDurationUs = boundedElapsedUs(
        time_us_32() - attemptStartUs);
    if (replacementDurationUs > replacementGuardUs) {
        replacementGuardUs = replacementDurationUs;
    }
    if (crossedSof) {
        // The SOF callback timestamp can lag the wire boundary. If hardware
        // observes a crossing, retain the actual remaining phase as the new
        // runtime guard so the same boundary cannot be crossed again.
        uint32_t const crossedBoundaryGuardUs = boundedElapsedUs(
            USB_FULL_SPEED_FRAME_US - phaseUs);
        if (crossedBoundaryGuardUs > replacementGuardUs) {
            replacementGuardUs = crossedBoundaryGuardUs;
        }
    }
    if (pending) {
        replacementTimingReady = true;
    }
    restore_interrupts(irqState);
    return pending;
}
