/*
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: Copyright (c) 2026 OpenStickCommunity (gp2040-ce.info)
 */

#include "drivers/shared/USBReportScheduler.h"

#include "tusb.h"
#include "device/usbd_pvt.h"

#include "pico/platform.h"
#include "pico/time.h"

#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "hardware/sync.h"

#include <cstring>

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

void USBReportScheduler::configure(uint8_t endpoint, uint32_t pollIntervalUs) {
    this->endpoint = endpoint;
    this->pollIntervalUs = pollIntervalUs;
    reset();
}

void USBReportScheduler::start() {
    if (endpoint != 0) {
        tud_sof_isr_set(&USBReportScheduler::onSof);
    }
}

void USBReportScheduler::reset() {
    uint32_t const irqState = save_and_disable_interrupts();
    lastSofUs = 0;
    sofEpoch = 0;
    sofTimingReady = false;
    reportPendingAtSof = false;
    restore_interrupts(irqState);

    inputStartUs = 0;
    maxInputIntervalUs = 0;
    maxInputProcessingUs = 0;
    replacementGuardUs = 0;
    completedSofEpoch = 0;
    pendingSofEpoch = 0;
    deliveredPriority = {};
    pendingPriority = {};
    pendingOwned = false;
    pendingPriorityUnchanged = false;
    inputTimingStarted = false;
    inputTimingReady = false;
    replacementTimingReady = false;
    completedSofEpochValid = false;
    pendingSofEpochValid = false;
}

void __not_in_flash_func(USBReportScheduler::onSof)(uint32_t frameCount) {
    (void)frameCount;

    USBReportScheduler &scheduler = getInstance();
    uint8_t const epNum = scheduler.endpoint & 0x0f;
    scheduler.reportPendingAtSof = epNum != 0 && epNum < USB_NUM_ENDPOINTS &&
        (usb_dpram->ep_buf_ctrl[epNum].in & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL)) ==
            (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL);
    scheduler.lastSofUs = time_us_32();
    ++scheduler.sofEpoch;
    scheduler.sofTimingReady = true;
}

void __not_in_flash_func(USBReportScheduler::beginInputProcessing)() {
    if (endpoint == 0) {
        return;
    }

    uint32_t const now = time_us_32();
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
        USBReportPriority const &priority, uint32_t now) const {
    USBReportPriority const &baseline = pendingOwned ? pendingPriority : deliveredPriority;
    if (priority != baseline || !inputTimingReady) {
        return false;
    }

    uint32_t const irqState = save_and_disable_interrupts();
    uint32_t const sofUs = lastSofUs;
    uint32_t const sofIntervalUs = pollIntervalUs;
    bool const timingReady = sofTimingReady;
    restore_interrupts(irqState);

    if (!timingReady) {
        return false;
    }

    uint32_t const phaseUs = now - sofUs;
    if (phaseUs >= sofIntervalUs) {
        return false;
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

    uint32_t const remainingUs = sofIntervalUs - phaseUs;
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
    uint32_t const now = time_us_32();
    if (allowAnalogScheduling && shouldDefer(priority, now)) {
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
            0, endpoint, const_cast<uint8_t *>(static_cast<uint8_t const *>(report)), len);
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
        phaseUs < pollIntervalUs &&
        guardUs < pollIntervalUs - phaseUs;
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
            pollIntervalUs - phaseUs);
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
