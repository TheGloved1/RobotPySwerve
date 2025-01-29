/*
 * Copyright (c) 2018-2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#ifdef _MSC_VER
// Disable deprecation warnings for this file when using VS compiler
#pragma warning(disable : 4996)
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <wpi/deprecated.h>

#include "rev/ClosedLoopSlot.h"
#include "rev/REVLibError.h"
#include "rev/SparkLowLevel.h"

namespace rev::spark {

class SparkBase;

class SparkClosedLoopController {
    friend class SparkBase;

public:
    /**
     * Units for arbitrary feed-forward
     */
    enum class ArbFFUnits { kVoltage = 0, kPercentOut = 1 };

    SparkClosedLoopController(SparkClosedLoopController&&) = default;
    SparkClosedLoopController& operator=(SparkClosedLoopController&&) = default;

    SparkClosedLoopController(const SparkClosedLoopController& rhs) = default;

    ~SparkClosedLoopController() = default;

    /**
     * Set the controller reference value based on the selected control mode.
     *
     * @param value The value to set depending on the control mode. For basic
     * duty cycle control this should be a value between -1 and 1
     * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     * (RPM) Position Control: Position (Rotations) Current Control: Current
     * (Amps). The units can be changed for position and velocity by a scale
     * factor using setPositionConversionFactor().
     *
     * @param ctrl Is the control type
     *
     * @param slot The ClosedLoopSlot to use
     *
     * @param arbFeedforward A value from -32.0 to 32.0 which is a voltage
     * applied to the motor after the result of the specified control mode. The
     * units for the parameter is Volts. This value is set after the control
     * mode, but before any current limits or ramp rates.
     *
     * @param arbFFUnits the units for arbitrary feed-forward
     * @return REVLibError::kOk if successful
     */
    REVLibError SetReference(double value, SparkLowLevel::ControlType ctrl,
                             ClosedLoopSlot slot = ClosedLoopSlot::kSlot0,
                             double arbFeedforward = 0,
                             SparkClosedLoopController::ArbFFUnits arbFFUnits =
                                 ArbFFUnits::kVoltage);

    /**
     * Set the I accumulator of the closed loop controller. This is useful when
     * wishing to force a reset on the I accumulator of the Closed Loop
     * Controller. You can also preset values to see how it will respond to
     * certain I characteristics
     *
     * To use this function, the controller must be in a closed loop control
     * mode by calling setReference()
     *
     * @param iAccum The value to set the I accumulator to
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetIAccum(double iAccum);

    /**
     * Get the I accumulator of the closed loop controller. This is useful when
     * wishing to see what the I accumulator value is to help with PID tuning
     *
     * @return The value of the I accumulator
     */
    double GetIAccum() const;

private:
    SparkBase* m_device;

    explicit SparkClosedLoopController(SparkBase& device);
};

}  // namespace rev::spark

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
