/*
 * Copyright (c) 2021-2024 REV Robotics
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

#include <stdint.h>

#include <atomic>

#include "rev/REVLibError.h"
#include "rev/RelativeEncoder.h"

namespace rev::spark {

class SparkBase;

/**
 * Get an instance of this class by using SparkBase::GetEncoder() or
 * SparkBase::GetEncoder(SparkRelativeEncoder::Type, int).
 */
class SparkRelativeEncoder : public RelativeEncoder {
    // Friend to allow construction
    friend class SparkBase;

public:
    SparkRelativeEncoder(SparkRelativeEncoder&& rhs) = default;
    SparkRelativeEncoder& operator=(SparkRelativeEncoder&& rhs) = default;

    SparkRelativeEncoder(const SparkRelativeEncoder& rhs) = default;

    ~SparkRelativeEncoder() override = default;

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     *
     */
    double GetPosition() const override;

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'RPM' by default, and can be changed by a scale factor
     * using setVelocityConversionFactor().
     *
     * @return Number the RPM of the motor
     *
     */
    double GetVelocity() const override;

    /**
     * Set the position of the encoder.
     *
     * @param position Number of rotations of the motor
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetPosition(double position) override;

private:
    SparkBase* m_device;

    explicit SparkRelativeEncoder(SparkBase& device);
};

}  // namespace rev::spark
