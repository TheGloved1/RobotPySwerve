/*
 * Copyright (c) 2024 REV Robotics
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

#ifndef FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKRELATIVEENCODERSIM_H_
#define FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKRELATIVEENCODERSIM_H_

#include <string>

#include <frc/simulation/SimDeviceSim.h>
#include <hal/SimDevice.h>

#include "rev/SparkBase.h"
#include "rev/SparkFlex.h"
#include "rev/SparkMax.h"

namespace rev::spark {

class SparkRelativeEncoderSim {
public:
    explicit SparkRelativeEncoderSim(SparkMax* motor);
    explicit SparkRelativeEncoderSim(SparkFlex* motor);

    void SetPosition(double position);
    double GetPosition() const;

    void SetVelocity(double velocity);
    double GetVelocity() const;

    void SetInverted(bool inverted);
    bool GetInverted() const;

    void SetZeroOffset(double zeroOffset);
    double GetZeroOffset() const;

    void SetPositionConversionFactor(double positionConversionFactor);
    double GetPositionConversionFactor() const;

    void SetVelocityConversionFactor(double velocityConversionFactor);
    double GetVelocityConversionFactor() const;

    void iterate(double velocity, double dt);

private:
    void SetupSimDevice();

    hal::SimDouble m_position;
    hal::SimDouble m_velocity;
    hal::SimBoolean m_isInverted;
    hal::SimDouble m_zeroOffset;
    hal::SimDouble m_positionConversionFactor;
    hal::SimDouble m_velocityConversionFactor;
    SparkBase* m_spark;
    std::string simDeviceName;
};

}  // namespace rev::spark

#endif  // FRC_REVLIB_SRC_MAIN_NATIVE_INCLUDE_REV_SIM_SPARKRELATIVEENCODERSIM_H_
