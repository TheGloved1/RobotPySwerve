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

#pragma once

#include "rev/config/BaseConfig.h"

namespace rev::spark {

class LimitSwitchConfig : public BaseConfig {
public:
    enum Type { kNormallyOpen = 0, kNormallyClosed = 1 };

    LimitSwitchConfig() = default;
    ~LimitSwitchConfig() override = default;

    LimitSwitchConfig(const LimitSwitchConfig&) = delete;
    LimitSwitchConfig& operator=(const LimitSwitchConfig&) = delete;
    LimitSwitchConfig(LimitSwitchConfig&&) noexcept = delete;
    LimitSwitchConfig& operator=(LimitSwitchConfig&&) noexcept = delete;

    /**
     * Applies settings from another LimitSwitchConfig to this one.
     *
     * <p>Settings in the provided config will overwrite existing values in this
     * object. Settings not specified in the provided config remain unchanged.
     *
     * @param config The LimitSwitchConfig to copy settings from
     * @return The updated LimitSwitchConfig for method chaining
     */
    LimitSwitchConfig& Apply(LimitSwitchConfig& config);

    /**
     * Configures the data port to use limit switches, which is specifically
     * required for SPARK MAX.
     *
     * <p>NOTE: This method is only necessary when using limit switches with a
     * SPARK MAX without configuring any of its settings
     *
     * <p>IMPORTANT: SPARK MAX does not support using limit switches in
     * conjunction with an alternate encoder.
     *
     * @return The modified LimitSwitchConfig object for method chaining
     */
    LimitSwitchConfig& SetSparkMaxDataPortConfig();

    /**
     * Set whether to enable or disable motor shutdown based on the forward
     * limit switch state. This does not not affect the result of the
     * isPressed() command.
     *
     * @param enabled True to enable motor shutdown behavior
     * @return The modified LimitSwitchConfig object for method chaining
     */
    LimitSwitchConfig& ForwardLimitSwitchEnabled(bool enabled);

    /**
     * Set the normal state of the forward limit switch.
     *
     * @param type kNormallyOpen or kNormallyClosed
     * @return The modified LimitSwitchConfig object for method chaining
     */
    LimitSwitchConfig& ForwardLimitSwitchType(Type type);

    /**
     * Set whether to enable or disable motor shutdown based on the reverse
     * limit switch state. This does not not affect the result of the
     * isPressed() command.
     *
     * @param enabled True to enable motor shutdown behavior
     * @return The modified LimitSwitchConfig object for method chaining
     */
    LimitSwitchConfig& ReverseLimitSwitchEnabled(bool enabled);

    /**
     * Set the normal state of the reverse limit switch.
     *
     * @param type kNormallyOpen or kNormallyClosed
     * @return The modified LimitSwitchConfig object for method chaining
     */
    LimitSwitchConfig& ReverseLimitSwitchType(Type type);
};

}  // namespace rev::spark
