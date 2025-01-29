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

#include "rev/ServoChannel.h"
#include "rev/config/ServoChannelConfig.h"

namespace rev::servohub {

class ServoChannelConfigAccessor {
    friend class ServoHubConfigAccessor;

public:
    /**
     * @return The channel's pulse range (min, center, max) in microseconds
     */
    ServoChannelConfig::PulseRange_t GetPulseRange();

    /**
     * Get the output power behavior when the channel is disabled.
     *
     * <p>When the channel is enabled [ServoChannel::SetEnabled(true)],
     * the output power to the servo follows the channel's power setting
     * [ServoChannel::SetPowered()].
     *
     * <p>When the channel is disabled [ServoChannel::SetEnabled(false)],
     * the output power to the servo follows the channel's disableBehavior.
     *
     * @return The channel's disable behavior
     */
    ServoChannelConfig::BehaviorWhenDisabled GetDisableBehavior();

private:
    explicit ServoChannelConfigAccessor(ServoChannel::ChannelId channelId,
                                        void* servoHubHandle);

private:
    ServoChannel::ChannelId m_channelId;

    void* m_servoHubHandle;
};

}  // namespace rev::servohub
