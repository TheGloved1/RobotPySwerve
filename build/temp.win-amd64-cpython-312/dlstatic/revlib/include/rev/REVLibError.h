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

namespace rev {

enum class REVLibError {
    kOk = 0,
    kError = 1,
    kTimeout = 2,
    kNotImplemented = 3,
    kHALError = 4,
    kCantFindFirmware = 5,
    kFirmwareTooOld = 6,
    kFirmwareTooNew = 7,
    kParamInvalidID = 8,
    kParamMismatchType = 9,
    kParamAccessMode = 10,
    kParamInvalid = 11,
    kParamNotImplementedDeprecated = 12,
    kFollowConfigMismatch = 13,
    kInvalid = 14,
    kSetpointOutOfRange = 15,
    kUnknown = 16,
    kCANDisconnected = 17,
    kDuplicateCANId = 18,
    kInvalidCANId = 19,
    kSparkMaxDataPortAlreadyConfiguredDifferently = 20,
    kSparkFlexBrushedWithoutDock = 21,
    kInvalidBrushlessEncoderConfiguration = 22,
    kFeedbackSensorIncompatibleWithDataPortConfig = 23,
    kParamInvalidChannel = 24,
    kParamInvalidValue = 25,
    kCannotPersistParametersWhileEnabled = 26,
};

}  // namespace rev
