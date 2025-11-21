/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @author Chen HuaMing <337467729@qq.com>
 */

/**
 * JOINT CAN mode
 *
 *  0 - JOINT CAN disabled.
 *  1 - Enables support for JOINT CAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for JOINT CAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for JOINT CAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to JOICAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_ENABLE, 0);

/**
 * JOINT CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_BITRATE, 1000000);

/**
 * JOINT CAN 1 servo 1 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C1S1_OFF, 0.0);

/**
 * JOINT CAN 1 servo 2 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C1S2_OFF, 0.0);

/**
 * JOINT CAN 1 servo 3 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C1S3_OFF, 0.0);

/**
 * JOINT CAN 1 servo 4 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C1S4_OFF, 0.0);

/**
 * JOINT CAN 2 servo 1 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C2S1_OFF, 0.0);

/**
 * JOINT CAN 2 servo 2 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C2S2_OFF, 0.0);

/**
 * JOINT CAN 2 servo 3 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C2S3_OFF, 0.0);

/**
 * JOINT CAN 2 servo 4 zero offset
 *
 * @unit rad
 * @min -2.0000
 * @max 2.0000
 * @decimal 10
 * @increment 0.0001
 * @group JOINT_CAN
 */
PARAM_DEFINE_FLOAT(JOICAN_C2S4_OFF, 0.0);

/**
 * JOINT CAN 1 servo 1 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C1S1_REV, 0);

/**
 * JOINT CAN 1 servo 2 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C1S2_REV, 0);

/**
 * JOINT CAN 1 servo 3 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C1S3_REV, 0);

/**
 * JOINT CAN 1 servo 4 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C1S4_REV, 0);

/**
 * JOINT CAN 2 servo 1 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C2S1_REV, 0);

/**
 * JOINT CAN 2 servo 2 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C2S2_REV, 0);

/**
 * JOINT CAN 2 servo 3 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C2S3_REV, 0);

/**
 * JOINT CAN 2 servo 4 reverse setpoint
 *
 *  0 - Not Reverse.
 *  1 - Reverse.
 *
 * @min 0
 * @max 1
 * @value 0 Not Reverse
 * @value 1 Reverse
 * @group JOINT_CAN
 */
PARAM_DEFINE_INT32(JOICAN_C2S4_REV, 0);

/**
* Low pass filter cutoff frequency for JOINT CAN servo
*
* The cutoff frequency for the 2nd order butterworth filter on the servos.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group JOINT_CAN
*/
PARAM_DEFINE_FLOAT(JOICAN_SV_CUTOFF, 100.0f);
