/**
 *
 * @copyright &copy; 2010 - 2021, Fraunhofer-Gesellschaft zur Foerderung der
 *  angewandten Forschung e.V. All rights reserved.
 *
 * BSD 3-Clause License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 3.  Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer
 * to foxBMS in your hardware, software, documentation or advertising
 * materials:
 *
 * &Prime;This product uses parts of foxBMS&reg;&Prime;
 *
 * &Prime;This product includes parts of foxBMS&reg;&Prime;
 *
 * &Prime;This product is derived from foxBMS&reg;&Prime;
 *
 */

/**
 * @file    batterycell_cfg.h
 * @author  foxBMS Team
 * @date    14.03.2017 (date of creation)
 * @ingroup BATTERY_CELL_CONF
 * @prefix  BC
 *
 * @brief   Configuration of the battery cell (e.g., minimum and maximum cell voltage)
 *
 * This files contains basic macros of the battery cell in order to derive needed inputs
 * in other parts of the software. These macros are all depended on the hardware.
 *
 */

#ifndef BATTERYCELL_CFG_H_
#define BATTERYCELL_CFG_H_


#if BUILD_MODULE_IMPORT_CELL_DATASHEET == 1

#include "LG_INR18650MJ1.h"

#else


/**
 * @ingroup CONFIG_BATTERYCELL
 * Maximum temperature limit during discharge.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * &deg;C
 * \par Default:
 * 55
*/
#define BC_TEMPMAX_DISCHARGE_MSL      60
#define BC_TEMPMAX_DISCHARGE_RSL      55
#define BC_TEMPMAX_DISCHARGE_MOL      55

/**
 * @ingroup CONFIG_BATTERYCELL
 * Minimum temperature limit during discharge.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * &deg;C
 * \par Default:
 * -20
*/
#define BC_TEMPMIN_DISCHARGE_MSL      -20
#define BC_TEMPMIN_DISCHARGE_RSL      -15
#define BC_TEMPMIN_DISCHARGE_MOL      -10

/**
 * @ingroup CONFIG_BATTERYCELL
 * Maximum temperature limit during charge.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * &deg;C
 * \par Default:
 * 45
*/
#define BC_TEMPMAX_CHARGE_MSL     45
#define BC_TEMPMAX_CHARGE_RSL     40
#define BC_TEMPMAX_CHARGE_MOL     35

/**
 * @ingroup CONFIG_BATTERYCELL
 * Minimum temperature limit during discharge.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * &deg;C
 * \par Default:
 * 0
*/
#define BC_TEMPMIN_CHARGE_MSL     -20
#define BC_TEMPMIN_CHARGE_RSL     -15
#define BC_TEMPMIN_CHARGE_MOL     -10

/**
 * @ingroup CONFIG_BATTERYCELL
 * Maximum cell voltage limit.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * mV
 * \par Default:
 * 2800
*/
#define BC_VOLTMAX_MSL      4200
#define BC_VOLTMAX_RSL      4100
#define BC_VOLTMAX_MOL      4100

/**
 * @ingroup CONFIG_BATTERYCELL
 * nominal cell voltage according to datasheet
 * \par Type:
 * int
 * \par Unit:
 * mV
 * \par Default:
 * 2500
*/
#define BC_VOLT_NOMINAL     3600

/**
 * @ingroup CONFIG_BATTERYCELL
 * Minimum cell voltage limit.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * mV
 * \par Default:
 * 1700
*/
#define BC_VOLTMIN_MSL      2800
#define BC_VOLTMIN_RSL      3000
#define BC_VOLTMIN_MOL      3000

/**
 * @ingroup CONFIG_BATTERYCELL
 * Deep-discharge cell voltage limit.
 * If this voltage limit is violated, the cell is faulty. The BMS won't allow
 * a closing of the contactors until this cell is replaced. a replacement of
 * the cell is confirmed by sending the respective CAN debug message
 * \par Type:
 * int
 * \par Unit:
 * mV
 * \par Default:
 * BC_VOLTMIN_MSL
*/
#define BC_VOLT_DEEP_DISCHARGE          BC_VOLTMIN_MSL

/**
 * @ingroup CONFIG_BATTERYCELL
 * Maximum discharge current limit.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * mA
 * \par Default:
 * 180000
*/
#define BC_CURRENTMAX_DISCHARGE_MSL     180000
#define BC_CURRENTMAX_DISCHARGE_RSL     150000
#define BC_CURRENTMAX_DISCHARGE_MOL     150000

/**
 * @ingroup CONFIG_BATTERYCELL
 * Maximum charge current limit.
 * When maximum safety limit (MSL) is violated, error state is requested and
 * contactors will open. When recommended safety limit (RSL) or maximum
 * operating limit (MOL) is violated, the respective flag will be set.
 * \par Type:
 * int
 * \par Unit:
 * mA
 * \par Default:
 * 180000
*/
#define BC_CURRENTMAX_CHARGE_MSL        18000
#define BC_CURRENTMAX_CHARGE_RSL        15000
#define BC_CURRENTMAX_CHARGE_MOL        15000

/*
 * the cell capacity used for SOC calculation, in this case Ah counting
 * @type int
 * @unit mAh
 * @default 3500
 * @group
 */
#define BC_CAPACITY 18000

#if BC_VOLTMIN_MSL < BC_VOLT_DEEP_DISCHARGE
#error "Configuration error! - Maximum safety limit for under voltage can't be lower than deep-discharge limit"
#endif

#endif

#endif /* BATTERYCELL_CFG_H_ */
