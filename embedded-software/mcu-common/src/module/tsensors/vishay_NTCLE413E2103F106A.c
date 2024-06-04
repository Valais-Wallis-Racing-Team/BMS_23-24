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
 * @file    vishay_NTCLE413E2103F106A.c
 * @author  foxBMS Team and Mattia_Bétrisey
 * @date    08.03.2024 (date of creation)
 * @ingroup TSENSORS
 * @prefix  NTCLE413E2103F106A
 *
 * @brief   Resistive divider used for measuring temperature used for the formula student car
 *
 */

/*================== Includes ===============================================*/
#include "vishay_NTCLE413E2103F106A.h"

#include <float.h>
#include "foxmath.h"

/*================== Macros and Definitions =================================*/

/**
 * temperature-resistance LUT for VISHAY NTCLE413E2103F106A
 */
typedef struct {
    int16_t temperature_C;
    float resistance_Ohm;
} NTCLE413E2103F106A_LUT_s;

/*================== Static Constant and Variable Definitions ===============*/

/**
 * LUT filled from higher resistance to lower resistance
 */
static const NTCLE413E2103F106A_LUT_s NTCLE413E2103F106A_LUT[] = {
    { -40, 190935.00 },
    { -35, 145935.00 },
    { -30, 112440.00 },
    { -25, 87285.00 },
    { -20, 68260.00 },
    { -15, 53762.00 },
    { -10, 42636.00 },
    {  -5, 34038.00 },
    {   0, 27348.00 },
    {   5, 22108.00 },
    {  10, 17979.00 },		//to modify
    {  15, 14706.00 },
    {  20, 12094.00 },
    {  25, 10000.00 },
    {  30, 8310.80 },
    {  35, 6941.10 },
    {  40, 5824.90 },
    {  45, 4910.60 },
    {  50, 4158.30 },
    {  55, 3536.20 },
    {  60, 3019.70 },
    {  65, 2588.80 },
    {  70, 2228.00 },
    {  75, 1924.60 },
    {  80, 1668.40 },
    {  85, 1451.30 },
    {  90, 1266.70 },
    {  95, 1109.20 },
    { 100, 974.26 },
    { 105, 858.33 }
};

static uint16_t sizeLUT = sizeof(NTCLE413E2103F106A_LUT)/sizeof(NTCLE413E2103F106A_LUT_s);


/*================== Extern Constant and Variable Definitions ===============*/
/* Defines for calculating the ADC voltage on the ends of the operating range.
 * The ADC voltage is calculated with the following formula:
 *
 * Vadc = ((Vsupply * Rntc) / (R + Rntc))
 *
 * Depending on the position of the NTC in the voltage resistor (R1/R2),
 * different Rntc values are used for the calculation.
 */
#if NTCLE413E2103F106A_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == TRUE
#define ADC_VOLTAGE_VMAX_V    (float)((NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * NTCLE413E2103F106A_LUT[sizeLUT-1].resistance_Ohm) / (NTCLE413E2103F106A_LUT[sizeLUT-1].resistance_Ohm+NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#define ADC_VOLTAGE_VMIN_V    (float)((NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * NTCLE413E2103F106A_LUT[0].resistance_Ohm) / (NTCLE413E2103F106A_LUT[0].resistance_Ohm+NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#else /*NTCLE413E2103F106A_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == FALSE */
#define ADC_VOLTAGE_VMIN_V    (float)((NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * NTCLE413E2103F106A_LUT[sizeLUT-1].resistance_Ohm) / (NTCLE413E2103F106A_LUT[sizeLUT-1].resistance_Ohm+NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#define ADC_VOLTAGE_VMAX_V    (float)((NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * NTCLE413E2103F106A_LUT[0].resistance_Ohm) / (NTCLE413E2103F106A_LUT[0].resistance_Ohm+NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#endif
/*================== Static Function Prototypes =============================*/

/*================== Static Function Implementations ========================*/

/*================== Extern Function Implementations ========================*/

extern float NTCLE413E2103F106A_GetTempFromLUT(uint16_t vadc_mV) {
    float temperature = 0.0;
    float resistance_Ohm = 0.0;
    float adcVoltage_V = vadc_mV/1000.0;    /* Convert mV to V */

    /* Check for valid ADC measurements to prevent undefined behavior */
    if (adcVoltage_V > ADC_VOLTAGE_VMAX_V) {
        /* Invalid measured ADC voltage -> sensor out of operating range or disconnected/shorted */
        temperature = -FLT_MAX;
    } else if (adcVoltage_V < ADC_VOLTAGE_VMIN_V) {
        /* Invalid measured ADC voltage -> sensor out of operating range or shorted/disconnected */
        temperature = FLT_MAX;
    } else {
        /* Calculate NTC resistance based on measured ADC voltage */
#if NTCLE413E2103F106A_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == TRUE

        /* R1 = R2*((Vsupply/Vadc)-1) */
        resistance_Ohm = NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm *
                ((NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V/adcVoltage_V) - 1);
#else /* B57861S0103F045_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == FALSE */

        /* R2 = R1*(V2/(Vsupply-Vadc)) */
        resistance_Ohm = NTCLE413E2103F106A_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm *
                (adcVoltage_V/(NTCLE413E2103F106A_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V - adcVoltage_V));
#endif /* B57861S0103F045_POSITION_IN_RESISTOR_DIVIDER_IS_R1 */

        /* Variables for interpolating LUT value */
        uint16_t between_high = 0;
        uint16_t between_low = 0;
        for (uint16_t i = 1; i < sizeLUT; i++) {
            if (resistance_Ohm < NTCLE413E2103F106A_LUT[i].resistance_Ohm) {
                between_low = i+1;
                between_high = i;
            }
        }

        /* Interpolate between LUT vales, but do not extrapolate LUT! */
        if (!((between_high == 0 && between_low == 0) ||  /* measured resistance > maximum LUT resistance */
                 (between_low > sizeLUT))) {              /* measured resistance < minimum LUT resistance */
            temperature = MATH_linearInterpolation(NTCLE413E2103F106A_LUT[between_low].resistance_Ohm,
            		NTCLE413E2103F106A_LUT[between_low].temperature_C,
					NTCLE413E2103F106A_LUT[between_high].resistance_Ohm,
					NTCLE413E2103F106A_LUT[between_high].temperature_C,
                              resistance_Ohm);
        }
    }

    /* Return temperature based on measured NTC resistance */
    return temperature;
}


extern float NTCLE413E2103F106A_GetTempFromPolynom(uint16_t vadc_mV) {
    float temperature = 0.0;
    float vadc_V = vadc_mV/1000.0;
    float vadc2 = vadc_V * vadc_V;
    float vadc3 = vadc2 * vadc_V;
    float vadc4 = vadc3 * vadc_V;
    float vadc5 = vadc4 * vadc_V;

    /* 5th grade polynomial for VISHAY NTCLE413E2103F106A NTC-Thermistor, 10 kOhm, Series NTCLE413E2103F106A, Vref = 3V, R in series 10k */
    temperature = -6.2765f*vadc5 + 49.0397f*vadc4 - 151.3602f*vadc3 + 233.2521f*vadc2 - 213.4588f*vadc_V + 130.5822f;

    return temperature;
}
