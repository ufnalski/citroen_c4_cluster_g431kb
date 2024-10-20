/*
 * mustang.h
 *
 *  Created on: Jan 9, 2024
 *      Author: user
 */

// Sources:
// https://autowp.github.io/
#ifndef INC_CITROEN_C4_H_
#define INC_CITROEN_C4_H_

#include "main.h"
#include <string.h>
#include <math.h>

#define CAN_ID_ODOMETER_TO_IPC 0xF6 // mileage cannot be decreased - do not send random patterns!
#define CAN_ID_ODOMETER_FROM_IPC 0x257 // sent couple of times at the power-up

void send_cruise_data(uint32_t delay_before_send);
void send_christmas_lights_and_gear(uint8_t gear_number,
		uint32_t delay_before_send);
void send_christmas_lights_bis(uint32_t delay_before_send);
void send_odometer_and_coolant_temp(uint8_t coolant_temp, uint32_t odometer,
		uint32_t delay_before_send);
void send_fuel_level_and_oil_temp(uint8_t fuel, uint8_t oil_temp,
		uint32_t delay_before_send);
void send_ignition_on(uint8_t lcd_brightness, uint32_t delay_before_send);
void send_rpm_and_speed(uint16_t rpm, uint16_t speed,
		uint32_t delay_before_send);

void send_maintenance(uint32_t delay_before_send);
void send_trip_computer(uint32_t delay_before_send);

#endif /* INC_CITROEN_C4_H_ */
