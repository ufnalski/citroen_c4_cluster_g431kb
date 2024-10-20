/*
 * focus.c
 *
 *  Created on: Jan 9, 2024
 *      Author: user
 *
 *  Sources:
 *  https://autowp.github.io/
 *
 */

#include "citroen_c4.h"
#include "fdcan.h"

extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData8[8];

//----------------------------------- 0x128 (many indicators) ---------------------------
void send_christmas_lights_and_gear(uint8_t gear_number,
		uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x128;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0xFF;

	TxData8[1] = 0x00;
	switch (gear_number)
	{
	case 0:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b10010000; // 1st gear
		break;
	case 1:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b10000000; // 2nd gear
		break;
	case 2:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b01110000; // 3rd gear
		break;
	case 3:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b01100000; // 4th gear
		break;
	case 4:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b01010000; // 5th gear
		break;
	case 5:
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b01000000; // 6th gear
		break;
	case 'P':
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b00000000; // P
		break;
	case 'R':
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b00010000; // R
		break;
	case 'N':
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b00100000; // N
		break;
	case 'D':
		TxData8[1] = (TxData8[1] & 0b00001111) | 0b00110000; // D
		break;
	default:
		TxData8[1] = (TxData8[1] & 0b11110000);
	}

	TxData8[2] = 0xFF;
	TxData8[3] = 0xFF;
	TxData8[4] = 0xFF;
	TxData8[5] = 0xFF;
	TxData8[6] = 0xFF;
	TxData8[7] = 0xFF;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ------------------------------------------- 0x168 (many indicators) --------------
void send_christmas_lights_bis(uint32_t delay_before_send)
{

	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x168;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0xff;
	TxData8[1] = 0xff;
	TxData8[2] = 0xff;
	TxData8[3] = 0xff;
	TxData8[4] = 0xff;
	TxData8[5] = 0xff;
	TxData8[6] = 0xff;
	TxData8[7] = 0xff;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}

}

// ------------------------------------------------------- 0x36 (ignition) -----------------
void send_ignition_on(uint8_t lcd_brightness, uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x36;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0b00001110;  // ignition on -> activates arrows!!!
	TxData8[1] = 0;
	TxData8[2] = 0;
	TxData8[3] = 0b00100000 | lcd_brightness;
	TxData8[4] = 0b00000001; // ignition
	TxData8[5] = 0;
	TxData8[6] = 0;
	TxData8[7] = 0b10100000;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ---------------------------------------------- 0x1A8 (cruise data) -----------
void send_cruise_data(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x1A8;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x55;
	TxData8[1] = 0x55;
	TxData8[2] = 0x55;
	TxData8[3] = 0x55;
	TxData8[4] = 0x55;
	TxData8[5] = 0x04; // trip distance
	TxData8[6] = 0x55;
	TxData8[7] = 0x55;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ------------------------------------------- 0xB6 (rpm and ground speed) ------------------
void send_rpm_and_speed(uint16_t rpm, uint16_t speed,
		uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0xB6;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = (uint8_t) (rpm >> 5);
	TxData8[1] = (uint8_t) (rpm << 3);
	TxData8[2] = (uint8_t) ((speed * 100) >> 8);
	TxData8[3] = (uint8_t) (speed * 100);
	TxData8[4] = 0x00;
	TxData8[5] = 0x00;
	TxData8[6] = 0x00;
	TxData8[7] = 0x00; //0xD0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ------------------------------------------------ 0x161 (fuel level and oil temp) -----------
void send_fuel_level_and_oil_temp(uint8_t fuel, uint8_t oil_temp,
		uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_7;
	TxHeader.Identifier = 0x161;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0;
	TxData8[1] = 0;
	TxData8[2] = oil_temp + 39; // wild guess
	TxData8[3] = fuel;
	TxData8[4] = 0;
	TxData8[5] = 0;
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// --------------------------------------- 0xF6 (coolant temp and odometer) ---------
void send_odometer_and_coolant_temp(uint8_t coolant_temp, uint32_t odometer,
		uint32_t delay_before_send)
{
	coolant_temp += 39;  // incorrect formula?
	odometer *= 10;
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = CAN_ID_ODOMETER_TO_IPC;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x8E; // 0x8e;  //0x88;
	TxData8[1] = coolant_temp;

	// Be careful when paying with this frame - you cannot decrease the displayed value (contrary to e.g. Peugeot 308 T9).
	TxData8[2] = (uint8_t) (odometer >> 16);
	TxData8[3] = (uint8_t) (odometer >> 8);
	TxData8[4] = (uint8_t) odometer;
	TxData8[5] = 0x8E; //0xAA; //0x6f;
	TxData8[6] = 0xAA; //(uint8_t) round(25.0 / 2.0 - 39.5);
	TxData8[7] = 0x00; //0xAA; //0x23;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ------------------------------------------ 0x3E7 (maintenance(?)) ---------
void send_maintenance(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_5;
	TxHeader.Identifier = 0x3E7;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x40;
	TxData8[1] = 0x00;
	TxData8[2] = 0xFE;
	TxData8[3] = 0x00;
	TxData8[4] = 0xAA;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// ------------------------------------------ 0x221 (trip computer) ---------
void send_trip_computer(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_7;
	TxHeader.Identifier = 0x221;
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x80;
	TxData8[1] = 0x00;
	TxData8[2] = 0x00;
	TxData8[3] = 0x00;
	TxData8[4] = 0xFF;
	TxData8[5] = 0x00;
	TxData8[6] = 0xFF;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}
