/*
 * wiegand.cpp
 *
 *  Created on: 2019-07-20
 *      Author: Dabi
 */

#include "wiegand.h"

HAL_StatusTypeDef  Wiegand::Init(
	TIM_HandleTypeDef *htim,
	GPIO_TypeDef *outD1Port, uint16_t outD1Pin,
	GPIO_TypeDef *outD0Port, uint16_t outD0Pin,
	GPIO_TypeDef *inD1Port,	uint16_t inD1Pin,
	GPIO_TypeDef *inD0Port, uint16_t inD0Pin,
	uint8_t facility) {
	m_outD1Port=outD1Port;
	m_outD0Port=outD0Port;
	m_inD1Port=inD1Port;
	m_inD0Port=inD0Port;
	m_outD1Pin=outD1Pin;
	m_outD0Pin=outD0Pin;
	m_inD1Pin=inD1Pin;
	m_inD0Pin=inD0Pin;
	m_facility=static_cast<uint32_t>(facility) << 16;
	return HAL_TIM_Base_Start_IT(htim);
}

void Wiegand::SetCode(DIR dir, uint32_t code) {
	uint32_t i = 0;
	uint32_t mask = 1 << 24;

	code |= m_facility;

	code <<= 1;

	bool parity = false;		// 26..13 = Even parity
	while (i++ < 12) {
		if (code & mask)
			parity = !parity;
		mask >>= 1;
	}
	if (parity)
		code |= 1 << 25;

	i = 0;
	parity = true;				// 26..13 = Odd parity
	while (i++ < 12) {
		if (code & mask)
			parity = !parity;
		mask >>= 1;
	}
	if (parity)
		code |= 1;

	code <<= 6;
	if (dir == IN) {
		m_inCode = code;
		m_inLeft = 26;
	} else {
		m_outCode = code;
		m_outLeft = 26;
	}
}

void Wiegand::TimerUpdateIT(TIM_HandleTypeDef *htim) {
	bool start = false;

	if (m_outLeft) {
		if (m_outCode & 0x80000000) {
			HAL_GPIO_WritePin(m_outD1Port, m_outD1Pin,GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(m_outD0Port, m_outD0Pin,GPIO_PIN_RESET);
		}
		m_outCode <<= 1;
		m_outLeft--;
		start = true;
	}

	if (m_inLeft) {
		if (m_inCode & 0x80000000) {
			HAL_GPIO_WritePin(m_inD1Port, m_inD1Pin,GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(m_inD0Port, m_inD0Pin,GPIO_PIN_RESET);
		}
		m_inCode <<= 1;
		m_inLeft--;
		start = true;
	}

	if (start) {
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
		__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
	}
}

void Wiegand::TimerPWMIT(TIM_HandleTypeDef *htim) {
	HAL_GPIO_WritePin(m_outD1Port, m_outD1Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(m_outD0Port, m_outD0Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(m_inD1Port, m_inD1Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(m_inD0Port, m_inD0Pin,GPIO_PIN_SET);
	__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
}

