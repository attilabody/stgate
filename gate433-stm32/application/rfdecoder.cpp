/*
 * RFDecoder.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: compi
 */

#include <tim.h>
#include <gpio.h>
#include <rfdecoder.h>

////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef RFDecoder::Init(IDecoderCallback &callback)
{
	HAL_StatusTypeDef	res = HAL_OK;

	m_callback = &callback;

	if((res = HAL_TIM_Base_Start_IT(&htim1)) != HAL_OK)
		return res;
	if((res = HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1)) != HAL_OK)
		return res;
	return HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
}

////////////////////////////////////////////////////////////////////
void RFDecoder::PeriodEllapsed(TIM_HandleTypeDef *htim)
{
	if(m_overflow < 0xff)
		++m_overflow;
}

////////////////////////////////////////////////////////////////////
void RFDecoder::CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t	length = m_overflow;
	uint16_t	currentCapture;
	bool		level;

	switch(htim->Channel)
	{
	case HAL_TIM_ACTIVE_CHANNEL_1:
		currentCapture = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
		level = true;
		break;

	case HAL_TIM_ACTIVE_CHANNEL_2:
		currentCapture = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
		level = false;
		break;

	default:
		return;
	}

	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) ++length;
	if (currentCapture < m_lastCapture) --length;
	length <<= 16;
	length += (uint16_t)(currentCapture - m_lastCapture);

	m_sync = ProcessPeriod(level, length);

	m_lastCapture = currentCapture;
	m_lastLength = length;
	m_overflow = 0;
}

////////////////////////////////////////////////////////////////////
uint8_t RFDecoder::ProcessPeriod(bool level, uint32_t length)
{
	if (length >= SYNC_PAUSE_MIN) {
		if (level || length > SYNC_PAUSE_MAX) return SYNC_PAUSE_WAIT;
		if (m_sync == SYNC_OK && m_bits == 12) {
			m_lastDecoded = m_code;
			if (m_callback) m_callback->CodeReceived(m_lastDecoded);
		}
		return SYNC_SHORT_WAIT;
	}
	if (m_sync == SYNC_OK) {
		if (length > BIT_TIME_MAX) return SYNC_PAUSE_WAIT;
		if (level) {
			uint32_t bit_time = m_lastLength + length;
			if (bit_time < BIT_TIME_MIN || bit_time > BIT_TIME_MAX) return SYNC_PAUSE_WAIT;
			if (length > m_lastLength)  //	1
				m_code |= 1 << m_bits;
			++m_bits;
		}
		return SYNC_OK;
	}
	if (m_sync == SYNC_SHORT_WAIT && level && length >= SYNC_SHORT_MIN && length <= SYNC_SHORT_MAX) {
		m_code = 0;
		m_bits = 0;
		return SYNC_OK;
	}
	return SYNC_PAUSE_WAIT;
}

////////////////////////////////////////////////////////////////////
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	RFDecoder::Instance().CaptureCallback(htim);
}
