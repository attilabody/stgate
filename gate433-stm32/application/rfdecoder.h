/*
 * RFDecoder.h
 *
 *  Created on: Jan 13, 2018
 *      Author: compi
 */

#ifndef RFDECODER_H_
#define RFDECODER_H_

#include "stm32f1xx_hal.h"
#include <sg/Singleton.h>

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

extern "C" TIM_HandleTypeDef htim1;

// TIM1 @ 24MHz
#define RF_US_TO_TICK(us) ((us) * 24)

class RFDecoder : public sg::Singleton<RFDecoder>
{
	friend class Singleton<RFDecoder>;
	friend void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
	friend void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
public:
	struct IDecoderCallback {
		virtual void CodeReceived(uint16_t code) = 0;
	};
	HAL_StatusTypeDef Init(IDecoderCallback &callback);

private:
	RFDecoder() = default;

	void PeriodEllapsed(TIM_HandleTypeDef *htim);
	void CaptureCallback(TIM_HandleTypeDef *htim);

	uint8_t ProcessPeriod(bool level, uint32_t length);

	TIM_HandleTypeDef &m_htim = htim1;

	IDecoderCallback *m_callback = nullptr;

	// Ref time: 14350 us +- 5023 us (+- 35%)
	static const uint32_t SYNC_PAUSE_MIN = RF_US_TO_TICK(14350 - 5023);
	static const uint32_t SYNC_PAUSE_MAX = RF_US_TO_TICK(14350 + 5023);
	// Ref time: 500 us +- 200 us (+- 40%)
	static const uint32_t SYNC_SHORT_MIN = RF_US_TO_TICK(500 - 200);
	static const uint32_t SYNC_SHORT_MAX = RF_US_TO_TICK(500 + 200);
	// Ref time: 1250 us +- 437 us (+- 35%)
	static const uint32_t BIT_TIME_MIN = RF_US_TO_TICK(1250 - 437);
	static const uint32_t BIT_TIME_MAX = RF_US_TO_TICK(1250 + 437);

	uint32_t	m_lastLength = 0;

	uint16_t	m_lastCapture = 0;
	uint16_t	m_code, m_bits;
	uint16_t	m_lastDecoded;

	uint8_t		m_overflow = 0;
	uint8_t		m_sync = 0;
	static const uint8_t SYNC_PAUSE_WAIT = 0;
	static const uint8_t SYNC_SHORT_WAIT = 1;
	static const uint8_t SYNC_OK = 2;
};

#endif /* __cplusplus */
#endif /* RFDECODER_H_ */
