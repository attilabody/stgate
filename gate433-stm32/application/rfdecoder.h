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

	// Ref time: 16250 us +- 3250 us (+- 20%)
	static const uint32_t SYNC_PAUSE_MIN = RF_US_TO_TICK(16250 - 3250);
	static const uint32_t SYNC_PAUSE_MAX = RF_US_TO_TICK(16250 + 3250);
	// Ref time: 550 us +- 150 us (+- 27%)
	static const uint32_t SYNC_SHORT_MIN = RF_US_TO_TICK(550 - 150);
	static const uint32_t SYNC_SHORT_MAX = RF_US_TO_TICK(550 + 150);
	// Ref time: 1380 us +- 280 us (+- 20%)
	static const uint32_t BIT_TIME_MIN = RF_US_TO_TICK(1380 - 280);
	static const uint32_t BIT_TIME_MAX = RF_US_TO_TICK(1380 + 280);

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
