/*
 * wiegand.h
 *
 *  Created on: 2019-07-20
 *      Author: Dabi
 */

#ifndef WIEGAND_H_
#define WIEGAND_H_

#include "stm32f1xx_hal.h"
#include <sg/Singleton.h>

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Wiegand : public sg::Singleton<Wiegand>
{
	friend class Singleton<Wiegand>;
	friend void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
	friend void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
  public:
	~Wiegand() = default;
	HAL_StatusTypeDef Init(
		TIM_HandleTypeDef *htim,
		GPIO_TypeDef *outD1Port, uint16_t outD1Pin,
		GPIO_TypeDef *outD0Port, uint16_t outD0Pin,
		GPIO_TypeDef *inD1Port, uint16_t inD1Pin,
		GPIO_TypeDef *inD0Port, uint16_t inD0Pin,
		uint8_t facility
	);
	void SetCode(bool inner, uint32_t code);
	void TimerUpdateIT(TIM_HandleTypeDef *htim);
	void TimerPWMIT(TIM_HandleTypeDef *htim);
  private:
	Wiegand() = default;

	GPIO_TypeDef	*m_outD1Port, *m_outD0Port, *m_inD1Port, *m_inD0Port;
	uint16_t		m_outD1Pin, m_outD0Pin, m_inD1Pin, m_inD0Pin;

	uint32_t m_inCode=0;
	uint8_t m_inLeft=0;
	uint32_t m_outCode=0;
	uint8_t m_outLeft=0;
	uint32_t m_facility=0;
};

#endif /* __cplusplus */
#endif /* WIEGAND_H_ */
