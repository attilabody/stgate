/*
 * mainloop.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: compi
 */
#include <ctype.h>
#include <rfdecoder.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "thindb.h"

#include <sg/ItLock.h>


#include "config.h"

#include "mainloop.h"

//#define TESTLOOP
//#define TEST_SDCARD
#define TEST_EEPROM
#define TEST_RTC

#if defined(TEST_SDCARD)
#include "SdFile.h"
#endif	//	TEST_SDCARD


extern "C" void _MainLoop();
extern "C" void HAL_SYSTICK_Callback(void);
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);


bool g_mainLoppReady = false;

char g_stateSigns[States::NUMSTATES] = { ' ', 'W', 'C', 'A', 'W', 'D', 'U', 'H', 'P' };

////////////////////////////////////////////////////////////////////
void _MainLoop()
{
	Config::Instance().Load();

	MainLoop::Instance().Loop();
}

////////////////////////////////////////////////////////////////////
void HAL_SYSTICK_Callback(void)
{
	uint32_t	now = HAL_GetTick();
	if(g_mainLoppReady)
		MainLoop::Instance().Tick(now);
}

////////////////////////////////////////////////////////////////////
MainLoop::MainLoop()
: m_lights(256, 128)
, m_gate(OPEN_GPIO_Port, OPEN_Pin, false)
, m_com(&huart1, sg::UsartCallbackDispatcher::Instance(), m_serialOutRingBuffer, sizeof(m_serialOutRingBuffer), true)
, m_wifi(&huart3, sg::UsartCallbackDispatcher::Instance(), m_wifiOutRingBuffer, sizeof(m_wifiOutRingBuffer), true)
, m_i2c(I2c1::Instance())
, m_db(MainI2cEeprom::Instance(), (sizeof(ConfigData)/I2CEEPROMPAGELENGTH+1)*I2CEEPROMPAGELENGTH)
, m_rtc(m_i2c)
, m_lcd(m_i2c, Config::Instance().lcdI2cAddress)
, m_decoder(RFDecoder::Instance())
, m_loop(LOOP_I_GPIO_Port, LOOP_I_Pin, LOOP_O_GPIO_Port, LOOP_O_Pin, true)
, m_log("LOG.TXT")
, m_proc(*this)
{
	m_decoder.Init(*this);
	Wiegand::Instance().Init(
		&htim4,
		OUT_D1_GPIO_Port, OUT_D1_Pin,
		OUT_D0_GPIO_Port, OUT_D0_Pin,
		IN_D1_GPIO_Port, IN_D1_Pin,
		IN_D0_GPIO_Port, IN_D0_Pin,
		250
	);
}

////////////////////////////////////////////////////////////////////
#define TICK(x) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); m_com << x << "\r\n"
#define MSG(x)	m_com << x << "\r\n";

////////////////////////////////////////////////////////////////////
void MainLoop::Fail(const char * file, int line)
{
	m_com << file << ": " << (uint32_t)line << "\r\n";
	m_wifi << file << ": " << (uint32_t)line << "\r\n";
	while(1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
	}
}

#define FAIL() Fail(__FILE__, __LINE__)

//////////////////////////////////////////////////////////////////////
//void MainLoop::DumpBufferLine(uint8_t *buffer, uint16_t base, uint16_t offset, uint8_t count)
//{
//	m_com << sg::Usart::hex << sg::Usart::pad << (uint16_t)(base + offset);
//	while(count--) {
//		m_com << ' ' << buffer[offset++];
//	}
//	m_com << sg::Usart::endl;
//
//}

////////////////////////////////////////////////////////////////////
void MainLoop::LineReceived(char *buffer, uint16_t count)
{
	HAL_StatusTypeDef ret;
	(void)ret;

	if(buffer != m_serialBuffer && buffer != m_wifiBuffer)
		return;
	if(buffer == m_serialBuffer && m_serialLineReceived) {
		ret = m_com.Receive(m_serialBuffer, sizeof(m_serialBuffer), *this);
		return;
	} else if( buffer == m_wifiBuffer && m_wifiLineReceived) {
		ret = m_wifi.Receive(m_wifiBuffer, sizeof(m_wifiBuffer), *this);
		return;
	}

	char *bufPtr = buffer + count -1;
	while(bufPtr >= buffer && (*bufPtr == '\r' || *bufPtr == '\n' || !*bufPtr))
		--bufPtr;
	if(bufPtr < buffer + LINEBUFFER_SIZE - 1)
		*(bufPtr + 1) = 0;
	else
		buffer[ LINEBUFFER_SIZE - 1] = 0;

	if(buffer == m_serialBuffer)
		m_serialLineReceived = true;
	else
		m_wifiLineReceived = true;
}


////////////////////////////////////////////////////////////////////
// Called from ISR
////////////////////////////////////////////////////////////////////
void MainLoop::CodeReceived(uint16_t code)
{
	if(!m_codeReceived) {
		m_code = code;
		m_codeReceived = true;
	}
	uint32_t now = HAL_GetTick();
	if(m_codeLogQueueIndex < CODE_LOG_QUEUE_SIZE && (m_lastCodeReceived != code || now - m_lastCodeReceivedTick > 1000))
		m_codeLogQueue[m_codeLogQueueIndex++] = code;

	m_lastCodeReceived = code;
	m_lastCodeReceivedTick = now;
}

////////////////////////////////////////////////////////////////////
void MainLoop::Tick(uint32_t now)
{
	m_lights.Tick(now);
	m_loop.Tick(now);
	m_gate.Tick(now);
}

////////////////////////////////////////////////////////////////////
#ifdef TESTLOOP
void MainLoop::Loop()
{
	HAL_StatusTypeDef	ret = HAL_OK;
	bool				firstCode = true;
	uint8_t				eepromBuffer[256];

	for(uint8_t cnt = 0; cnt < 16; ++cnt) {
	  uint32_t start = SysTick->VAL;
	  while(SysTick->VAL - start < 1000 );
	  TICK('.');
	}
	m_com << "\r\n";

#ifdef TEST_SDCARD
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "This is STM32 working with FatFs\r\n"; /* File write buffer */
	uint8_t rtext[100];                                   /* File read buffer */

	uint32_t fpos = 0;
	SdFile sdFileW;

	m_lcd.Print("SD card");

	TICK("openw1");
	if(sdFileW.Open("STM32.TXT", static_cast<SdFile::OpenMode>(SdFile::CREATE_ALWAYS | SdFile::WRITE))) {
		SdFile sdFileR;
		TICK("openr1");
		if(sdFileR.Open("FILE.TXT", static_cast<SdFile::OpenMode>(SdFile::OPEN_EXISTING | SdFile::READ))) {
			TICK("read1");
			if((bytesread = sdFileR.Read(rtext, sizeof(rtext))) != 0) {
				m_com << sg::Usart::Buffer(rtext, bytesread) << "\r\n";
				fpos = sdFileR.Ftell();
				TICK("write1.1");
				byteswritten = sdFileW.Write(wtext, sizeof(wtext)-1);
				TICK("closer1");
				sdFileR.Close();
				if(byteswritten) {
					TICK("closew1");
					sdFileW.Close();
					if(sdFileW.Open("STM32.TXT", static_cast<SdFile::OpenMode>(SdFile::OPEN_EXISTING | SdFile::WRITE))) {
						sdFileW.Seek(sdFileW.Size());
						TICK("write1.2");
						byteswritten = sdFileW.Write(rtext, bytesread);
						TICK("closew1");
						sdFileW.Close();
					}
					TICK("openr2");
					if(sdFileR.Open("STM32.TXT", static_cast<SdFile::OpenMode>(SdFile::OPEN_EXISTING | SdFile::READ))) {
						do {
							TICK("read2");
							if((bytesread = sdFileR.Read(rtext, sizeof(rtext))))
								m_com.Send(rtext, bytesread);
						} while(bytesread != 0);
						TICK("closer2");
						sdFileR.Close();
					} else FAIL();
				} else FAIL();
			} else FAIL();
		} else FAIL();
	} else FAIL();
	(void)fpos;
#endif	//	TEST_SDCARD

	m_lcd.Clear();
	m_lcd.Print("EEPROM");

#ifdef TEST_EEPROM
	for(uint16_t i=0; i < sizeof(eepromBuffer); ++i)
		eepromBuffer[i] = 0xff-i;

	ret = m_i2cEeprom.Write(eepromBuffer, sizeof(eepromBuffer), sizeof(eepromBuffer));
	memset(eepromBuffer, 0xaa, sizeof(eepromBuffer));
	ret = m_i2cEeprom.Read(eepromBuffer, 0, sizeof(eepromBuffer));
	if(ret == HAL_OK)
	{
		m_i2cEeprom.Sync();
		for(uint16_t offset = 0; offset < sizeof(eepromBuffer); offset += 32)
			DumpBufferLine(eepromBuffer, 0, offset, 32);
		m_com << sg::Usart::endl;

		ret = m_i2cEeprom.Read(eepromBuffer, sizeof(eepromBuffer), sizeof(eepromBuffer));
		if(ret == HAL_OK) {
			m_i2cEeprom.Sync();
			for(uint16_t offset = 0; offset < sizeof(eepromBuffer); offset += 32)
				DumpBufferLine(eepromBuffer, sizeof(eepromBuffer), offset, 32);
			m_com << sg::Usart::endl;
		}
	}

	database::dbrecord	rec;
	m_db.getParams(0, rec);

	uint16_t	base = 0;
#endif	//	TEST_EEPROM

#ifdef TEST_RTC
	using Ts = sg::DS3231::Ts;
	ret = m_rtc.Init();
	Ts		ts;
	bool	desync = false;
#endif	//	TEST_RTC
	ret = m_com.Receive(m_serialOutRingBuffer, sizeof(m_serialOutRingBuffer), *this);

	uint32_t	counter = 0;
	while(true)
	{
		uint32_t offset = 0;
		for(uint8_t u = 0; u < m_lights.COUNT; ++u) {
			uint32_t curval = (counter+offset) & 0x1fff;
			m_lights.Set(u, curval > 4096 ? 8192 - curval : curval);
			offset += 4096/5;
		}
		counter += 256;

		auto firstTick = HAL_GetTick();
		do {
			if(m_serialLineReceived) {
				m_com << m_serialOutRingBuffer << "\r\n";
				m_serialLineReceived = false;
				ret = m_com.Receive(m_serialOutRingBuffer, sizeof(m_serialOutRingBuffer), *this);
			}
		} while(HAL_GetTick() - firstTick < 20 );

		if(counter >= 8192)
		{
			counter -= 8192;
#ifdef TEST_RTC
			ret = m_rtc.Get(ts, desync);
			m_com << sg::Usart::dec << (ts.hour < 10 ? "0" : "") << ts.hour << (ts.min < 10 ? ":0" : ":") << ts.min << (ts.sec < 10 ? ":0" : ":") << ts.sec << "  ";
#else
			TICK('#');
#endif
			if(m_codeReceived) {
				auto code = m_code;
				m_codeReceived = false;
				m_com << code;
				if(firstCode) {
					m_lcd.Clear();
					firstCode = false;
				} else
					m_lcd.SetCursor(0, 0);
				m_lcd.Print(code, false, 4);
			}
			m_com << sg::Usart::endl;

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
#if defined(TEST_EEPROM)
			m_com << sg::Usart::endl;
			ret = m_i2cEeprom.Read(eepromBuffer, base, sizeof(eepromBuffer));
			m_i2cEeprom.Sync();
			for(uint16_t offset = 0; offset < sizeof(eepromBuffer); offset += 32)
				DumpBufferLine(eepromBuffer, base, offset, 32);
			m_com << sg::Usart::endl << sg::Usart::endl;
			base += 256;
#endif
		}
	}
	(void)ret;
}
#else	//	TESTLOOP

////////////////////////////////////////////////////////////////////
void MainLoop::Loop()
{
	HAL_StatusTypeDef		ret = HAL_OK;
	uint32_t				oldTick, ellapsed;
	uint32_t				now, lastHeartbeat;
	sg::DS3231::Ts			ts;

	InductiveLoop::STATUS	ilStatus = InductiveLoop::NONE;
	bool					ilConflict = false;
	bool					inner = false;
	bool					ilChanged = false;
	
	{
		SdFile	f;
		if( f.Open("IMP", static_cast<SdFile::OpenMode>(SdFile::OPEN_EXISTING | SdFile::READ)) == FR_OK) {
			f.Close();
			SdVolume v;
			v.Unlink("IMP");
			uint16_t changed;
			Import(0,MAX_CODE,changed);
			m_lcd.Update(0,0,"Imported:");
			m_lcd.UpdateLastReceivedId(changed);
			HAL_Delay(5000);
			m_lcd.Clear();
		}
	}

	g_mainLoppReady = true;

	m_lcd.UpdateLoopStatus(false, true, false);
	m_lights.SetMode(States::NUMSTATES, false);
	HAL_Delay(1000);
	m_lcd.UpdateLoopStatus(true, false, false);
	m_lights.SetMode(States::NUMSTATES, true);
	HAL_Delay(1000);
	m_lcd.UpdateLoopStatus(false, false, false);
	m_lights.SetMode(States::OFF, false);

	{
		m_rtc.Get(m_rtcDateTime, m_rtcDesync);
		do {
			m_rtc.Get(ts, m_rtcDesync);
		} while(ts.sec == m_rtcDateTime.sec);
		m_rtcTick = HAL_GetTick();
		UpdateDow(ts);
		m_rtcDateTime = ts;
	}
	oldTick = m_rtcTick;
	lastHeartbeat = m_rtcTick;

	m_log.log(m_log.INFO, m_rtcDateTime, "RESET", (int)m_rtcDesync);

	if((ret = m_com.Receive(m_serialBuffer, sizeof(m_serialBuffer), *this)) != HAL_OK)
		m_log.log(logwriter::ERROR, m_rtcDateTime, "m_com.Receive", ret);
	if((ret = m_wifi.Receive(m_wifiBuffer, sizeof(m_wifiBuffer), *this)) != HAL_OK)
		m_log.log(logwriter::ERROR, m_rtcDateTime, "m_wifi.Receive", ret);

	m_com << "\r\n" CMNTS  "READY\r\n";
	m_wifi << "\r\n" CMNTS "READY\r\n";

	while(true)
	{
		now = HAL_GetTick();

		if(m_com.GetAndClearError()) {
			if((ret = m_com.Receive(m_serialBuffer, sizeof(m_serialBuffer), *this)) != HAL_OK)
				m_log.log(logwriter::ERROR, m_rtcDateTime, "m_com.Receive", ret);
			m_serialLineReceived = false;
		}
		if(m_serialLineReceived) {
			if(m_serialBuffer[0])
				m_proc.Process(m_com, m_serialBuffer);
			m_serialLineReceived = false;
			ret = m_com.Receive(m_serialBuffer, sizeof(m_serialBuffer), *this);
		}
		if(m_wifi.GetAndClearError()) {
			if((ret = m_wifi.Receive(m_wifiBuffer, sizeof(m_wifiBuffer), *this)) != HAL_OK)
				m_log.log(logwriter::ERROR, m_rtcDateTime, "m_wifi.Receive", ret);
			m_wifiLineReceived = false;
		}
		if(m_wifiLineReceived) {
			if(m_wifiBuffer[0])
				m_proc.Process(m_wifi, m_wifiBuffer);
			m_wifiLineReceived = false;
			ret = m_wifi.Receive(m_wifiBuffer, sizeof(m_wifiBuffer), *this);
		}

		if(m_codeLogQueueIndex) {
			uint16_t code;
			{
				sg::ItLock	lock;
				code = m_codeLogQueue[--m_codeLogQueueIndex];
			}
			m_log.log(m_log.INFO, m_rtcDateTime, "RCV", code & 0x3ff, code >> 10, -1, -1, -1);

			if(code != m_lcd.GetLastReceivedId())
				m_lcd.UpdateLastReceivedId(code);
		}

		if(now != oldTick)
		{
			{
				sg::ItLock	lock;
				ilStatus = m_loop.GetStatus();
				ilConflict = m_loop.GetConflict();
			}
			ilChanged = ilStatus != m_ilStatus || ilConflict != m_ilConflict;

			if(ilChanged) {
				m_ilStatus = ilStatus;
				m_ilConflict = ilConflict;
				if (ilStatus != InductiveLoop::NONE) {
					m_lcd.UpdateLoopStatus(ilStatus == InductiveLoop::INNER, ilStatus == InductiveLoop::OUTER, ilConflict);
				} else {
					m_lcd.UpdateDow(m_rtcDateTime.wday);
				}
				inner = ilStatus == InductiveLoop::INNER;
			}

			if(CheckDateTime(now))
				m_lcd.UpdateDt(m_rtcDateTime, m_rtcDesync);

			if(now - lastHeartbeat >= 500) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				lastHeartbeat = now;
			}

			bool sw=HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin);

			if (m_switchOld && !sw) {
				m_log.log(m_log.INFO, m_rtcDateTime, "status del");
				m_lcd.ClrId();
				ClrAllStatus();
			}
			m_switchOld = sw;

			oldTick = now;
		}

		switch(m_state)
		{
		case States::OFF:
		case States::CONFLICT:
		case States::CODEWAIT:
			if(ilChanged)
			{
				ChangeState(ilStatus == InductiveLoop::NONE ? States::OFF : (ilConflict ? States::CONFLICT : States::CODEWAIT), inner, now);
			}
			else if(m_state == States::CODEWAIT && m_codeReceived)
			{
				if(m_code == m_countedCode) {	// received second time
					ChangeState(Authorize(m_code, inner), inner, now);
				} else {
					m_countedCode = m_code;
					m_codeReceived = false;
				}
			}

			break;

		case States::ACCEPT:
		case States::WARN:
		case States::HURRY:
		case States::PASSING:
			if(ilChanged) {
				if(ilStatus == InductiveLoop::NONE) {
					SetStatus(m_countedCode, m_cycleInner ? database::dbrecord::OUTSIDE : database::dbrecord::INSIDE);
					ChangeState(States::OFF, inner, now);
				} else if(m_state != States::PASSING) {
					ChangeState(States::PASSING, inner, m_stateStartedTick);
				}
			} else if(m_state == States::ACCEPT || m_state == States::WARN) {
				ellapsed = now - m_stateStartedTick;
				if(ellapsed > Config::Instance().passTimeout * 1000)
					ChangeState(States::HURRY, inner, now);
			} else {
				ellapsed = now - m_stateStartedTick;
				if(ellapsed > Config::Instance().hurryTimeout * 1000)
					ChangeState(States::OFF, inner, now);
			}
			break;

		case States::DENY:
		case States::UNREGISTERED:
			if(ilChanged && ilStatus != (m_cycleInner ? InductiveLoop::INNER : InductiveLoop::OUTER)) {
				ChangeState(ilStatus == InductiveLoop::NONE ? States::OFF : (ilConflict ? States::CONFLICT : States::CODEWAIT), inner, now);
			}
			break;

		case States::NUMSTATES:	// should not happen
			break;
		}	//	switch(m_state)

		m_log.LogFromQueue(1);

	}	//	while(true)

	(void)ret;
}

////////////////////////////////////////////////////////////////////
void MainLoop::ChangeState(States newStatus, bool inner, uint32_t now)
{
	bool open = false;
	if(newStatus == States::CODEWAIT) {
		m_countedCode = -1;
		m_codeReceived = false;
		m_cycleInner = inner;
	} else if(newStatus == States::ACCEPT || newStatus == States::WARN) {
		m_gate.Set(500, 1500, now);
		open = true;
	} else if(newStatus == States::OFF)
		m_gate.Reset();

	m_stateStartedTick = now;
	m_lights.SetMode(newStatus, inner);
	if (open) {
		Wiegand::Instance().SetCode(inner, m_code&0x3ff);
		if (m_lastAuthMaster && m_rtcDesync)
			m_lights.BlinkPrimaryYellow(inner);
	}
	m_state = newStatus;
}

////////////////////////////////////////////////////////////////////
// may return ACCEPT WARN DENY UNREGISTERED
States MainLoop::Authorize( uint16_t id, bool inner )
{
	database::dbrecord	rec;
	States				ret(States::ACCEPT);
	char				reason = ' ';

	uint16_t	mod( m_rtcDateTime.min + m_rtcDateTime.hour * 60 );
	uint8_t		dow( 1<<(m_rtcDateTime.wday - 1));

	if( !m_db.getParams(id & 0x3ff, rec) )
		return ret;

	rec.position = GetStatus(id);
	m_lastAuthMaster = rec.days & 0x80;

	if(!rec.in_start && !rec.in_end)
		ret = States::UNREGISTERED;
	else if( rec.position == ( inner ? database::dbrecord::OUTSIDE : database::dbrecord::INSIDE ) ) {
		ret = Config::Instance().relaxedPos ? States::WARN : States::DENY;
		reason = 'P';
	} else if(!m_rtcDesync)
	{
		States	timeFail = Config::Instance().relaxedDateTime ? States::WARN : States::DENY;
		if( !( rec.days & dow )) {
			ret = timeFail;
			reason = 'D';
		} else {
			uint16_t	start( inner ? rec.out_start : rec.in_start );
			uint16_t	end( inner ? rec.out_end : rec.in_end );

			if(mod < start || mod > end) {
				ret = timeFail;
				reason = 'T';
			}
		}
	}
	m_log.log(logwriter::INFO, m_rtcDateTime, "Auth", id&0x3ff, id >> 10, rec.position, inner, ret, reason );
	if(m_lastAuthMaster && ret == States::DENY)
		ret = States::WARN;

	m_lcd.UpdateLastDecision(ret, id, reason);
	return ret;
}

////////////////////////////////////////////////////////////////////
bool MainLoop::CheckDateTime(uint32_t now)
{
	sg::DS3231::Ts	ts;
	bool desync;

	if(now - m_rtcTick > 980) {
		m_rtc.Get(ts, desync);
		if(m_rtcDateTime.sec != ts.sec) {
			if(m_rtcDateTime.mday != ts.mday)
				UpdateDow(ts);
			m_rtcTick = now;
			m_rtcDateTime = ts;
			m_rtcDesync = desync;
			return true;
		}
	}
	return false;
}

////////////////////////////////////////////////////////////////////
void MainLoop::UpdateDow(sg::DS3231::Ts &ts)
{
	SdFile		f;
	char		buffer[17];
	const char	*bufPtr;
	uint8_t		month, day, dow;

	uint8_t	used = 0;
	FRESULT	fr;
	unsigned int	read;

	if(f.Open("SHUFFLE.TXT", static_cast<SdFile::OpenMode>(SdFile::OPEN_EXISTING | SdFile::READ )) == FR_OK)
	{
		do {
			if((fr = f.Read(buffer + used, sizeof(buffer) - 1 - used, &read)) == FR_OK) {
				used += read;
				buffer[used] = 0;
				bufPtr = buffer;
				if( (month = sg::GetIntParam(bufPtr)) != 0xff &&
					(day = sg::GetIntParam(bufPtr)) != 0xff &&
					(dow = sg::GetIntParam(bufPtr) != 0xff) &&
					month == ts.mon && day == ts.mday)
				{
					ts.wday = dow;
					break;
				}
				memmove(buffer, bufPtr, used - (bufPtr - buffer));
				used -= bufPtr - buffer;
			}
		} while(fr == FR_OK && (used || read));
		f.Close();
	}
}

////////////////////////////////////////////////////////////////////
void MainLoop::SetStatus( int code, database::dbrecord::POSITION pos ) {
	if (code<1024) {
		uint8_t shift = (code & 3) << 1;
		uint8_t index = code >> 2;
		m_status[index] = (m_status[index] & (0xFF ^ (3<<shift))) | ((uint8_t)pos << shift);
	}
}

////////////////////////////////////////////////////////////////////
database::dbrecord::POSITION MainLoop::GetStatus( int code) {
	database::dbrecord::POSITION res = database::dbrecord::UNKNOWN;
	if (code<1024) {
		uint8_t shift = (code & 3) << 1;
		uint8_t index = code >> 2;
		res = static_cast<database::dbrecord::POSITION>((m_status[index] >> shift) & 3);
	}
	return res;
}

////////////////////////////////////////////////////////////////////
void MainLoop::ClrAllStatus() {
	memset(&m_status,0,sizeof(m_status));
}

////////////////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim==&htim1) {
		RFDecoder::Instance().PeriodEllapsed(htim);
	} else {
		Wiegand::Instance().TimerUpdateIT(htim);
	}
}

////////////////////////////////////////////////////////////////////
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	Wiegand::Instance().TimerPWMIT(htim);
}

//////////////////////////////////////////////////////////////////////////////
bool MainLoop::Import(uint16_t from, uint16_t to, uint16_t &changed) {
	changed=0;
	if(to > MAX_CODE) to = MAX_CODE;
	if(from == 0xffff) from = 0;
	else if(from > to) from = to;
	thindb tdb;
	database::dbrecord rec, old;
	if(tdb.init("DB.TXT", true)) {
		uint16_t id;
		for(id = from; id <= to; ++id) {
			if(!tdb.getParams(id, rec) || !m_db.getParams(id, old))
				break;
			if(!rec.infoequal(old))
			{
				if(!m_db.setParams(id, rec))
					break;
				else
					changed++;
			}
		}
		if(id != to+1)
			id = 0;
		tdb.close();
		return true;
	}
	return false;
}

#endif	//	TESTLOOP
