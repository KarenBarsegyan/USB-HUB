#include "iarduino_I2C_DSL.h"																									//
																																//
//		Инициализация модуля:																									//	Возвращаемое значение: результат инициализации.
bool	iarduino_I2C_DSL::begin			(void){																					//	Параметр: отсутствует
		//	Инициируем работу с шиной I2C:																						//
			objI2C->begin(100);																									//	Инициируем передачу данных по шине I2C на скорости 100 кГц.
		//	Если адрес не указан, то ищим модуль на шине I2C:																	//
			if(valAddrTemp==0){																									//
				for(int i=1; i<127; i++){																						//	Проходим по всем адресам на шине I2C
					if( objI2C->checkAddress(i)											){	valAddr=i; delay(2);				//	Если на шине I2C есть устройство с адресом i, то используем этот адрес для проверки найденного модуля...
					if(_readBytes(REG_MODEL,4)											){										//	Читаем 4 байта начиная с регистра «REG_MODEL» в массив «data».
					if( data[0]     == DEF_MODEL_DSL									){										//	Если у модуля с адресом i в регистре «MODEL»   (data[0]) хранится значение DEF_MODEL_DSL, то ...
					if((data[2]>>1) == i                 || data[2] == 0xFF				){										//	Если у модуля с адресом i в регистре «ADDRESS» (data[2]) хранится значение i (адрес+младший бит) или 0xFF (адрес не задавался), то ...
					if( data[3]     == DEF_CHIP_ID_FLASH || data[3] == DEF_CHIP_ID_METRO){										//	Если у модуля с адресом i в регистре «CHIP_ID» (data[3]) хранится значение DEF_CHIP_ID_FLASH (идентификатор модулей Flash), или DEF_CHIP_ID_METRO (идентификатор модулей Metro), то ...
						valAddrTemp=i; i=128;																					//	Считаем что модуль обнаружен, сохраняем значение i как найденный адрес и выходим из цикла.
					}}}}}																										//
				}																												//
			}																													//
		//	Если модуль не найден, то возвращаем ошибку инициализации:															//
			if( valAddrTemp == 0														){	valAddr=0; return false;}			//
		//	Проверяем наличие модуля на шине I2C:																				//
			if( objI2C->checkAddress(valAddrTemp) == false								){	valAddr=0; return false;}			//	Если на шине I2C нет устройств с адресом valAddrTemp, то возвращаем ошибку инициализации
			valAddr=valAddrTemp;																								//	Сохраняем адрес модуля на шине I2C.
		//	Проверяем значения регистров модуля:																				//
			if(_readBytes(REG_MODEL,4)==false											){	valAddr=0; return false;}			//	Если не удалось прочитать 4 байта в массив «data» из модуля начиная с регистра «REG_MODEL», то возвращаем ошибку инициализации.
			if( data[0]     != DEF_MODEL_DSL											){	valAddr=0; return false;}			//	Если значение  регистра «MODEL»   (data[0]) не совпадает со значением DEF_MODEL_DSL, то возвращаем ошибку инициализации.
			if((data[2]>>1) != valAddrTemp       && data[2] !=0xFF						){	valAddr=0; return false;}			//	Если значение  регистра «ADDRESS» (data[2]) не совпадает с адресом модуля и не совпадает со значением 0xFF, то возвращаем ошибку инициализации.
			if( data[3]     != DEF_CHIP_ID_FLASH && data[3] != DEF_CHIP_ID_METRO		){	valAddr=0; return false;}			//	Если значение  регистра «CHIP_ID» (data[3]) не совпадает со значением DEF_CHIP_ID_FLASH и DEF_CHIP_ID_METRO, то возвращаем ошибку инициализации.
			valVers=data[1];																									//	Сохраняем байт регистра «VERSION» (data[1]) в переменую «valVers».
		//	Перезагружаем модуль устанавливая его регистры в значение по умолчанию:												//
			reset();																											//	Выполняем программную перезагрузку.
			delay(5);																											//
			return true;																										//	Возвращаем флаг успешной инициализаии.
}																																//
																																//
//		Перезагрузка модуля:																									//	Возвращаемое значение:	результат перезагрузки.
bool	iarduino_I2C_DSL::reset			(void){																					//	Параметр:				отсутствует.
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Устанавливаем бит перезагрузки:																					//
				if(_readBytes(REG_BITS_0,1)==false){return false;}																//	Читаем 1 байт регистра «BITS_0» в массив «data».
				data[0] |= 0b10000000;																							//	Устанавливаем бит «SET_RESET»
				if(_writeBytes(REG_BITS_0,1)==false){return false;}																//	Записываем 1 байт в регистр «BITS_0» из массива «data».
			//	Ждём установки флага завершения перезагрузки:																	//
				do{ if(_readBytes(REG_FLAGS_0,1)==false){return false;} }														//	Читаем 1 байт регистра «REG_FLAGS_0» в массив «data».
				while( (data[0]&0b10000000) == 0);																				//	Повторяем чтение пока не установится флаг «FLG_RESET».
				return true;																									//
			}else{																												//	Иначе, если датчик освещённости не инициализирован, то ...
				return false;																									//	Возвращаем ошибку
			}																													//
}																																//
																																//
//		Смена адреса модуля:																									//	Возвращаемое значение:	резульат смены адреса.
bool	iarduino_I2C_DSL::changeAddress	(uint8_t newAddr){																		//	Параметр:				newAddr - новый адрес модуля (0x07 < адрес < 0x7F).
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Проверяем новый адрес:																							//
				if(newAddr>0x7F){newAddr>>=1;}																					//	Корректируем адрес, если он указан с учётом бита RW.
				if(newAddr==0x00 || newAddr==0x7F){return false;}																//	Запрещаем устанавливать адрес 0x00 и 0x7F.
			//	Записываем новый адрес:																							//
				if(_readBytes(REG_BITS_0,1)==false){return false;}																//	Читаем 1 байт регистра «BITS_0» в массив «data».
				data[0] |= 0b00000010;																							//	Устанавливаем бит «SAVE_ADR_EN»
				if(_writeBytes(REG_BITS_0,1)==false){return false;}																//	Записываем 1 байт в регистр «BITS_0» из массива «data».
				data[0] = (newAddr<<1)|0x01;																					//	Готовим новый адрес к записи в модуль, установив бит «SAVE_FLASH».
				if(_writeBytes(REG_ADDRESS,1)==false){return false;}															//	Записываем 1 байт в регистр «ADDRESS» из массива «data».
				delay(200);																										//	Даём более чем достаточное время для применения модулем нового адреса.
			//	Проверяем наличие модуля с новым адресом на шине I2C:															//
				if(objI2C->checkAddress(newAddr)==false){return false;}															//	Если на шине I2C нет модуля с адресом newAddr, то возвращаем ошибку.
				valAddr     = newAddr;																							//	Сохраняем новый адрес как текущий.
				valAddrTemp = newAddr;																							//	Сохраняем новый адрес как указанный.
				return true;																									//	Возвращаем флаг успеха.
			}else{																												//	Иначе, если датчик освещённости не инициализирован, то ...
				return false;																									//	Возвращаем ошибку
			}																													//
}																																//
																																//
//		Чтение освещённости в люксах:																							//	Возвращаемое значение:	освещённость от 0 до 8191 лк.
uint16_t iarduino_I2C_DSL::getLux		(void){																					//	Параметр:				отсутствует.
			uint16_t	result=0;																								//	Определяем переменную для хранения результата чтения.
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Читаем значение освещённости:																					//
				_readBytes(REG_DSL_LUX_L,2);																					//	Читаем 2 байта из регистров «REG_DSL_LUX_L» и «REG_DSL_LUX_H» в массив «data».
				result = (((uint16_t)data[1])<<8) | data[0];																	//	Объеденяем два прочитанных байта в одно число.
			}																													//
			return result;																										//	Возвращаем результат чтения (0...8191).
}																																//
																																//
//		Чтение близости препятствий:																							//	Возвращаемое значение:	близость препятствий от 0 (нет  препятствий) до 1023 (датчик перекрыт).
uint16_t iarduino_I2C_DSL::getProximity	(void){																					//	Параметр:				отсутствует.
			uint16_t	result=0;																								//	Определяем переменную для хранения результата чтения.
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Читаем значение близости препятствий:																			//
				_readBytes(REG_DSL_PROXIMITY_L,2);																				//	Читаем 2 байта из регистров REG_DSL_PROXIMITY_L и «REG_DSL_PROXIMITY_H» в массив «data».
				result = (((uint16_t)data[1])<<8) | data[0];																	//	Объеденяем два прочитанных байта в одно число.
			}																													//
			return result;																										//	Возвращаем результат чтения (0...1023).
}																																//
																																//
//		Чтение коэффициента пульсаций света:																					//	Возвращаемое значение:	коэффициент пульсаций света от 0 до 100%.
uint8_t	iarduino_I2C_DSL::getPulsation	(void){																					//	Параметр:				отсутствует.
			data[0]=0;																											//	Сбрасываем значение 0 элемента массива «data».
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Читаем значение близости препятствий:																			//
				_readBytes(REG_DSL_COEFFICIENT,1);																				//	Читаем байт из регистра «REG_DSL_COEFFICIENT» в массив «data».
			}																													//
			return data[0];																										//	Возвращаем результат чтения.
}																																//
																																//
//		Получение флага изменения освещённости:																					//	Возвращаемое значение:	флаг указывающий на изменение освещённости (true/false).
bool	iarduino_I2C_DSL::getLuxChanged	(void){																					//	Параметр:				отсутствует.
			bool	 result = 0;																								//	Определяем переменную для хранения результата чтения.
			uint32_t time   = millis();																							//
			if(valAddr){																										//	Если модуль был инициализирован, то ...
				if( ( time > tmrReadFLG + tmrPeriodFLG ) || ( time < tmrReadFLG ) ){											//	Если с момента последнего опроса регистра флагов прошло более «tmrPeriodFLG» мс, или произошло переполнение «time», то ...
					tmrReadFLG = time;																							//
				//	Читаем регистр флагов:																						//
					_readBytes(REG_DSL_FLG,1);																					//	Читаем байт из регистра «REG_DSL_FLG» в массив «data».
					result = (data[0] & DSL_GET_CHANGED);																		//	Получаем состояние бита «DSL_GET_CHANGED» из прочитанного байта.
				}																												//
			}																													//
			return result;																										//	Возвращаем флаг указывающий на изменение освещённости (true/false).
}																																//
																																//
//		Установка чувствительности фиксации изменения освещённости.																//	Возвращаемое значение:	результат установки чувствиетльности (true/false).
bool	iarduino_I2C_DSL::setLuxChange	(uint8_t lux){																			//	Параметр:				освещённость в лк.
			if(lux < 1){ lux = 1; }																								//
			bool	result=0;																									//	Определяем переменную для хранения результата записи.
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Записываем новое значение чувствительности:																		//
				data[0] = lux;																									//	Готовим байт данных для пережачи.
				result = _writeBytes(REG_DSL_LUX_CHANGE,1);																		//	Записываем 1 байт в регистр «REG_DSL_LUX_CHANGE» из массива «data».
			}																													//
			return result;																										//	Возвращаем флаг указывающий на результат записи (true/false).
}																																//
																																//
//		Установка коэффициента усреднения показаний освещённости и близости.													//	Возвращаемое значение:	результат установки коэффициента (true/false).
bool	iarduino_I2C_DSL::setAveraging	(uint8_t k){																			//	Параметр:				коэффициент от 0 (без усреднений) до 255 (максимальное усреднение).
			bool	result=0;																									//	Определяем переменную для хранения результата записи.
			if(valAddr){																										//	Если модуль был инициализирован, то ...
			//	Записываем новое значение чувствительности:																		//
				data[0] = k;																									//	Готовим байт данных для пережачи.
				result = _writeBytes(REG_DSL_AVERAGING,1);																		//	Записываем 1 байт в регистр «REG_DSL_AVERAGING» из массива «data».
			}																													//
			return result;																										//	Возвращаем флаг указывающий на результат записи (true/false).
}																																//
																																//
//		Чтение данных из регистров в массив data:																				//	Возвращаемое значение:	результат чтения (true/false).
bool	iarduino_I2C_DSL::_readBytes		(uint8_t reg, uint8_t sum){															//	Параметры:				reg - номер первого регистра, sum - количество читаемых байт.
			bool	result=false;																								//	Определяем флаг       для хранения результата чтения.
			uint8_t	sumtry=10;																									//	Определяем переменную для подсчёта количества оставшихся попыток чтения.
			do{	result = objI2C->readBytes(valAddr, reg, data, sum);															//	Считываем из модуля valAddr, начиная с регистра reg, в массив data, sum байт.
				sumtry--;	if(!result){delay(1);}																				//	Уменьшаем количество попыток чтения и устанавливаем задержку при неудаче.
			}	while		(!result && sumtry>0);																				//	Повторяем чтение если оно завершилось неудачей, но не более sumtry попыток.
			delayMicroseconds(500);																								//	Между пакетами необходимо выдерживать паузу.
			return result;																										//	Возвращаем результат чтения (true/false).
}																																//
																																//
//		Запись данных в регистры из массива data:																				//	Возвращаемое значение:	результат записи (true/false).
bool	iarduino_I2C_DSL::_writeBytes	(uint8_t reg, uint8_t sum, uint8_t num){												//	Параметры:				reg - номер первого регистра, sum - количество записываемых байт, num - номер первого элемента массива data.
			bool	result=false;																								//	Определяем флаг       для хранения результата записи.
			uint8_t	sumtry=10;																									//	Определяем переменную для подсчёта количества оставшихся попыток записи.
			do{	result = objI2C->writeBytes(valAddr, reg, &data[num], sum);														//	Записываем в модуль valAddr начиная с регистра reg, sum байи из массива data начиная с элемента num.
				sumtry--;	if(!result){delay(1);}																				//	Уменьшаем количество попыток записи и устанавливаем задержку при неудаче.
			}	while		(!result && sumtry>0);																				//	Повторяем запись если она завершилась неудачей, но не более sumtry попыток.
			delay(10);																											//	Ждём применения модулем записанных данных.
			return result;																										//	Возвращаем результат записи (true/false).
}																																//
																																//