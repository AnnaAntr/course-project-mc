#include <xc.h>
#include <avr/io.h>			// описание всех регистров и портов контроллера
#include <avr/interrupt.h>	// прерывания
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 8000000UL		// частота 8 МГц

// LCD
#define RS1 PB0
#define RW1 PB1
#define E1 PB2
#define RS2 PB3
#define RW2 PB4
#define E2 PB5
#define LCD_DATA_REG DDRA
#define LCD_CONTROL_REG DDRB
#define LCD_DATA_PORT PORTA
#define LCD_CONTROL_PORT PORTB

// DHT11
#define DHT_DDR DDRF
#define DHT_PORT PORTF
#define DHT_PIN PINF
#define DHT_connect PF0

// BMP180
// 0x77 - фиксированный адрес BMP180
#define BMP_ADDRESS_READ ((0x77 << 1) | 1)
#define BMP_ADDRESS_WRITE ((0x77 << 1) | 0)
#define BMP_MEASURE_REG 0xF4		// адрес регистра измерений
#define BMP_TEMPERATURE 0x2E		// записываем в регистр, чтобы прочитать температуру
#define BMP_PRESSURE 0x34			// записываем в регистр, чтобы прочитать давление
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

// кнопки
#define BTN_hours PE4
#define BTN_minutes PE5
#define BTN_day PE6
#define BTN_enter PE7
#define BTN_TIME_PIN PINE

// глобальные переменные для времени и флаги
volatile uint8_t hours = 0, minutes = 0, seconds = 0, day = 0, millisec = 0;
volatile char enter = 0, update_measurement = 0, show_week_info = 0, clear_week_meas = 0, mode = 0, printed = 1;
// mode - режим отображения, 0 - текущие измерения, 1 - информация за неделю


// LCD ---------------------------------------------------------------------------------------------
// отправка байта
void lcd_send_byte(uint8_t byte, char mode, uint8_t lcd_num)
{
	// определение управляющих битов в зависимости от номера дисплея
	uint8_t rs = lcd_num == 1 ? RS1 : RS2;
	uint8_t rw = lcd_num == 1 ? RW1 : RW2;
	uint8_t e = lcd_num == 1 ? E1 : E2;
	
	// mode: c - команды, d - данные
	if (mode == 'c')
	{
		LCD_CONTROL_PORT &= ~(1 << rs);				// RS = 0 (режим команд)
		LCD_CONTROL_PORT &= ~(1 << rw);				// RW = 0 (запись)
	}
	else if (mode == 'd')
	{
		LCD_CONTROL_PORT |= (1 << rs);				// RS = 1 (режим данных)
		LCD_CONTROL_PORT &= ~(1 << rw);				// RW = 0 (запись)
	}
	
	// старший полубайт
	if (lcd_num == 1)
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b11110000) | (byte >> 4);				// 1-ый дисплей
	else
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | (byte & 0b11110000);		// 2-ой дисплей
	
	LCD_CONTROL_PORT |= (1 << e);		// Ex = 1
	LCD_CONTROL_PORT &= ~(1 << e);		// Ex = 0
	_delay_us(40);						// задержка
	
	// младший полубайт
	if (lcd_num == 1)
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b11110000) | (byte & 0b00001111);		// 1-ый дисплей
	else
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | (byte << 4);				// 2-ой дисплей
	
	LCD_CONTROL_PORT |= (1 << e);		// Ex = 1
	LCD_CONTROL_PORT &= ~(1 << e);		// Ex = 0
	_delay_us(40);						// задержка
}

// инициализация дисплея
void lcd_init(uint8_t lcd_num)
{
	// определение управляющих битов в зависимости от номера дисплея
	uint8_t rs = lcd_num == 1 ? RS1 : RS2;
	uint8_t rw = lcd_num == 1 ? RW1 : RW2;
	uint8_t e = lcd_num == 1 ? E1 : E2;
	
	// 4-разрядный режим
	LCD_CONTROL_PORT &= ~(1 << rs);				// RS = 0 (режим команды)
	LCD_CONTROL_PORT &= ~(1 << rw);				// RW = 0 (запись)
	
	if (lcd_num == 1)
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b11110000) | (0b00000010);	// 1-ый дисплей в 4-битный режим
	else
		LCD_DATA_PORT = (LCD_DATA_PORT & 0b00001111) | (0b00100000);	// 2-ой дисплей в 4-битный режим
	
	LCD_CONTROL_PORT |= (1 << e);					// Ex = 1
	LCD_CONTROL_PORT &= ~(1 << e);					// Ex = 0
	_delay_us(40);   								// задержка
	
	
	lcd_send_byte(0b00101000, 'c', lcd_num);		// 4-разрядная шина, 2 строки, 5x8 точек
	
	lcd_send_byte(0b00001100, 'c', lcd_num);		// вкл отображение, курсор выкл
	
	lcd_send_byte(0b00000001, 'c', lcd_num);		// очистка дисплея
	_delay_ms(2);
	
}

// установка позиции курсора
void lcd_set_cursor(uint8_t row, uint8_t col, uint8_t lcd_num)
{
	uint8_t address;
	address = row == 1 ? 0x00 : 0x40;			// определение адреса DDRAM в зависимости от номера строки
	address += col;								// добавляем смещение по столбцу
	
	lcd_send_byte(0x80 | address, 'c', lcd_num);		// установка адреса DDRAM
}

// вывод строки
void lcd_print_string(const char* str, uint8_t lcd_num)
{
	// посимвольный вывод строки
	while (*str)
	{
		lcd_send_byte(*str, 'd', lcd_num);
		str++;
	}
}

// вывод чисел различной разрядности
void lcd_print_number8(uint8_t number, uint8_t lcd_num)
{
	char buffer[3];
	sprintf(buffer, "%u", number);
	lcd_print_string(buffer, lcd_num);
}

void lcd_print_number16(uint16_t number, uint8_t lcd_num)
{
	char buffer[5];
	sprintf(buffer, "%u", number);
	lcd_print_string(buffer, lcd_num);
}

// вывод времени
void lcd_print_time()
{
	lcd_set_cursor(1, 17, 1);
	lcd_print_number8(hours, 1);
	lcd_print_string(":", 1);
	lcd_print_number8(minutes, 1);
	//lcd_print_string(":", 1);
	//lcd_print_number8(seconds, 1);
	
	lcd_set_cursor(2, 15, 1);
	switch (day)
	{
		case 1:
			lcd_print_string("Tuesday    ", 1);
			break;
		case 2:
			lcd_print_string("Wednesday  ", 1);
			break;
		case 3:
			lcd_print_string("Thursday   ", 1);
			break;
		case 4:
			lcd_print_string("Friday     ", 1);
			break;
		case 5:
			lcd_print_string("Saturday   ", 1);
			break;
		case 6:
			lcd_print_string("Sunday     ", 1);
			break;
		default:
			lcd_print_string("Monday     ", 1);
	}
}

// вывод измеренных параметров
void lcd_print_curr_meas(uint8_t t, uint8_t h, uint16_t p)
{
	lcd_send_byte(0b00000001, 'c', 2);
	_delay_ms(2);
	
	lcd_set_cursor(1, 0, 2);
	lcd_print_string("Temperature: ", 2);
	lcd_print_number8(t, 2);
	lcd_print_string("C", 2);
	
	lcd_set_cursor(1, 20, 2);
	lcd_print_string("Humidity: ", 2);
	lcd_print_number8(h, 2);
	lcd_print_string("%", 2);
	
	lcd_set_cursor(2, 0, 2);
	lcd_print_string("Pressure: ", 2);
	lcd_print_number16(p, 2);
	lcd_print_string("mm Hg", 2);
}

// вывод данных о температуре за неделю
void lcd_print_week_meas(const uint8_t* min_t, const uint8_t* max_t)
{
	// очистка дисплея
	lcd_send_byte(0b00000001, 'c', 2);
	_delay_ms(2);
	
	lcd_set_cursor(1, 0, 2);
	lcd_print_string("min  ", 2);
	
	for (uint8_t i = 0; i < 7; i++)
	{
		if (min_t[i] == 50)
			lcd_print_string("-", 2);
		else
			lcd_print_number8(min_t[i], 2);
		lcd_print_string("   ", 2);
	}
	
	lcd_set_cursor(2, 0, 2);
	lcd_print_string("max  ", 2);
	
	for (uint8_t i = 0; i < 7; i++)
	{
		if (max_t[i] == 0)
			lcd_print_string("-", 2);
		else
			lcd_print_number8(max_t[i], 2);
		lcd_print_string("   ", 2);
	}
}

// -------------------------------------------------------------------------------------------------



// DHT11 -------------------------------------------------------------------------------------------

uint8_t dht_read(uint8_t* temp, uint8_t* humid)
{
	DHT_DDR |= (1 << DHT_connect);
	DHT_PORT |= (1 << DHT_connect);
	_delay_ms(100);
	
	// низкий уровень на 18 мс
	DHT_PORT &= ~(1 << DHT_connect);
	_delay_ms(18);
	
	// высокий уровень на 40 мкс
	DHT_PORT |= (1 << DHT_connect);
	// настройка регистра на ввод
	DHT_DDR &= ~(1 << DHT_connect);
	_delay_us(40);
	
	// ответ от датчика - 80 мкс низкий уровень и 80 мкс высокий уровень
	if (DHT_PIN & (1 << DHT_connect))		// если сразу высокий уровень
		return 0;
	_delay_us(80);
	
	if (!(DHT_PIN & (1 << DHT_connect)))	// если сразу низкий уровень
		return 0;
	_delay_us(80);
	
	// от датчика приходит 5 байт данных, последний - контрольная сумма
	uint8_t data[5] = {0, 0, 0, 0, 0};
	for (uint8_t byte = 0; byte < 5; byte++)
	{
		for (uint8_t bit = 0; bit < 8; bit++)
		{
			while (!(DHT_PIN & (1 << DHT_connect)));	// перед началом бита низкий уровень
			_delay_us(30);
			
			// бит '0' - высокий уровень на 28 мкс, бит '1' - на 70 мкс
			// спустя 30 секунд проверяем, бит = 0 или бит = 1
			// устанавливаем нужный бит, если бит = 1
			if (DHT_PIN & (1 << DHT_connect))
				data[byte] |= (1 << (7 - bit));
			
			while (DHT_PIN & (1 << DHT_connect));		// ждем конца высокого уровня
		}
	}
	
	// сброс портов
	DHT_DDR |= (1 << DHT_connect);
	DHT_PORT |= (1 << DHT_connect);
	_delay_ms(100);
	
	// сумма данных не должна быть равна 0
	if (data[0] + data[1] + data[2] + data[3] == 0)
		return 0;
	
	// сумма данных должна быть равна контрольной сумме
	if (data[0] + data[1] + data[2] + data[3] != data[4])
		return 0;
	
	*temp = data[2];
	*humid = data[0];
	
	return 1;
}

// -------------------------------------------------------------------------------------------------



// I2C ---------------------------------------------------------------------------------------------

// инициализация
void i2c_init()
{
	TWBR = 32;		// 100 кГц при F_CPU = 8 МГц
	TWSR = 0;
}

// сигнал старт
void i2c_start()
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));		// ожидание завершения
}

// сигнал стоп
void i2c_stop()
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	while (TWCR & (1 << TWSTO));
}

// отправка байта
void i2c_send_byte(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));		// ожидание завершения
}

// чтение байта с битом ACK
uint8_t i2c_read_byte()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// чтение байта с битом NACK (последний)
uint8_t i2c_read_last_byte()
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// -------------------------------------------------------------------------------------------------



// BMP180 ------------------------------------------------------------------------------------------

// чтение калибровочных данных
void bmp_read_calibration()
{
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_WRITE);
	i2c_send_byte(0xAA);		// начальный адрес регистров
	i2c_stop();
	
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_READ);
	
	ac1 = (i2c_read_byte() << 8) | i2c_read_byte();
	ac2 = (i2c_read_byte() << 8) | i2c_read_byte();
	ac3 = (i2c_read_byte() << 8) | i2c_read_byte();
	ac4 = (i2c_read_byte() << 8) | i2c_read_byte();
	ac5 = (i2c_read_byte() << 8) | i2c_read_byte();
	ac6 = (i2c_read_byte() << 8) | i2c_read_byte();
	b1 = (i2c_read_byte() << 8) | i2c_read_byte();
	b2 = (i2c_read_byte() << 8) | i2c_read_byte();
	mb = (i2c_read_byte() << 8) | i2c_read_byte();
	mc = (i2c_read_byte() << 8) | i2c_read_byte();
	md = (i2c_read_byte() << 8) | i2c_read_last_byte();
	
	i2c_stop();
};

// получение некомпенсированного значения температуры
int32_t bmp_get_utemperature()
{
	// говорим, что хотим измерить температуру
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_WRITE);
	i2c_send_byte(BMP_MEASURE_REG);
	i2c_send_byte(BMP_TEMPERATURE);
	i2c_stop();
	
	_delay_ms(5);	// ждем конца измерений
	
	// теперь читаем результат
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_READ);
	i2c_send_byte(0xF6);		// начинаем читать измерения с этого адреса
	i2c_stop();
	
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_READ);
	
	
	uint32_t b1 = i2c_read_byte();
	uint32_t b2 = i2c_read_last_byte();
	int32_t ut = (b1 << 8) | b2;
	
	i2c_stop();
	
	return ut;
}

// получение некомпенсированного значения давления
int32_t bmp_get_upressure()
{
	// говорим, что хотим измерить давление
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_WRITE);
	i2c_send_byte(BMP_MEASURE_REG);
	i2c_send_byte(BMP_PRESSURE);
	i2c_stop();
	
	_delay_ms(5);	// ждем конца измерений
	
	// теперь читаем результат
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_READ);
	i2c_send_byte(0xF6);		// начинаем читать измерения с этого адреса
	i2c_stop();
	
	i2c_start();
	i2c_send_byte(BMP_ADDRESS_READ);
	
	uint32_t b1 = i2c_read_byte();
	uint32_t b2 = i2c_read_byte();
	uint32_t b3 = i2c_read_last_byte();
	int32_t up = ((b1 << 16) | (b2 << 8) | b3) >> 8;

	i2c_stop();
	
	return up;
}

// вычисление реального значения давления
int32_t bmp_calculate_pressure()
{
	int32_t p, t;
	int32_t x1, x2, b5, b6, x3, b3;
	uint32_t b4, b7;
	
	int32_t ut = bmp_get_utemperature();
	int32_t up = bmp_get_upressure();
	
	// вычисление температуры
	x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
	x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
	b5 = x1 + x2;
	t = (b5 + 8) >> 4;		// t в 0.1 C
	t /= 10;
	
	// вычисление давление (в Паскалях)
	b6 = b5 - 4000;
	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = ((int32_t)ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((int32_t)ac1 * 4 + x3) + 2) >> 2;
	
	x1 = ((int32_t)ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	b4 = ((int32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = (up - b3) * 50000;
	
	if (b7 < 0x80000000)
		p = b7 * 2 / b4;
	else
		p = b7 / b4 * 2;
	
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p = p + ((x1 + x2 + 3791) >> 4);
	
	return p;
}

// -------------------------------------------------------------------------------------------------



// INTERRUPTS --------------------------------------------------------------------------------------
// INT2 week (отображение информации за неделю)
// INT4 hours
// INT5 minutes
// INT6 week day
// INT7 enter (ввод времени закончен)

ISR(INT2_vect)
{
	_delay_ms(20);
	if (PIND & (1 << PD2))
	{
		if (mode == 0)
		{
			show_week_info = 1;
			mode = 1;
		}
		
		else if (mode == 1)
		{
			mode = 0;
			printed = 0;
		}
	}
}

ISR(INT4_vect)
{
	_delay_ms(20);
	if (BTN_TIME_PIN & (1 << BTN_hours))
		hours++;
	
	if (hours == 24)
		hours = 0;
}

ISR(INT5_vect)
{
	_delay_ms(20);
	if (BTN_TIME_PIN & (1 << BTN_minutes))
		minutes++;
	
	if (minutes == 60)
		minutes = 0;
}

ISR(INT6_vect)
{
	_delay_ms(20);
	if (BTN_TIME_PIN & (1 << BTN_day))
		day++;
	
	if (day == 7)
		day = 0;
}

ISR(INT7_vect)
{
	_delay_ms(20);
	if (BTN_TIME_PIN & (1 << BTN_enter))
	{
		// enter = 1;
		// теперь разрешено только прерывание INT2
		// EIMSK = 0x04;
		
		
		// изначально enter = 0
		// нажата кнопка = ввод времени окончен
		if (enter == 0)
		{
			enter = 1;
			// теперь разрешены только прерывания INT2 и INT 7
			EIMSK = 0x84;
			
			// запуск таймера
			// режим СТС (при совпадении с OCR таймер сбрасывается в 0)
			TCCR1A = 0x00;
			TCCR1B |= (1 << WGM12);
			OCR1A = 3905;
			// разрешаем прерывание по совпадению
			TIMSK |= (1 << OCIE1A);
			// предделитель 256
			TCCR1B |= (1 << CS12);
		}
		
		// нажатие кнопки для изменения времени
		else
		{
			enter = 0;
			
			// разрешение прерываний кнопок
			EIMSK = 0xF4;
			
			// остановка таймера
			TCCR1B &= ~(1 << WGM12);
			TCCR1B &= ~(1 << CS12);
			TIMSK &= ~(1 << OCIE1A);
			
			seconds = 0;
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	seconds++;
	
	// обновление показаний каждые 5 секунд
	if (seconds % 5 == 0)
		update_measurement = 1;
	
	if (seconds == 60)
	{
		seconds = 0;
		minutes += 1;
		
		// каждые 10 минут обновляем показания
		//if (minutes % 10 == 0)
			//update_measurement = 1;
		
		if (minutes == 60)
		{
			minutes = 0;
			hours++;
			
			if (hours == 24)
			{
				hours = 0;
				day++;
				
				if (day == 7)
				{
					day = 0;
					clear_week_meas = 1;		// устанавливаем флаг для очистки массивов с информацией по неделе
				}
			}
		}
	}
}

// -------------------------------------------------------------------------------------------------



// MAIN --------------------------------------------------------------------------------------------
int main()
{
	// настройка регистров для ЖКИ
	LCD_DATA_REG = 0xFF;		// порт A как выход (данные)
	LCD_CONTROL_REG = 0x3F;		// PB0-PB5 как выходы (управление)
	lcd_init(1);				// инициализация дисплеев
	lcd_init(2);
	
	EICRA |= (1 << ISC21) | (1 << ISC20);	// INT2 по фронту
	EICRB = 0x55;							// INT4-7 по изменению уровня
	EIMSK = 0xF4;							// разрешение INT2, INT4-7
	sei();									// глобальное разрешение прерываний
	
	i2c_init();					// инициализация I2C
	bmp_read_calibration();		// чтение калибровочных данных BMP180
	
	
	// переменные для измерений
	uint8_t temperature = 0, humidity = 0;
	uint16_t pressure = 0;
	
	// массивы для мин и макс значений
	// большие значения для минимумов, чтобы они точно обновились
	uint8_t min_t[7] = {50, 50, 50, 50, 50, 50, 50};
	// маленькие значения (0) для максимумов, чтобы они точно обновились
	uint8_t max_t[7];
	
	// первичное измерение
	dht_read(&temperature, &humidity);
	pressure = (int16_t)(bmp_calculate_pressure() / 133.3);
	lcd_print_curr_meas(temperature, humidity, pressure);

	
	while (1)
	{
		// очистка массивов
		if (clear_week_meas)
		{
			for (uint8_t i = 0; i < 7; i++)
			{
				min_t[i] = 50;
				max_t[i] = 0;
			}
			clear_week_meas = 0;
		}
		
		// обновление показаний
		if (update_measurement & enter)
		{
			dht_read(&temperature, &humidity);
			pressure = (int16_t)(bmp_calculate_pressure() / 133.3);
			
			if (mode == 0)
				lcd_print_curr_meas(temperature, humidity, pressure);
			
			update_measurement = 0;
			
			// обновление мин/макс значения температуры за день
			if (temperature > max_t[day])
				max_t[day] = temperature;
			if (temperature < min_t[day])
				min_t[day] = temperature;
		}
		
		// если в режиме отображения текущих измерений и они не выведены
		// например, если переключились из режима 1 в режим 0, то нужно вывести измерения
		if ((mode == 0) & (printed == 0))
		{
			lcd_print_curr_meas(temperature, humidity, pressure);
			printed = 1;
		}
		
		if (show_week_info)
		{
			lcd_print_week_meas(min_t, max_t);
			show_week_info = 0;
		}
		
		lcd_print_time();
	}
	
	return 0;
}