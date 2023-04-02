// Pawel Kuc
// 2019-10-27
//
// Wyswietlanie na LCD1602-I2C temperatury
// oraz ID wykrytych termometrow 1-Wire.
// Maksymalna ilosc wyswietlanych - 9.
//
// Adres EXPANDERA zmien wg konfiguracji sprzetowej w lcd44780.h
// Pin i port podlaczenia magistrali 1-wire zmień w onewire.h
// Pin i port podlaczenia triggera (u-switch lub TTP226 lub inne) zmien w konfiguracji ponizej
// Wyzwalanie triggera stanem wysokim
//
// Program nie używa timerów sprzetowych aby latwo przeniesc go na dowolny mikrokontroler.

//		chip: ATMega16
//		freq: 2.000 MHz int
//			4 182 bytes FLASH
//			  393 bytes RAM
//

// Podlaczenie portow
// PortA.0 - klawisz
// PortC.0 - SCL - LCD
// PortC.1 - SDA - LCD
// PortC.2 - 1-WIRE - DS18x20
// PortD.7 - LED

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "LCD/lcd44780.h"
#include "1Wire/ds18x20.h"

#define LEDTEST_PIN (1<<7)		// pin podlaczenia diody
#define LEDTEST_PORT PORTD		// port podlaczenia diody
#define LEDTEST_ON  LEDTEST_PORT |=  LEDTEST_PIN
#define LEDTEST_OFF LEDTEST_PORT &= ~LEDTEST_PIN

#define KEY_PIN (1<<0)			// pin podlaczenia triggera
#define KEY_PORT PORTA			// port podlaczenia triggera

//	------------------ definicja zmiennych i wartosci ------------------------------
uint8_t subzero, cel, cel_fract_bits; // zmienne do temperatury: minus, stopnie, dziecietne-stopni
uint8_t czujniki_cnt;				// wykryta ilosc czujnikow na magistrali

uint8_t temp_meas[MAXSENSORS][3];	// tablica z temperaturami odczytanymi
char temp_id[MAXSENSORS][8];		// tablica ze znalezionymi identyfikatorami sensorow

uint8_t bounce = 50;				// czas oczekiwania w ms na wygaszenie drgan klawisza, zalecane 50 dla mikroswitch, dla TTP223 może być 1 (0 nie jest obsługiwane przez _delay_ms)

uint8_t tryb = 0;					// zmienna sterująca aktualnym trybem
	// 0 - po uruchomieniu, reset czujnik_cur do 0
	// 1 - uruchomienie testera, wyszukiwanie czujnikow, pomiar i zapis tablicy, informacja o ilosci wykrytych
	// 2 - wyswietlanie, zmiana czujnik_cur na wieksza od 1
	// 3 - zakonczenie lub informacja o zbyt duzej ilosci czujnikow, po wyjsciu zmiana na tryb = 0
uint8_t czujnik_cur = 0;					// zmienna przechowujaca numer aktualnie prezentowanego czujnika

//	------------------ definicja portow i kierunkow --------------------------------
void port_init()
	{
	DDRB  = 0b00001000;		// ustawienie portu B - 3 jako wyjscia
	PORTB = 0b00001000;		// ustawienie portu B -
	DDRC  = 0b00011111;		// ustawienie portu C - 0-5 jako wyjscia
	PORTC = 0b00000000;		// ustawienie portu C -
	DDRD  = 0b10000000;		// ustawienie portu D - 3 jako wyjscia
	PORTD = 0b0000000;		// ustawienie portu D -
	
	}

void LCD_START()
{
	lcd_cls();

	lcd_locate(0,0);
	lcd_str(" Tester DS18x20 ");
	lcd_locate(1,0);
	lcd_str("Nacisnij klawisz");
}

//	------------------ program testujacy DS18x20 ------------------------------------------
void TESTER()
{
	LEDTEST_ON;

	lcd_cls();
	lcd_locate(0,0);
	lcd_str("WYSZUKIWANIE...");

	uint8_t *cl=(uint8_t*)gSensorIDs;	// pobieramy wskaznik do tablicy adresow czujnikow
	for( uint8_t i=0; i<MAXSENSORS*OW_ROMCODE_SIZE; i++)
		*cl++ = 0; // kasujemy cala tablice
	czujniki_cnt = search_sensors();	// ponownie wykrywamy ile jest czujnikow i zapelniamy tablice

	_delay_ms(750);
	DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );

	_delay_ms(750);
	if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[0], &subzero, &cel, &cel_fract_bits) )
		{
		for(uint8_t i=0;i<MAXSENSORS;i++)
			{
			if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[i], &subzero, &cel, &cel_fract_bits) )
				{
				for(uint8_t ai=0; ai<8; ai++)
					{
					temp_id[i][ai] = gSensorIDs[i][ai];	// zapis id odczytanego termometru
					}
				temp_meas[i][0] = subzero;	// zapis temperatury do tablicy temperatur
				temp_meas[i][1] = cel;
				temp_meas[i][2] = cel_fract_bits;
				}
			}
		lcd_cls();
		lcd_locate(0,0);
		lcd_str("Wykryto: ");
		if(czujniki_cnt>9)
			lcd_str("ponad 9");
		else
			lcd_int(czujniki_cnt);
		lcd_locate(1,0);
		lcd_str("Nacisnij klawisz");
		}
	else
		{
		lcd_cls();
		lcd_locate(0,0);
		lcd_str(" BRAK LUB BLAD  ");
		lcd_locate(1,0);
		lcd_str("   CZUJNIKOW    ");
		tryb = 0;						// zerowanie trybu - wymuszenie rozpoczecia testu od poczatku
		}
	LEDTEST_OFF;
	}

void WYSWIETLANIE()
{
	for(int i=0; i<czujniki_cnt; i++)
	{
		while(!(PINA & (1<<PA0)))	{}
		_delay_ms(bounce);
		while(PINA & (1<<PA0))	{}
		_delay_ms(bounce);

		lcd_cls();
		lcd_locate(0,0);
		lcd_str("T");
		lcd_int(i);
		lcd_str(": ");

		if(cel<10) lcd_str(" ");			// wyswietl spacje jesli temp mniejsza niz 10st
		if(temp_meas[i][0]) lcd_str("-");	// wyswietl '-' jesli subzero==1 (temp. ujemna)
		lcd_int(temp_meas[i][1]);			// wyswietl calkowite stopnie temperatury
		lcd_str(".");						// wyswietl kropke
		lcd_int(temp_meas[i][2]);			// wyswietl dziesietne czesci stopnia
		lcd_write_data(0xdf);				// znak stopnia
		lcd_str("C"); 						// znak jednostek C

		lcd_locate(0,13);
		lcd_str("ID:");

		lcd_locate(1,0);

		for(int j=0; j<8; j++)				// petla wyswietlajaca ID sensora
		{
			lcd_locate(1,j+j);				// przyrost pozycji co 2 znaki
			if(gSensorIDs[i][j]<10) lcd_int(0);		// wyswietl '0' jesli hex < A
			lcd_hex(gSensorIDs[i][j]);		// wyswietl hex
		}
	}
	while(!(PINA & (1<<PA0)))	{}
	_delay_ms(bounce);
	while(PINA & (1<<PA0))	{}
	_delay_ms(bounce);

	if(czujniki_cnt>9)						// jesli wiecej sensorow niz 9
		{
		lcd_cls();
		lcd_locate(0,0);
		lcd_str("Brak mozliwosci");
		lcd_locate(1,0);
		lcd_str("odczytu wszyst.");

		while(!(PINA & (1<<PA0)))	{}
		_delay_ms(bounce);
		while(PINA & (1<<PA0))	{}
		_delay_ms(bounce);
		}

	lcd_cls();
	lcd_locate(0,0);
	lcd_str("Zakonczono TEST");
}

void TRYB_ZMIANA()
{
	switch(tryb)
	{
		case 0:
			tryb++;
			czujnik_cur = 0;
			LCD_START();
			break;
		case 1:
			tryb++;
			TESTER();
			break;
		case 2:
			WYSWIETLANIE();
		default:
			tryb = 0;
			break;
	}
}

//	------------------ program glowny ----------------------------------------------
int main(void)
{
	port_init();
	lcd_init();

	TRYB_ZMIANA();	// wyswietlenie pierwszego "0" przypadku stanu testera - ekran poczatkowy

// ------------------ petla glowna ----------------------------------------------
while(1)
	{
	if(!(PINA & (1<<PA0)))
		{
			_delay_ms(bounce);
			while(!(PINA & (1<<PA0)))	{}
			_delay_ms(bounce);
			TRYB_ZMIANA();
		}

	} // koniec WHILE(1)
} // koniec MAIN
