/* http://rn-wissen.de/wiki/index.php/Gleitender_Mittelwert_in_C */

#ifndef FLOAT_AVERAGE_H
#define FLOAT_AVERAGE_H

#include <Arduino.h>

//#include <inttypes.h>		//adds 32-bit-integers
#include <avr/io.h>

// Ueber wieviele Werte soll der gleitende Mittelwert berechnet werden?
// Zulaessige Werte 1..255
#define SIZE_OF_AVG  8		//should really be 4, 8, 16 or 32 -> otherwise many bytes are wasted for div by SIZE_OF_AVG

// Datentyp, ueber den der gleitende Mittelwert berechnet werden soll.
typedef float FloatAverage_t;


// Wird nur intern fuer die Durchschnittsberechnung benutzt.
// Muss Zahlen fassen koennen, die SIZE_OF_AVG mal groesser als tFloatAvgType sind.
typedef float TempSum_t;


class floatAverageClass {
	public:
		// Initialisiert das Filter mit einem Startwert.
		void setAverage(FloatAverage_t i_DefaultValue);

		// Schreibt einen neuen Wert in das Filter.
		void AddToFloatAverage(FloatAverage_t i_ui16NewValue);

		// Berechnet den Durchschnitt aus den letzten SIZE_OF_AVG eingetragenen Werten.
		FloatAverage_t getAverage();
		
		// Gibt den aktuellsten, d.h. den zuletzt zum Array hinzugefügten Wert zurück
		FloatAverage_t getLeastAddedValue();
		
		// gibt den ältesten, d.h. den als nächstes zu überschreibenden Wert des Arrays zurück
		FloatAverage_t getOldestAddedValue();
		
	private:
		// Die Struktur, in der die Daten zwischengespeichert werden
		FloatAverage_t aData[SIZE_OF_AVG];
		uint8_t IndexNextValue;

};

//extern floatAverageClass floatAverage;



#endif