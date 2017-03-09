/*
 * memcheck.h
 *
 * Created: 16.01.2017 20:33:47
 *  Author: buero
 */ 


#ifndef MEMCHECK_H_
#define MEMCHECK_H_


#define DEBUG_MEMCHECK			0  //activate functions for memcheck (shows free sram)	http://rn-wissen.de/wiki/index.php?title=Speicherverbrauch_bestimmen_mit_avr-gcc

// Mask to init SRAM and check against
#define MASK 0xA3

extern unsigned short get_mem_unused (void);




#endif /* MEMCHECK_H_ */