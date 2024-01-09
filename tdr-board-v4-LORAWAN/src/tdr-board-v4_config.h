/*
 * constants.h
 *
 *  Created on: 30. maj 2022
 *      Author: au703540
 */

#ifndef TDR_BOARD_V4_CONFIG_H_
#define TDR_BOARD_V4_CONFIG_H_

/*
 * There are 5 boards, so each of them will have slightly different behaviour. But this is not really used right now, because they need to be properly
 * characterized and this hasn't been done for a veeeeeeeeeery long time. For now leave it as is.
 */
#define BOARD_NUM						-1

/*
 * Size of the memory array for the TDR data. If we assume that we're taking a measurement every 2h, we would reach 42 days worth of data with the size of 512 records.
 * Very primitive and it's one of those things that should be completely changed because storing data in SRAM is a really bad idea.
 */
#define TDR_MEMORY_SIZE                 1 // 512*2h = 1024h = 42.(6) days


/*
 * Coefficients to calculate the actual distance where the "leak" occurs from the ADC raw output. Not used as of now. With A=1 and B=0, the return values for the TDR
 * are raw ADC values which can be later converted to voltage and from that to distance (defined in the TDR 1st report).
 */
#if BOARD_NUM == 1
#define TDR_LINEAR_A                    1
#define TDR_LINEAR_B                    0
//#define THERMISTOR_CURRENT              0.0001161 //estimated so that temp vals were close to HIH

#elif BOARD_NUM == 2
#define TDR_LINEAR_A                    0.0312
#define TDR_LINEAR_B                    -4.4944
//#define THERMISTOR_CURRENT              0.000117 //estimated so that temp vals were close to HIH

#elif BOARD_NUM == 3
#define TDR_LINEAR_A                    0.034
#define TDR_LINEAR_B                    -3.659
//#define THERMISTOR_CURRENT              0.0001161 //estimated so that temp vals were close to HIH

#elif BOARD_NUM == 4
#define TDR_LINEAR_A                    0.034
#define TDR_LINEAR_B                    -3.659
//#define THERMISTOR_CURRENT              0.000113 //estimated so that temp vals were close to HIH

#elif BOARD_NUM == 5
#define TDR_LINEAR_A                    0.0355
#define TDR_LINEAR_B                    -2.0166
//#define THERMISTOR_CURRENT              0.0001147 // measured

#else
#define TDR_LINEAR_A 1
#define TDR_LINEAR_B 0

#endif


#endif /* TDR_BOARD_V4_CONFIG_H_ */
