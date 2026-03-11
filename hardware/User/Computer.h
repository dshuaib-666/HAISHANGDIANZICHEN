#ifndef __COMPUTER_H
#define __COMPUTER_H
#include "Config.h"

extern void PushAdValue(float AdVaule);

extern void ComputerAdcValue(void);

extern  void ZeroVerify(void);

extern  void StartMeasure(void);

extern void ComputeInsAvgData(void);

//extern float L_Ncc_Rates[11];

//extern double CurrentWeight;
//extern char ZeroMarkFlag;

extern void GetWeightValue(void);

void FilterWeightValue(void);

void simgle_dispose(void);

void SetWeightToAdcValue(void);
#endif
