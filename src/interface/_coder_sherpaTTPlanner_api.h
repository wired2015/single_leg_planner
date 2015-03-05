/* 
 * File: _coder_sherpaTTPlanner_api.h 
 *  
 * MATLAB Coder version            : 2.7 
 * C/C++ source code generated on  : 05-Mar-2015 15:01:21 
 */

#ifndef ___CODER_SHERPATTPLANNER_API_H__
#define ___CODER_SHERPATTPLANNER_API_H__
/* Include Files */ 
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Type Definitions */ 
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T
{
    real_T *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_real_T*/
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /*typedef_emxArray_real_T*/
#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct
{
    real_T l1;
    real_T l2;
    real_T l3;
    real_T l4;
    real_T l5;
    real_T l6;
    real_T l7;
    real_T l8;
    real_T zeta;
    real_T r;
    real_T B2PXOffset;
    real_T B2PZOffset;
    real_T legAngleOffset[4];
} struct0_T;
#endif /*typedef_struct0_T*/

/* Function Declarations */ 
extern void sherpaTTPlanner_initialize(emlrtContext *aContext);
extern void sherpaTTPlanner_terminate(void);
extern void sherpaTTPlanner_atexit(void);
extern void buildBiDirectionalRRTWrapper_api(const mxArray *prhs[8], const mxArray *plhs[5]);
extern void buildBiDirectionalRRTWrapper(real_T nInitCartesianB[6], real_T nGoalCartesianB[6], real_T phiInit, real_T omegaInit, real_T jointLimits[20], struct0_T *kC, int32_T legNum, real_T uBDot[6], emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T *success);
extern void buildRRTWrapper_api(const mxArray *prhs[8], const mxArray *plhs[4]);
extern void buildRRTWrapper(real_T nInitCartesianB[6], real_T nGoalCartesianB[6], real_T phiInit, real_T omegaInit, real_T jointLimits[20], struct0_T *kC, int32_T legNum, real_T uBDot[6], emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T *success);
extern void randomStateGenerator_api(const mxArray *prhs[5], const mxArray *plhs[1]);
extern void randomStateGenerator(int32_T N, real_T jointLimits[20], struct0_T *kC, real_T panHeight, int32_T legNum, emxArray_real_T *states);
extern void sherpaTTPlanner_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_sherpaTTPlanner_api.h 
 *  
 * [EOF] 
 */
