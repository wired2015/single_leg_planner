/* 
 * File: _coder_buildRRTWrapper_api.h 
 *  
 * MATLAB Coder version            : 2.7 
 * C/C++ source code generated on  : 12-Feb-2015 09:24:14 
 */

#ifndef ___CODER_BUILDRRTWRAPPER_API_H__
#define ___CODER_BUILDRRTWRAPPER_API_H__
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

/* Function Declarations */ 
extern void buildRRTWrapper_initialize(emlrtContext *aContext);
extern void buildRRTWrapper_terminate(void);
extern void buildRRTWrapper_atexit(void);
extern void buildRRTWrapper_api(const mxArray *prhs[10], const mxArray *plhs[4]);
extern void buildRRTWrapper(real_T nInitCartesianB[6], real_T nGoalCartesianB[6], real_T jointLimits[12], real_T bodyHeight, real_T U[10], real_T dt, real_T Dt, real_T kinematicConst[16], real_T threshold, int32_T legNum, emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T *success);
extern void buildRRTWrapper_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_buildRRTWrapper_api.h 
 *  
 * [EOF] 
 */
