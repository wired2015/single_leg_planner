// 
// File: buildRRTWrapper_types.h 
//  
// MATLAB Coder version            : 2.7 
// C/C++ source code generated on  : 05-Feb-2015 15:38:22 
//

#ifndef __BUILDRRTWRAPPER_TYPES_H__
#define __BUILDRRTWRAPPER_TYPES_H__

// Include Files 
#include "rtwtypes.h"

// Type Definitions 
#ifndef struct_emxArray__common
#define struct_emxArray__common
struct emxArray__common
{
    void *data;
    int *size;
    int allocatedSize;
    int numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray__common*/
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T
{
    double *data;
    int *size;
    int allocatedSize;
    int numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray_real_T*/

#endif
// 
// File trailer for buildRRTWrapper_types.h 
//  
// [EOF] 
//
