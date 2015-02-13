// 
// File: buildRRTWrapper_types.h 
//  
// MATLAB Coder version            : 2.7 
// C/C++ source code generated on  : 13-Feb-2015 15:29:21 
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
#ifndef struct_emxArray_real_T_1x11
#define struct_emxArray_real_T_1x11
struct emxArray_real_T_1x11
{
    double data[11];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x11*/
#ifndef struct_emxArray_real_T_1x16
#define struct_emxArray_real_T_1x16
struct emxArray_real_T_1x16
{
    double data[16];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x16*/
#ifndef struct_emxArray_real_T_1x6
#define struct_emxArray_real_T_1x6
struct emxArray_real_T_1x6
{
    double data[6];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x6*/
#ifndef struct_emxArray_real_T_5x3
#define struct_emxArray_real_T_5x3
struct emxArray_real_T_5x3
{
    double data[15];
    int size[2];
};
#endif /*struct_emxArray_real_T_5x3*/
typedef struct
{
    double l1;
    double l2;
    double l3;
    double l4;
    double l5;
    double l6;
    double l7;
    double l8;
    double zeta;
    double r;
    double B2PXOffset;
    double B2PZOffset;
    double legAngleOffset[4];
} struct0_T;

#endif
// 
// File trailer for buildRRTWrapper_types.h 
//  
// [EOF] 
//
