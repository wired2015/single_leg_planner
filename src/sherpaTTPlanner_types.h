// 
// File: sherpaTTPlanner_types.h 
//  
// MATLAB Coder version            : 2.7 
// C/C++ source code generated on  : 05-Mar-2015 11:17:25 
//

#ifndef __SHERPATTPLANNER_TYPES_H__
#define __SHERPATTPLANNER_TYPES_H__

// Include Files 
#include "rtwtypes.h"

// Type Definitions 
#include <stdio.h>
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
#ifndef struct_emxArray_int32_T_1x80
#define struct_emxArray_int32_T_1x80
struct emxArray_int32_T_1x80
{
    int data[80];
    int size[2];
};
#endif /*struct_emxArray_int32_T_1x80*/
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
#ifndef struct_emxArray_real_T_1x13
#define struct_emxArray_real_T_1x13
struct emxArray_real_T_1x13
{
    double data[13];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x13*/
#ifndef struct_emxArray_real_T_1x9
#define struct_emxArray_real_T_1x9
struct emxArray_real_T_1x9
{
    double data[9];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x9*/
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
// File trailer for sherpaTTPlanner_types.h 
//  
// [EOF] 
//
