/*
 * sherpaTTPlanner_mex_types.h
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper'
 *
 */

#ifndef __SHERPATTPLANNER_MEX_TYPES_H__
#define __SHERPATTPLANNER_MEX_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray__common
#define struct_emxArray__common
struct emxArray__common
{
    void *data;
    int32_T *size;
    int32_T allocatedSize;
    int32_T numDimensions;
    boolean_T canFreeData;
};
#endif /*struct_emxArray__common*/
#ifndef typedef_emxArray__common
#define typedef_emxArray__common
typedef struct emxArray__common emxArray__common;
#endif /*typedef_emxArray__common*/
#ifndef struct_emxArray_int32_T_1x80
#define struct_emxArray_int32_T_1x80
struct emxArray_int32_T_1x80
{
    int32_T data[80];
    int32_T size[2];
};
#endif /*struct_emxArray_int32_T_1x80*/
#ifndef typedef_emxArray_int32_T_1x80
#define typedef_emxArray_int32_T_1x80
typedef struct emxArray_int32_T_1x80 emxArray_int32_T_1x80;
#endif /*typedef_emxArray_int32_T_1x80*/
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
#ifndef struct_emxArray_real_T_1x13
#define struct_emxArray_real_T_1x13
struct emxArray_real_T_1x13
{
    real_T data[13];
    int32_T size[2];
};
#endif /*struct_emxArray_real_T_1x13*/
#ifndef typedef_emxArray_real_T_1x13
#define typedef_emxArray_real_T_1x13
typedef struct emxArray_real_T_1x13 emxArray_real_T_1x13;
#endif /*typedef_emxArray_real_T_1x13*/
#ifndef struct_emxArray_real_T_1x9
#define struct_emxArray_real_T_1x9
struct emxArray_real_T_1x9
{
    real_T data[9];
    int32_T size[2];
};
#endif /*struct_emxArray_real_T_1x9*/
#ifndef typedef_emxArray_real_T_1x9
#define typedef_emxArray_real_T_1x9
typedef struct emxArray_real_T_1x9 emxArray_real_T_1x9;
#endif /*typedef_emxArray_real_T_1x9*/
#ifndef typedef_sherpaTTPlanner_mexStackData
#define typedef_sherpaTTPlanner_mexStackData
typedef struct
{
    union
    {
        struct
        {
            real_T T1[46500];
        } f0;
        struct
        {
            real_T T[139500];
        } f1;
    } u1;
    struct
    {
        real_T T2[46500];
        real_T T1[46500];
    } f2;
} sherpaTTPlanner_mexStackData;
#endif /*typedef_sherpaTTPlanner_mexStackData*/
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

#endif
/* End of code generation (sherpaTTPlanner_mex_types.h) */
