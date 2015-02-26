/* 
 * File: buildBiDirectionalRRTWrapper_types.h 
 *  
 * MATLAB Coder version            : 2.7 
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49 
 */

#ifndef __BUILDBIDIRECTIONALRRTWRAPPER_TYPES_H__
#define __BUILDBIDIRECTIONALRRTWRAPPER_TYPES_H__

/* Include Files */ 
#include "rtwtypes.h"

/* Type Definitions */ 
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
#ifndef typedef_emxArray__common
#define typedef_emxArray__common
typedef struct emxArray__common emxArray__common;
#endif /*typedef_emxArray__common*/
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
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /*typedef_emxArray_real_T*/
#ifndef struct_emxArray_real_T_1x10
#define struct_emxArray_real_T_1x10
struct emxArray_real_T_1x10
{
    double data[10];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x10*/
#ifndef typedef_emxArray_real_T_1x10
#define typedef_emxArray_real_T_1x10
typedef struct emxArray_real_T_1x10 emxArray_real_T_1x10;
#endif /*typedef_emxArray_real_T_1x10*/
#ifndef struct_emxArray_real_T_1x1000
#define struct_emxArray_real_T_1x1000
struct emxArray_real_T_1x1000
{
    double data[1000];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x1000*/
#ifndef typedef_emxArray_real_T_1x1000
#define typedef_emxArray_real_T_1x1000
typedef struct emxArray_real_T_1x1000 emxArray_real_T_1x1000;
#endif /*typedef_emxArray_real_T_1x1000*/
#ifndef struct_emxArray_real_T_1x16
#define struct_emxArray_real_T_1x16
struct emxArray_real_T_1x16
{
    double data[16];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x16*/
#ifndef typedef_emxArray_real_T_1x16
#define typedef_emxArray_real_T_1x16
typedef struct emxArray_real_T_1x16 emxArray_real_T_1x16;
#endif /*typedef_emxArray_real_T_1x16*/
#ifndef struct_emxArray_real_T_1x5
#define struct_emxArray_real_T_1x5
struct emxArray_real_T_1x5
{
    double data[5];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x5*/
#ifndef typedef_emxArray_real_T_1x5
#define typedef_emxArray_real_T_1x5
typedef struct emxArray_real_T_1x5 emxArray_real_T_1x5;
#endif /*typedef_emxArray_real_T_1x5*/
#ifndef struct_emxArray_real_T_5x13
#define struct_emxArray_real_T_5x13
struct emxArray_real_T_5x13
{
    double data[65];
    int size[2];
};
#endif /*struct_emxArray_real_T_5x13*/
#ifndef typedef_emxArray_real_T_5x13
#define typedef_emxArray_real_T_5x13
typedef struct emxArray_real_T_5x13 emxArray_real_T_5x13;
#endif /*typedef_emxArray_real_T_5x13*/
#ifndef typedef_struct0_T
#define typedef_struct0_T
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
#endif /*typedef_struct0_T*/

#endif
/* 
 * File trailer for buildBiDirectionalRRTWrapper_types.h 
 *  
 * [EOF] 
 */
