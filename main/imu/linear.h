#ifndef BIODYN_LINEAR_ALG_H
#define BIODYN_LINEAR_ALG_H

#include "math.h"

typedef struct
{
	float x;
	float y;
	float z;
} float3;

typedef struct
{
	float w;
	float x;
	float y;
	float z;
} quaternion;

float len_f3(const float3 *v);
void normalize_f3(float3 *v);
float3 cross_f3(const float3 *a, const float3 *b);
void deg_to_rad_f3(float3 *v);

void normalize_quat(quaternion *q);
float3 rotate_f3_by_quat(const float3 *v, const quaternion *q);
quaternion conj_quat(const quaternion *q);

#endif // BIODYN_LINEAR_ALG_H