#include "linear.h"

float len_f3(const float3 *v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void normalize_f3(float3 *v)
{
	float length = len_f3(v);
	if (length > 0)
	{
		v->x /= length;
		v->y /= length;
		v->z /= length;
	}
}


float3 cross_f3(const float3 *a, const float3 *b)
{
	float3 result;
	result.x = a->y * b->z - a->z * b->y;
	result.y = a->z * b->x - a->x * b->z;
	result.z = a->x * b->y - a->y * b->x;
	return result;
}

void deg_to_rad_f3(float3 *v)
{
	v->x *= (3.14159265f / 180.0f);
	v->y *= (3.14159265f / 180.0f);
	v->z *= (3.14159265f / 180.0f);
}

void normalize_quat(quaternion *q)
{
	float length = sqrtf(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
	if (length > 0)
	{
		q->x /= length;
		q->y /= length;
		q->z /= length;
		q->w /= length;
	}
}