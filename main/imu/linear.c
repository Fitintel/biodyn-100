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

float3 rotate_f3_by_quat(const float3 *v, const quaternion *q)
{
	quaternion v_q = {0, v->x, v->y, v->z};
	quaternion q_conj = {q->w, -q->x, -q->y, -q->z};

	quaternion temp = {
		q->w * v_q.w - q->x * v_q.x - q->y * v_q.y - q->z * v_q.z,
		q->w * v_q.x + q->x * v_q.w + q->y * v_q.z - q->z * v_q.y,
		q->w * v_q.y - q->x * v_q.z + q->y * v_q.w + q->z * v_q.x,
		q->w * v_q.z + q->x * v_q.y - q->y * v_q.x + q->z * v_q.w};
	quaternion result = {
		temp.w * q_conj.w - temp.x * q_conj.x - temp.y * q_conj.y - temp.z * q_conj.z,
		temp.w * q_conj.x + temp.x * q_conj.w + temp.y * q_conj.z - temp.z * q_conj.y,
		temp.w * q_conj.y - temp.x * q_conj.z + temp.y * q_conj.w + temp.z * q_conj.x,
		temp.w * q_conj.z + temp.x * q_conj.y - temp.y * q_conj.x + temp.z * q_conj.w};

	float3 rotated_v = {result.x, result.y, result.z};
	return rotated_v;
}

quaternion conj_quat(const quaternion *q)
{
	quaternion conj = {q->w, -q->x, -q->y, -q->z};
	return conj;
}