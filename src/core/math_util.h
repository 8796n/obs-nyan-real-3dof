// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Quaternion / vector math and little/big-endian byte readers shared across
// the tracker, the protocol decoders and the renderer. Header-only.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#include "nyan_time.h"
#include "nyan_types.h"

constexpr double PI = 3.14159265358979323846;

static double clampd(double v, double lo, double hi)
{
	if (!std::isfinite(v))
		return lo;
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}

static double wrap_angle(double a)
{
	while (a > PI)
		a -= 2.0 * PI;
	while (a < -PI)
		a += 2.0 * PI;
	return a;
}

static quatd quat_normalize(quatd q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (!std::isfinite(n) || n <= 1e-12)
		return {};
	q.w /= n;
	q.x /= n;
	q.y /= n;
	q.z /= n;
	return q;
}

static quatd quat_inverse(quatd q)
{
	const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	if (!std::isfinite(n2) || n2 <= 1e-12)
		return {};
	return {q.w / n2, -q.x / n2, -q.y / n2, -q.z / n2};
}

static quatd quat_multiply(quatd a, quatd b)
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	};
}

static quatd quat_from_rot_x(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), std::sin(h), 0.0, 0.0};
}

static quatd quat_from_yaw_y(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), 0.0, std::sin(h), 0.0};
}

static quatd quat_from_rot_z(double rad)
{
	const double h = 0.5 * rad;
	return {std::cos(h), 0.0, 0.0, std::sin(h)};
}

static vec3d rotate_vector(quatd q, vec3d v)
{
	const double tx = 2.0 * (q.y * v.z - q.z * v.y);
	const double ty = 2.0 * (q.z * v.x - q.x * v.z);
	const double tz = 2.0 * (q.x * v.y - q.y * v.x);
	return {
		v.x + q.w * tx + (q.y * tz - q.z * ty),
		v.y + q.w * ty + (q.z * tx - q.x * tz),
		v.z + q.w * tz + (q.x * ty - q.y * tx),
	};
}

static vec3d rotate_world_vector_into_body(quatd q, vec3d world)
{
	return rotate_vector(quat_normalize(quat_inverse(q)), world);
}

static quatd quat_derivative(quatd q, double wx, double wy, double wz)
{
	const quatd m = quat_multiply(q, {0.0, wx, wy, wz});
	return {0.5 * m.w, 0.5 * m.x, 0.5 * m.y, 0.5 * m.z};
}

static double yaw_from_quat_heading(quatd q, double fallback = 0.0)
{
	const vec3d f = rotate_vector(quat_normalize(q), {0.0, 0.0, -1.0});
	const double hn = std::sqrt(f.x * f.x + f.z * f.z);
	if (!std::isfinite(hn) || hn < 1e-6)
		return fallback;
	return wrap_angle(std::atan2(-(f.x / hn), -(f.z / hn)));
}

static uint32_t elapsed_us32(uint32_t now, uint32_t then)
{
	return static_cast<uint32_t>(now - then);
}

static uint16_t read_u16_le(const uint8_t *p)
{
	return static_cast<uint16_t>(p[0]) |
	       static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
	return static_cast<uint32_t>(p[0]) |
	       (static_cast<uint32_t>(p[1]) << 8) |
	       (static_cast<uint32_t>(p[2]) << 16) |
	       (static_cast<uint32_t>(p[3]) << 24);
}

static uint64_t read_u64_le(const uint8_t *p)
{
	uint64_t v = 0;
	for (int i = 7; i >= 0; --i)
		v = (v << 8) | p[i];
	return v;
}

static uint32_t read_u24_le(const uint8_t *p)
{
	return static_cast<uint32_t>(p[0]) |
	       (static_cast<uint32_t>(p[1]) << 8) |
	       (static_cast<uint32_t>(p[2]) << 16);
}

static int16_t read_i16_le(const uint8_t *p)
{
	return static_cast<int16_t>(read_u16_le(p));
}

static int32_t read_i32_le(const uint8_t *p)
{
	return static_cast<int32_t>(read_u32_le(p));
}

static int16_t read_i16_be(const uint8_t *p)
{
	const uint16_t u = static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
						static_cast<uint16_t>(p[1]));
	return static_cast<int16_t>(u);
}

static int32_t read_i32_be(const uint8_t *p)
{
	const uint32_t u = (static_cast<uint32_t>(p[0]) << 24) |
			   (static_cast<uint32_t>(p[1]) << 16) |
			   (static_cast<uint32_t>(p[2]) << 8) |
			   static_cast<uint32_t>(p[3]);
	return static_cast<int32_t>(u);
}

static int32_t read_i24_le(const uint8_t *p)
{
	int32_t v = static_cast<int32_t>(read_u24_le(p));
	if (v & 0x800000)
		v -= 0x1000000;
	return v;
}


static float read_f32_le(const uint8_t *p)
{
	uint32_t u = read_u32_le(p);
	float f = 0.0f;
	std::memcpy(&f, &u, sizeof(f));
	return f;
}

static float read_f32_be(const uint8_t *p)
{
	const uint8_t b[4] = {p[3], p[2], p[1], p[0]};
	float f = 0.0f;
	std::memcpy(&f, b, sizeof(f));
	return f;
}

static uint32_t now_us32()
{
	return static_cast<uint32_t>(nyan_now_ns() / 1000ULL);
}
