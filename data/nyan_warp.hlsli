// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
// Shared 3DoF backward-warp math (no texture or parameter declarations) so the
// OBS effect and the standalone shader compute the identical warp - the single
// source of truth for the GPU side, matching warp_params on the CPU side. Each
// backend prepends this to its own wrapper (uniforms/cbuffer + sampler + entry
// points) before compiling. Plain HLSL only, valid both in the libobs effect
// language and raw D3DCompile: no #include, no out/inout params, no bool.

float3 nyan_rotate_by_quat(float3 v, float4 q)
{
	float3 qv = float3(q.y, q.z, q.w);
	float3 t = 2.0 * cross(qv, v);
	return v + (q.x * t) + cross(qv, t);
}

// Backward-warp a head-locked output pixel (out_uv, 0..1) onto the virtual
// screen (flat when screen_curve<=0, else a cylinder) and return where to sample
// the source: .xy = source UV, .z = 1 on a hit, 0 on a miss / off-screen (the
// caller then draws black).
float3 nyan_warp_uv(float2 out_uv, float4 pose_q, float2 tan_half_fov,
		    float screen_distance_m, float2 screen_half_size_m,
		    float screen_curve, float3 eye_pos_m)
{
	float2 ndc = float2((out_uv.x * 2.0 - 1.0) * tan_half_fov.x,
			    (1.0 - out_uv.y * 2.0) * tan_half_fov.y);
	float3 dir = normalize(float3(ndc.x, ndc.y, -1.0));
	float3 d = nyan_rotate_by_quat(dir, normalize(pose_q));
	if (screen_distance_m <= 1e-4 || screen_half_size_m.x <= 1e-6 ||
	    screen_half_size_m.y <= 1e-6)
		return float3(0.0, 0.0, 0.0);

	float curve = clamp(screen_curve, 0.0, 3.0);
	float2 suv;
	if (curve <= 0.0001) {
		if (d.z >= -1e-4)
			return float3(0.0, 0.0, 0.0);
		float t = (-screen_distance_m - eye_pos_m.z) / d.z;
		if (t <= 1e-4)
			return float3(0.0, 0.0, 0.0);
		float2 hit = eye_pos_m.xy + d.xy * t;
		suv = float2(0.5 + hit.x / (2.0 * screen_half_size_m.x),
			     0.5 - hit.y / (2.0 * screen_half_size_m.y));
	} else {
		float radius = screen_distance_m / curve;
		float center_z = radius - screen_distance_m;
		float ox = eye_pos_m.x;
		float oz = eye_pos_m.z - center_z;
		float a = d.x * d.x + d.z * d.z;
		float b = 2.0 * (ox * d.x + oz * d.z);
		float c = ox * ox + oz * oz - radius * radius;
		float disc = b * b - 4.0 * a * c;
		if (a <= 1e-6 || disc < 0.0)
			return float3(0.0, 0.0, 0.0);
		float root = sqrt(disc);
		float t = (-b + root) / (2.0 * a);
		if (t <= 1e-4)
			t = (-b - root) / (2.0 * a);
		if (t <= 1e-4)
			return float3(0.0, 0.0, 0.0);
		float3 hit = eye_pos_m + d * t;
		float theta = atan2(hit.x, center_z - hit.z);
		float arc_x = radius * theta;
		suv = float2(0.5 + arc_x / (2.0 * screen_half_size_m.x),
			     0.5 - hit.y / (2.0 * screen_half_size_m.y));
	}

	if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0)
		return float3(0.0, 0.0, 0.0);
	return float3(suv.x, suv.y, 1.0);
}

// Flat mirror (pose-follow off): map a per-eye output pixel (out_uv, 0..1) onto
// the source with aspect-fit letterbox/pillarbox so the picture is not stretched.
// 'fit' = the fraction of the view the source fills per axis (the CPU derives it
// from source vs per-eye view aspect; the smaller axis gets bars). Returns
// .xy = source UV, .z = 1 inside the image, 0 in the black bars.
float3 nyan_flat_uv(float2 out_uv, float2 fit)
{
	float2 f = max(fit, float2(1e-4, 1e-4));
	float2 origin = (float2(1.0, 1.0) - f) * 0.5;
	float2 suv = (out_uv - origin) / f;
	if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0)
		return float3(0.0, 0.0, 0.0);
	return float3(suv.x, suv.y, 1.0);
}

// Off-screen indicator: light the view edge(s) on the side the wall lies, so the
// user knows which way to turn. 'dir' (unit, +x right / +y up) points at the
// wall; its components pick the edges (left wall -> left edge; left-up -> left +
// top). 'band' is the glow thickness per axis in centered-coord units (CPU sets
// it to a fixed pixel width so horizontal/vertical match). 'intensity' 0..1
// (0 = visible / disabled), no distance falloff. White. out_uv = per-eye 0..1.
float3 nyan_offscreen_glow(float2 out_uv, float2 dir, float2 band, float intensity)
{
	if (intensity <= 0.001)
		return float3(0.0, 0.0, 0.0);
	// Centered coords, +x right / +y up (out_uv.y grows downward).
	float2 c = float2(out_uv.x * 2.0 - 1.0, 1.0 - out_uv.y * 2.0);
	float2 inner = float2(1.0, 1.0) - max(band, float2(1e-4, 1e-4));
	// Proximity to each edge (1 at the edge, 0 at the band's inner limit).
	float L = saturate((-c.x - inner.x) / band.x);
	float R = saturate((c.x - inner.x) / band.x);
	float T = saturate((c.y - inner.y) / band.y);
	float Bm = saturate((-c.y - inner.y) / band.y);
	// Which edges, by the wall direction's sign per axis.
	float g = L * saturate(-dir.x) + R * saturate(dir.x) +
		  T * saturate(dir.y) + Bm * saturate(-dir.y);
	return float3(1.0, 1.0, 1.0) * saturate(g) * intensity;
}
