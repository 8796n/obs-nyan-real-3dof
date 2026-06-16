// SPDX-License-Identifier: MIT
// Copyright (C) 2026 8796n <info@8796.jp>
#include "endpoint_volume.h"

#include <windows.h>

#include <endpointvolume.h>
#include <mmdeviceapi.h>

namespace {

std::wstring widen(const std::string &s)
{
	if (s.empty())
		return {};
	const int n =
		MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
	std::wstring w(n > 0 ? n - 1 : 0, L'\0');
	if (n > 0)
		MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
	return w;
}

// Open IAudioEndpointVolume for an endpoint id (caller releases). COM is assumed
// initialized on this thread (Qt UI thread). null on any failure.
IAudioEndpointVolume *open_endpoint_volume(const std::string &id)
{
	if (id.empty())
		return nullptr;
	IMMDeviceEnumerator *en = nullptr;
	if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
				    CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
				    reinterpret_cast<void **>(&en))) ||
	    !en)
		return nullptr;
	IMMDevice *dev = nullptr;
	const HRESULT hr = en->GetDevice(widen(id).c_str(), &dev);
	en->Release();
	if (FAILED(hr) || !dev)
		return nullptr;
	IAudioEndpointVolume *vol = nullptr;
	const HRESULT hr2 =
		dev->Activate(__uuidof(IAudioEndpointVolume), CLSCTX_ALL,
			      nullptr, reinterpret_cast<void **>(&vol));
	dev->Release();
	return SUCCEEDED(hr2) ? vol : nullptr;
}

} // namespace

float endpoint_volume_get(const std::string &endpoint_id)
{
	IAudioEndpointVolume *v = open_endpoint_volume(endpoint_id);
	if (!v)
		return -1.0f;
	float scalar = -1.0f;
	if (FAILED(v->GetMasterVolumeLevelScalar(&scalar)))
		scalar = -1.0f;
	v->Release();
	return scalar;
}

bool endpoint_volume_set(const std::string &endpoint_id, float scalar)
{
	IAudioEndpointVolume *v = open_endpoint_volume(endpoint_id);
	if (!v)
		return false;
	if (scalar < 0.0f)
		scalar = 0.0f;
	if (scalar > 1.0f)
		scalar = 1.0f;
	const bool ok =
		SUCCEEDED(v->SetMasterVolumeLevelScalar(scalar, nullptr));
	v->Release();
	return ok;
}

bool endpoint_volume_get_mute(const std::string &endpoint_id, bool *ok)
{
	if (ok)
		*ok = false;
	IAudioEndpointVolume *v = open_endpoint_volume(endpoint_id);
	if (!v)
		return false;
	BOOL muted = FALSE;
	if (SUCCEEDED(v->GetMute(&muted)) && ok)
		*ok = true;
	v->Release();
	return muted != FALSE;
}

bool endpoint_volume_set_mute(const std::string &endpoint_id, bool mute)
{
	IAudioEndpointVolume *v = open_endpoint_volume(endpoint_id);
	if (!v)
		return false;
	const bool ok = SUCCEEDED(v->SetMute(mute ? TRUE : FALSE, nullptr));
	v->Release();
	return ok;
}

std::string default_render_endpoint_id()
{
	IMMDeviceEnumerator *en = nullptr;
	if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
				    CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
				    reinterpret_cast<void **>(&en))) ||
	    !en)
		return "";
	IMMDevice *dev = nullptr;
	const HRESULT hr = en->GetDefaultAudioEndpoint(eRender, eConsole, &dev);
	en->Release();
	if (FAILED(hr) || !dev)
		return "";
	LPWSTR wid = nullptr;
	std::string out;
	if (SUCCEEDED(dev->GetId(&wid)) && wid) {
		const int n = WideCharToMultiByte(CP_UTF8, 0, wid, -1, nullptr,
						  0, nullptr, nullptr);
		if (n > 1) {
			out.resize(static_cast<size_t>(n - 1));
			WideCharToMultiByte(CP_UTF8, 0, wid, -1, out.data(), n,
					    nullptr, nullptr);
		}
	}
	if (wid)
		CoTaskMemFree(wid);
	dev->Release();
	return out;
}
