// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// XREAL One family: 134-byte IMU/MAG records over the USB-Ethernet TCP
// bridge (169.254.2.1:52998).
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#include <obs-module.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "../device_manager.h"
#include "../device_registry.h"
#include "../hid_io.h"
#include "../math_util.h"
#include "transports.h"

constexpr uint8_t RAW_HDR[] = {0x28, 0x36, 0x00, 0x00, 0x00, 0x80};
constexpr size_t RAW_HDR_LEN = sizeof(RAW_HDR);
constexpr size_t RAW_RECORD_LEN = 134;
constexpr uint32_t KIND_IMU = 11;
constexpr uint32_t KIND_MAG = 4;

struct decoded_record {
	uint32_t kind = 0;
	imu_sample imu;
	mag_sample mag;
};

static bool decode_record(const uint8_t *rec, decoded_record &out)
{
	if (std::memcmp(rec, RAW_HDR, RAW_HDR_LEN) != 0)
		return false;
	const uint32_t kind = read_u32_le(rec + 30);
	if (kind != KIND_IMU && kind != KIND_MAG)
		return false;

	const uint64_t ts_raw = read_u64_le(rec + 14);
	const uint32_t ts_us = static_cast<uint32_t>(ts_raw / 1000ULL);
	const uint32_t seq = read_u24_le(rec + 75);

	out.kind = kind;
	if (kind == KIND_IMU) {
		out.imu.ts_us = ts_us;
		out.imu.seq = seq;
		out.imu.gx = read_f32_le(rec + 34);
		out.imu.gy = read_f32_le(rec + 38);
		out.imu.gz = read_f32_le(rec + 42);
		out.imu.ax = read_f32_le(rec + 46);
		out.imu.ay = read_f32_le(rec + 50);
		out.imu.az = read_f32_le(rec + 54);
	} else {
		out.mag.ts_us = ts_us;
		out.mag.seq = seq;
		out.mag.mx = read_f32_le(rec + 58);
		out.mag.my = read_f32_le(rec + 62);
		out.mag.mz = read_f32_le(rec + 66);
		out.mag.temp_c = read_f32_le(rec + 70);
	}
	return true;
}


class winsock_scope {
public:
	winsock_scope()
	{
		WSADATA wsa = {};
		ok_ = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
	}
	~winsock_scope()
	{
		if (ok_)
			WSACleanup();
	}
	bool ok() const { return ok_; }

private:
	bool ok_ = false;
};

static bool find_local_bind_ip(std::string &ip)
{
	ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
		      GAA_FLAG_SKIP_DNS_SERVER;
	ULONG family = AF_INET;
	ULONG size = 0;
	if (GetAdaptersAddresses(family, flags, nullptr, nullptr, &size) !=
	    ERROR_BUFFER_OVERFLOW)
		return false;

	std::vector<uint8_t> buf(size);
	auto *addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
	if (GetAdaptersAddresses(family, flags, nullptr, addrs, &size) != NO_ERROR)
		return false;

	for (auto *adapter = addrs; adapter; adapter = adapter->Next) {
		if (adapter->OperStatus != IfOperStatusUp)
			continue;
		for (auto *ua = adapter->FirstUnicastAddress; ua; ua = ua->Next) {
			auto *sa = reinterpret_cast<sockaddr_in *>(ua->Address.lpSockaddr);
			if (!sa || sa->sin_family != AF_INET)
				continue;
			char text[INET_ADDRSTRLEN] = {};
			if (!inet_ntop(AF_INET, &sa->sin_addr, text, sizeof(text)))
				continue;
			std::string candidate(text);
			if (candidate.rfind("169.254.2.", 0) == 0) {
				ip = candidate;
				return true;
			}
		}
	}
	return false;
}

static SOCKET connect_tcp(const std::string &host, int port, int timeout_ms,
			  std::string &bind_used)
{
	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s == INVALID_SOCKET)
		return INVALID_SOCKET;

	std::string bind_ip;
	if (find_local_bind_ip(bind_ip)) {
		sockaddr_in local = {};
		local.sin_family = AF_INET;
		local.sin_port = 0;
		inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr);
		if (bind(s, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0)
			bind_used = bind_ip;
	}

	u_long nonblock = 1;
	ioctlsocket(s, FIONBIO, &nonblock);

	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(static_cast<u_short>(port));
	if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
		closesocket(s);
		return INVALID_SOCKET;
	}

	const int rc = connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
	if (rc == SOCKET_ERROR) {
		const int err = WSAGetLastError();
		if (err != WSAEWOULDBLOCK && err != WSAEINPROGRESS) {
			closesocket(s);
			return INVALID_SOCKET;
		}
		fd_set write_set;
		FD_ZERO(&write_set);
		FD_SET(s, &write_set);
		timeval tv = {};
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		if (select(0, nullptr, &write_set, nullptr, &tv) <= 0) {
			closesocket(s);
			return INVALID_SOCKET;
		}
		int so_error = 0;
		int len = sizeof(so_error);
		getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&so_error),
			   &len);
		if (so_error != 0) {
			closesocket(s);
			return INVALID_SOCKET;
		}
	}

	nonblock = 0;
	ioctlsocket(s, FIONBIO, &nonblock);
	BOOL nodelay = TRUE;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&nodelay),
		   sizeof(nodelay));
	DWORD recv_timeout = 1000;
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
		   reinterpret_cast<const char *>(&recv_timeout), sizeof(recv_timeout));
	return s;
}

// Returns true when at least one valid record was decoded, so the caller can
// distinguish live IMU data from a connection that only delivers garbage.
static bool consume_stream_bytes(device_manager *f, std::vector<uint8_t> &stash,
				 const uint8_t *buf, size_t len)
{
	bool decoded_any = false;
	if (!hid_device_ready(f)) {
		stash.clear();
		return false;
	}

	stash.insert(stash.end(), buf, buf + len);
	while (true) {
		if (stash.size() < RAW_HDR_LEN)
			return decoded_any;
		size_t hdr = std::string::npos;
		for (size_t i = 0; i + RAW_HDR_LEN <= stash.size(); ++i) {
			if (std::memcmp(stash.data() + i, RAW_HDR, RAW_HDR_LEN) == 0) {
				hdr = i;
				break;
			}
		}
		if (hdr == std::string::npos) {
			const size_t keep = std::min(stash.size(), RAW_HDR_LEN - 1);
			std::vector<uint8_t> tail(stash.end() - keep, stash.end());
			stash.swap(tail);
			return decoded_any;
		}
		if (hdr > 0)
			stash.erase(stash.begin(), stash.begin() + static_cast<ptrdiff_t>(hdr));
		if (stash.size() < RAW_RECORD_LEN)
			return decoded_any;

		decoded_record rec;
		if (decode_record(stash.data(), rec)) {
			if (rec.kind == KIND_IMU)
				publish_sensor_samples(f, &rec.imu, nullptr);
			else
				publish_sensor_samples(f, nullptr, &rec.mag);
			decoded_any = true;
		}
		stash.erase(stash.begin(), stash.begin() + static_cast<ptrdiff_t>(RAW_RECORD_LEN));
	}
}

void run_one_bridge_tcp_session(device_manager *f, uint32_t &seen_epoch)
{
	winsock_scope ws;
	if (!ws.ok()) {
		blog(LOG_ERROR, "[obs-nyan-real-3dof] WSAStartup failed");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	std::string ip;
	int port = 0;
	{
		std::lock_guard<std::mutex> lk(f->settings_mutex);
		ip = f->ip;
		port = f->port;
	}

	std::string bind_used;
	SOCKET s = connect_tcp(ip, port, 1500, bind_used);
	if (s == INVALID_SOCKET) {
		f->connected.store(false, std::memory_order_relaxed);
		publish_pose(f, false);
		if (f->debug_log.load(std::memory_order_relaxed))
			blog(LOG_WARNING, "[obs-nyan-real-3dof] TCP connect failed: %s:%d",
			     ip.c_str(), port);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		return;
	}

	f->connected.store(true, std::memory_order_relaxed);
	publish_pose(f, true);
	if (f->debug_log.load(std::memory_order_relaxed)) {
		blog(LOG_INFO, "[obs-nyan-real-3dof] TCP connected: %s:%d%s%s",
		     ip.c_str(), port, bind_used.empty() ? "" : " bind=",
		     bind_used.empty() ? "" : bind_used.c_str());
	}

	std::vector<uint8_t> stash;
	stash.reserve(65536);
	std::vector<uint8_t> buf(16384);
	rate_log_state rate_log;
	uint64_t last_record_ns = os_gettime_ns();

	while (!f->stop.load(std::memory_order_relaxed) &&
	       f->connect_enabled.load(std::memory_order_relaxed) &&
	       seen_epoch == f->reconnect_epoch.load(std::memory_order_relaxed)) {
		if (!hid_device_ready(f) ||
		    detected_transport_for(f) != imu_transport::one_bridge_tcp)
			break;
		// A TCP connection can stay open while delivering nothing (or
		// garbage). Tear the session down so the outer loop reconnects;
		// this is the TCP counterpart of the Air/RayNeo in-session
		// retries and makes a manual reconnect button unnecessary.
		if (os_gettime_ns() - last_record_ns > 3000000000ULL) {
			if (f->debug_log.load(std::memory_order_relaxed))
				blog(LOG_WARNING,
				     "[obs-nyan-real-3dof] TCP stream stalled "
				     "(no valid records for 3 s); reconnecting");
			break;
		}
		const int n = recv(s, reinterpret_cast<char *>(buf.data()),
				   static_cast<int>(buf.size()), 0);
		if (n > 0) {
			if (consume_stream_bytes(f, stash, buf.data(),
						 static_cast<size_t>(n)))
				last_record_ns = os_gettime_ns();
			maybe_log_sensor_rate(f, rate_log, "TCP");
			continue;
		}
		if (n == 0)
			break;
		const int err = WSAGetLastError();
		if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
			continue;
		break;
	}

	closesocket(s);
	f->connected.store(false, std::memory_order_relaxed);
	publish_pose(f, false);
	if (f->debug_log.load(std::memory_order_relaxed))
		blog(LOG_WARNING, "[obs-nyan-real-3dof] TCP disconnected");
	seen_epoch = f->reconnect_epoch.load(std::memory_order_relaxed);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}
