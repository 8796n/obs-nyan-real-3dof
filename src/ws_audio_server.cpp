// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
#include "ws_audio_server.h"

#include <obs-module.h>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#include <windows.h>
#include <wincrypt.h>

#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr size_t MAX_HEADER_BYTES = 8192;
constexpr size_t MAX_FRAME_BYTES = 4 * 1024 * 1024;

bool sha1(const uint8_t *data, size_t len, uint8_t out[20])
{
	HCRYPTPROV prov = 0;
	HCRYPTHASH hash = 0;
	bool ok = false;
	if (CryptAcquireContextW(&prov, nullptr, nullptr, PROV_RSA_FULL,
				 CRYPT_VERIFYCONTEXT)) {
		if (CryptCreateHash(prov, CALG_SHA1, 0, 0, &hash)) {
			DWORD hash_len = 20;
			ok = CryptHashData(hash, data, static_cast<DWORD>(len),
					   0) &&
			     CryptGetHashParam(hash, HP_HASHVAL, out, &hash_len,
					       0);
			CryptDestroyHash(hash);
		}
		CryptReleaseContext(prov, 0);
	}
	return ok;
}

std::string base64(const uint8_t *data, size_t len)
{
	static const char tbl[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string out;
	for (size_t i = 0; i < len; i += 3) {
		uint32_t v = data[i] << 16;
		if (i + 1 < len)
			v |= data[i + 1] << 8;
		if (i + 2 < len)
			v |= data[i + 2];
		out += tbl[(v >> 18) & 63];
		out += tbl[(v >> 12) & 63];
		out += i + 1 < len ? tbl[(v >> 6) & 63] : '=';
		out += i + 2 < len ? tbl[v & 63] : '=';
	}
	return out;
}

bool recv_exact(SOCKET s, uint8_t *buf, size_t len)
{
	while (len) {
		const int n = recv(s, reinterpret_cast<char *>(buf),
				   static_cast<int>(len), 0);
		if (n <= 0)
			return false;
		buf += n;
		len -= static_cast<size_t>(n);
	}
	return true;
}

bool send_all(SOCKET s, const uint8_t *buf, size_t len)
{
	while (len) {
		const int n = send(s, reinterpret_cast<const char *>(buf),
				   static_cast<int>(len), 0);
		if (n <= 0)
			return false;
		buf += n;
		len -= static_cast<size_t>(n);
	}
	return true;
}

// Server-to-client frame (unmasked).
bool send_frame(SOCKET s, uint8_t opcode, const uint8_t *payload, size_t len)
{
	std::vector<uint8_t> hdr;
	hdr.push_back(0x80 | opcode);
	if (len < 126) {
		hdr.push_back(static_cast<uint8_t>(len));
	} else if (len <= 0xFFFF) {
		hdr.push_back(126);
		hdr.push_back(static_cast<uint8_t>(len >> 8));
		hdr.push_back(static_cast<uint8_t>(len));
	} else {
		hdr.push_back(127);
		for (int i = 7; i >= 0; i--)
			hdr.push_back(static_cast<uint8_t>(len >> (8 * i)));
	}
	return send_all(s, hdr.data(), hdr.size()) &&
	       (len == 0 || send_all(s, payload, len));
}

// Case-insensitive single-line header lookup in a raw request block.
std::string find_header(const std::string &req, const char *name)
{
	std::string lower = req;
	for (char &c : lower)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	std::string key = name;
	for (char &c : key)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	key = "\n" + key + ":";
	const size_t at = lower.find(key);
	if (at == std::string::npos)
		return "";
	size_t begin = at + key.size();
	size_t end = req.find("\r\n", begin);
	if (end == std::string::npos)
		end = req.size();
	std::string value = req.substr(begin, end - begin);
	while (!value.empty() && value.front() == ' ')
		value.erase(value.begin());
	while (!value.empty() &&
	       (value.back() == ' ' || value.back() == '\r'))
		value.pop_back();
	return value;
}

bool websocket_handshake(SOCKET s)
{
	std::string req;
	uint8_t c = 0;
	while (req.size() < MAX_HEADER_BYTES &&
	       req.find("\r\n\r\n") == std::string::npos) {
		if (!recv_exact(s, &c, 1))
			return false;
		req += static_cast<char>(c);
	}
	const std::string key = find_header(req, "Sec-WebSocket-Key");
	if (key.empty()) {
		const char *bad = "HTTP/1.1 400 Bad Request\r\n\r\n";
		send_all(s, reinterpret_cast<const uint8_t *>(bad),
			 strlen(bad));
		return false;
	}
	const std::string accept_src =
		key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	uint8_t digest[20];
	if (!sha1(reinterpret_cast<const uint8_t *>(accept_src.data()),
		  accept_src.size(), digest))
		return false;
	const std::string resp = "HTTP/1.1 101 Switching Protocols\r\n"
				 "Upgrade: websocket\r\n"
				 "Connection: Upgrade\r\n"
				 "Sec-WebSocket-Accept: " +
				 base64(digest, 20) + "\r\n\r\n";
	return send_all(s, reinterpret_cast<const uint8_t *>(resp.data()),
			resp.size());
}

} // namespace

struct ws_audio_server::impl {
	SOCKET listener = INVALID_SOCKET;
	std::thread accept_thread;
	std::atomic<bool> stopping{false};
	ws_server_callbacks cb;
	uint64_t next_conn_id = 1;

	struct connection {
		uint64_t id = 0;
		SOCKET sock = INVALID_SOCKET;
		std::thread th;
		std::atomic<bool> done{false};
	};
	std::mutex conns_mutex;
	std::list<std::unique_ptr<connection>> conns;

	void run_connection(connection *conn)
	{
		if (websocket_handshake(conn->sock))
			frame_loop(conn);
		shutdown(conn->sock, SD_BOTH);
		closesocket(conn->sock);
		conn->sock = INVALID_SOCKET;
		if (cb.on_close)
			cb.on_close(conn->id);
		conn->done = true;
	}

	void frame_loop(connection *conn)
	{
		std::vector<uint8_t> message; // collects fragmented payloads
		uint8_t message_op = 0;
		for (;;) {
			uint8_t hdr[2];
			if (!recv_exact(conn->sock, hdr, 2))
				return;
			const bool fin = (hdr[0] & 0x80) != 0;
			const uint8_t opcode = hdr[0] & 0x0F;
			const bool masked = (hdr[1] & 0x80) != 0;
			uint64_t len = hdr[1] & 0x7F;
			if (len == 126) {
				uint8_t ext[2];
				if (!recv_exact(conn->sock, ext, 2))
					return;
				len = (static_cast<uint64_t>(ext[0]) << 8) |
				      ext[1];
			} else if (len == 127) {
				uint8_t ext[8];
				if (!recv_exact(conn->sock, ext, 8))
					return;
				len = 0;
				for (uint8_t b : ext)
					len = (len << 8) | b;
			}
			if (!masked || len > MAX_FRAME_BYTES)
				return; // protocol violation / abuse
			uint8_t mask[4];
			if (!recv_exact(conn->sock, mask, 4))
				return;
			std::vector<uint8_t> payload(static_cast<size_t>(len));
			if (len &&
			    !recv_exact(conn->sock, payload.data(),
					payload.size()))
				return;
			for (size_t i = 0; i < payload.size(); i++)
				payload[i] ^= mask[i & 3];

			if (opcode == 8) { // close
				send_frame(conn->sock, 8, nullptr, 0);
				return;
			}
			if (opcode == 9) { // ping
				send_frame(conn->sock, 10, payload.data(),
					   payload.size());
				continue;
			}
			if (opcode == 10) // pong
				continue;

			if (opcode == 1 || opcode == 2) {
				message.assign(payload.begin(), payload.end());
				message_op = opcode;
			} else if (opcode == 0 && message_op) {
				if (message.size() + payload.size() >
				    MAX_FRAME_BYTES)
					return;
				message.insert(message.end(), payload.begin(),
					       payload.end());
			} else {
				continue;
			}
			if (!fin)
				continue;

			if (message_op == 1 && cb.on_text) {
				message.push_back(0);
				obs_data_t *msg = obs_data_create_from_json(
					reinterpret_cast<const char *>(
						message.data()));
				if (msg) {
					cb.on_text(conn->id, msg);
					obs_data_release(msg);
				}
			} else if (message_op == 2 && cb.on_binary) {
				cb.on_binary(conn->id, message.data(),
					     message.size());
			}
			message.clear();
			message_op = 0;
		}
	}

	void reap_finished()
	{
		std::lock_guard<std::mutex> lk(conns_mutex);
		for (auto it = conns.begin(); it != conns.end();) {
			if ((*it)->done) {
				if ((*it)->th.joinable())
					(*it)->th.join();
				it = conns.erase(it);
			} else {
				++it;
			}
		}
	}

	void accept_loop()
	{
		for (;;) {
			SOCKET s = accept(listener, nullptr, nullptr);
			if (s == INVALID_SOCKET)
				return; // listener closed by stop()
			if (stopping) {
				closesocket(s);
				return;
			}
			// Realtime PCM in small frames: never let Nagle batch
			// them, that only adds delivery jitter on loopback.
			BOOL nodelay = TRUE;
			setsockopt(s, IPPROTO_TCP, TCP_NODELAY,
				   reinterpret_cast<const char *>(&nodelay),
				   sizeof(nodelay));
			reap_finished();
			auto conn = std::make_unique<connection>();
			conn->id = next_conn_id++;
			conn->sock = s;
			connection *raw = conn.get();
			{
				std::lock_guard<std::mutex> lk(conns_mutex);
				conns.push_back(std::move(conn));
			}
			raw->th = std::thread(
				[this, raw]() { run_connection(raw); });
		}
	}
};

ws_audio_server::~ws_audio_server()
{
	stop();
}

bool ws_audio_server::start(uint16_t port, ws_server_callbacks cb)
{
	stop();

	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return false;

	SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listener == INVALID_SOCKET) {
		WSACleanup();
		return false;
	}
	// Loopback only: this is a local extension ingest, never exposed.
	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
	if (bind(listener, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) !=
		    0 ||
	    listen(listener, 4) != 0) {
		blog(LOG_WARNING,
		     "[obs-nyan-real-3dof] audio wall: cannot listen on 127.0.0.1:%u (in use?)",
		     port);
		closesocket(listener);
		WSACleanup();
		return false;
	}

	impl_ = new impl();
	impl_->listener = listener;
	impl_->cb = std::move(cb);
	impl_->accept_thread = std::thread([this]() { impl_->accept_loop(); });
	running_ = true;
	port_ = port;
	blog(LOG_INFO,
	     "[obs-nyan-real-3dof] audio wall: WebSocket ingest on 127.0.0.1:%u",
	     port);
	return true;
}

void ws_audio_server::stop()
{
	if (!impl_)
		return;
	impl_->stopping = true;
	closesocket(impl_->listener); // unblocks accept()
	{
		std::lock_guard<std::mutex> lk(impl_->conns_mutex);
		for (auto &conn : impl_->conns) {
			if (conn->sock != INVALID_SOCKET)
				shutdown(conn->sock, SD_BOTH);
		}
	}
	if (impl_->accept_thread.joinable())
		impl_->accept_thread.join();
	for (;;) {
		std::unique_ptr<impl::connection> conn;
		{
			std::lock_guard<std::mutex> lk(impl_->conns_mutex);
			if (impl_->conns.empty())
				break;
			conn = std::move(impl_->conns.front());
			impl_->conns.pop_front();
		}
		if (conn->th.joinable())
			conn->th.join();
	}
	delete impl_;
	impl_ = nullptr;
	running_ = false;
	port_ = 0;
	WSACleanup();
}
