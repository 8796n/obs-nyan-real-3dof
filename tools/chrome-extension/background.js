// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Service worker: multiplexes the content scripts' PCM onto one WebSocket
// to the nyan Real Audio Wall (ws://127.0.0.1:<port>) and reports each
// stream's bearing as the normalized window-center position across the
// virtual desktop (computed here in Chrome's DIP space, so display scaling
// stays consistent).
"use strict";

// Version of the ingest protocol, sent as "v" in every meta message. Bump
// together with WS_PROTOCOL_VERSION in src/audio-wall-source.cpp on breaking
// changes only (additive fields don't count); see CONTRIBUTING.md.
const PROTOCOL_VERSION = 1;

let ws = null;
let wsReady = false;
let reconnectTimer = null;
let nextStreamId = 1;
// port -> { sid, tabId, sampleRate }
const streams = new Map();
let desktop = null; // { left, width } union of all displays

function browserExe() {
  const brands = navigator.userAgentData?.brands?.map((b) => b.brand) ?? [];
  if (brands.some((b) => b.includes("Edge"))) return "msedge.exe";
  if (brands.some((b) => b.includes("Brave"))) return "brave.exe";
  return "chrome.exe";
}

async function refreshDesktop() {
  const displays = await chrome.system.display.getInfo();
  let left = Infinity;
  let right = -Infinity;
  for (const d of displays) {
    left = Math.min(left, d.bounds.left);
    right = Math.max(right, d.bounds.left + d.bounds.width);
  }
  desktop = right > left ? { left, width: right - left } : null;
}

function normXFromCx(cx) {
  if (!desktop) return 0;
  return Math.max(0, Math.min(1, (cx - desktop.left) / desktop.width)) - 0.5;
}

function buildMeta(entry, normX) {
  return {
    type: "meta",
    v: PROTOCOL_VERSION,
    stream: entry.sid,
    label: entry.label || "tab",
    norm_x: normX,
    sample_rate: entry.sampleRate || 48000,
    channels: 2,
    exe: browserExe(),
  };
}

async function metaFor(entry) {
  const tab = await chrome.tabs.get(entry.tabId);
  const win = await chrome.windows.get(tab.windowId);
  if (!desktop) await refreshDesktop();
  entry.label = (tab.title || "tab").slice(0, 80);
  entry.windowId = tab.windowId;
  let normX = 0;
  if (win.left != null && win.width != null) {
    const cx = win.left + win.width / 2;
    entry.lastCx = cx;
    normX = normXFromCx(cx);
  }
  return buildMeta(entry, normX);
}

// Live drag tracking: onBoundsChanged only fires when the move commits and
// the renderer's window.screenX is not refreshed mid-drag either, but the
// HWND rect does move and Chromium keeps servicing UI-thread tasks during
// the modal move loop, so polling chrome.windows.get sees live bounds.
let posPoll = null;
function ensurePosPoll() {
  if (posPoll) return;
  posPoll = setInterval(async () => {
    if (!streams.size) {
      clearInterval(posPoll);
      posPoll = null;
      return;
    }
    if (!wsReady) return;
    const byWindow = new Map();
    for (const entry of streams.values()) {
      if (entry.windowId == null) continue;
      if (!byWindow.has(entry.windowId)) byWindow.set(entry.windowId, []);
      byWindow.get(entry.windowId).push(entry);
    }
    for (const [winId, list] of byWindow) {
      try {
        const win = await chrome.windows.get(winId);
        if (win.left == null || win.width == null) continue;
        const cx = win.left + win.width / 2;
        for (const entry of list) {
          if (entry.lastCx != null && Math.abs(cx - entry.lastCx) < 8)
            continue;
          entry.lastCx = cx;
          sendMetaFromCx(entry, cx);
        }
      } catch (e) {
        /* window gone; meta path cleans up via tab close */
      }
    }
  }, 200);
}

// Live-drag position report from the content script (window.screenX); no
// chrome.windows round-trip, so it works while the move loop is active.
async function sendMetaFromCx(entry, cx) {
  if (!wsReady) return;
  if (!desktop) await refreshDesktop();
  try {
    ws.send(JSON.stringify(buildMeta(entry, normXFromCx(cx))));
  } catch (e) {}
}

async function sendMeta(entry) {
  if (!wsReady) return;
  try {
    ws.send(JSON.stringify(await metaFor(entry)));
  } catch (e) {
    /* tab/window already gone */
  }
}

async function sendAllMeta() {
  for (const entry of streams.values()) await sendMeta(entry);
}

// Tells every streaming tab whether OBS is reachable; the content scripts
// fall back to normal local playback while it is not.
function broadcastWsState() {
  for (const p of streams.keys()) {
    try {
      p.postMessage({ type: "ws", connected: wsReady });
    } catch (e) {}
  }
}

async function ensureWs() {
  if (ws && (wsReady || ws.readyState === WebSocket.CONNECTING)) return;
  const { port } = await chrome.storage.sync.get({ port: 8796 });
  ws = new WebSocket(`ws://127.0.0.1:${port}/`);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => {
    wsReady = true;
    sendAllMeta();
    broadcastWsState();
  };
  ws.onclose = ws.onerror = () => {
    const was_ready = wsReady;
    wsReady = false;
    ws = null;
    if (was_ready) broadcastWsState();
    if (streams.size && !reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        ensureWs();
      }, 3000);
    }
  };
}

function sendPcm(entry, b64) {
  if (!wsReady) return;
  const bin = atob(b64);
  const buf = new ArrayBuffer(4 + bin.length);
  const view = new DataView(buf);
  view.setUint32(0, entry.sid, true);
  const bytes = new Uint8Array(buf, 4);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  try {
    ws.send(buf);
  } catch (e) {
    /* socket raced shut; reconnect path handles it */
  }
}

chrome.runtime.onConnect.addListener((p) => {
  if (p.name !== "audio" || p.sender?.tab?.id == null) return;
  const entry = { sid: nextStreamId++, tabId: p.sender.tab.id, sampleRate: 48000 };
  streams.set(p, entry);
  ensurePosPoll();
  p.onMessage.addListener((msg) => {
    if (msg.type === "start") {
      entry.sampleRate = msg.sampleRate || 48000;
      try {
        p.postMessage({ type: "ws", connected: wsReady });
      } catch (e) {}
      ensureWs().then(() => sendMeta(entry));
    } else if (msg.type === "pcm") {
      sendPcm(entry, msg.b64);
    } else if (msg.type === "pos") {
      entry.lastCx = msg.cx;
      sendMetaFromCx(entry, msg.cx);
    }
  });
  p.onDisconnect.addListener(() => {
    streams.delete(p);
    if (wsReady) {
      try {
        ws.send(JSON.stringify({ type: "close", stream: entry.sid }));
      } catch (e) {}
    }
  });
});

// Window moved or resized: re-report every stream living in it.
chrome.windows.onBoundsChanged.addListener(async (win) => {
  for (const entry of streams.values()) {
    try {
      const tab = await chrome.tabs.get(entry.tabId);
      if (tab.windowId === win.id) sendMeta(entry);
    } catch (e) {}
  }
});
// Tab dragged into another window.
chrome.tabs.onAttached.addListener((tabId) => {
  for (const entry of streams.values())
    if (entry.tabId === tabId) sendMeta(entry);
});
chrome.system.display.onDisplayChanged.addListener(async () => {
  await refreshDesktop();
  sendAllMeta();
});
