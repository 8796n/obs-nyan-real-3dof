// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// Hooks media elements with Web Audio: createMediaElementSource detaches
// their sound from the OS output entirely, a ScriptProcessor taps the PCM,
// and the samples go to the service worker (which relays them to the
// nyan Real Audio Wall over WebSocket). Locally the tab is silent - the
// only audible path is OBS monitoring, so nothing plays twice.
"use strict";

let port = null;
let ctx = null;
let mixBus = null;
let muteLeg = null; // zero-gain path that keeps the tap pulled by the graph
let localGain = null; // fallback to the speakers while OBS is unreachable
const hooked = new WeakSet();
let silentBuffers = 0;

function setLocalPlayback(enabled) {
  if (!localGain) return;
  // Short ramp avoids clicks when OBS appears/disappears.
  localGain.gain.setTargetAtTime(enabled ? 1 : 0, ctx.currentTime, 0.05);
}

function ensurePort() {
  if (port) return port;
  try {
    port = chrome.runtime.connect({ name: "audio" });
  } catch (e) {
    return null; // extension reloaded; next call retries
  }
  port.onMessage.addListener((msg) => {
    // Connection state of the OBS-side WebSocket: while it is down the
    // audio falls back to normal local playback instead of going silent.
    if (msg.type === "ws") setLocalPlayback(!msg.connected);
  });
  port.onDisconnect.addListener(() => {
    port = null; // service worker restarted; reconnect lazily
    setLocalPlayback(true);
  });
  if (ctx)
    port.postMessage({ type: "start", sampleRate: ctx.sampleRate });
  return port;
}

function toBase64(bytes) {
  let s = "";
  const CHUNK = 8192;
  for (let i = 0; i < bytes.length; i += CHUNK)
    s += String.fromCharCode.apply(null, bytes.subarray(i, i + CHUNK));
  return btoa(s);
}

// Interleaved s16 PCM out toward the service worker, with a silence
// throttle that still trickles enough to keep the OBS side's 5 s stream
// timeout from firing on a paused tab.
function sendPcmBytes(bytes) {
  const pcm = new Int16Array(bytes.buffer, bytes.byteOffset,
                             bytes.byteLength >> 1);
  let peak = 0;
  for (let i = 0; i < pcm.length; i += 32) {
    const a = pcm[i] < 0 ? -pcm[i] : pcm[i];
    if (a > peak) peak = a;
  }
  if (peak < 3) {
    if (++silentBuffers > 20 && silentBuffers % 40 !== 0) return;
  } else {
    silentBuffers = 0;
  }
  const p = ensurePort();
  if (!p) return;
  try {
    p.postMessage({ type: "pcm", b64: toBase64(bytes) });
  } catch (e) {
    port = null;
  }
}

// Fallback capture on the page's main thread; only used when the page's
// CSP blocks loading the worklet module. 2048 frames ~= 43 ms: smaller
// buffers miss deadlines on busy pages and the capture itself glitches.
function attachScriptProcessor() {
  const sp = ctx.createScriptProcessor(2048, 2, 2);
  sp.onaudioprocess = (ev) => {
    const inBuf = ev.inputBuffer;
    const frames = inBuf.length;
    const l = inBuf.getChannelData(0);
    const r = inBuf.numberOfChannels > 1 ? inBuf.getChannelData(1) : l;
    const pcm = new Int16Array(frames * 2);
    for (let i = 0; i < frames; i++) {
      pcm[i * 2] = Math.max(-32768, Math.min(32767, (l[i] * 32767) | 0));
      pcm[i * 2 + 1] = Math.max(-32768, Math.min(32767, (r[i] * 32767) | 0));
    }
    sendPcmBytes(new Uint8Array(pcm.buffer));
  };
  mixBus.connect(sp);
  sp.connect(muteLeg);
}

function ensureAudio() {
  if (ctx) return;
  ctx = new AudioContext();
  mixBus = ctx.createGain();
  // Taps only run while the graph pulls them toward the destination; a
  // zero-gain leg keeps them pulled without making the tab audible.
  muteLeg = ctx.createGain();
  muteLeg.gain.value = 0;
  muteLeg.connect(ctx.destination);
  // Local fallback leg: audible until the service worker reports a live
  // OBS connection, and again whenever it drops.
  localGain = ctx.createGain();
  localGain.gain.value = 1;
  mixBus.connect(localGain);
  localGain.connect(ctx.destination);

  // Preferred capture: AudioWorklet on the audio rendering thread - no
  // main-thread jank, so no capture dropouts on busy pages.
  ctx.audioWorklet
    .addModule(chrome.runtime.getURL("worklet.js"))
    .then(() => {
      const node = new AudioWorkletNode(ctx, "nyan-pcm-tap", {
        numberOfInputs: 1,
        numberOfOutputs: 1,
        outputChannelCount: [1],
        channelCount: 2,
        channelCountMode: "explicit",
      });
      node.port.onmessage = (ev) => sendPcmBytes(new Uint8Array(ev.data));
      mixBus.connect(node);
      node.connect(muteLeg);
    })
    .catch(() => attachScriptProcessor());

  const p = ensurePort();
  if (p) p.postMessage({ type: "start", sampleRate: ctx.sampleRate });
}

function hookElement(el) {
  if (!(el instanceof HTMLMediaElement) || hooked.has(el)) return;
  ensureAudio();
  if (ctx.state === "suspended") ctx.resume();
  try {
    // Detaches the element from the OS output; from here its audio only
    // exists inside this graph. Throws if the page already routed the
    // element through its own Web Audio graph - leave those alone.
    const src = ctx.createMediaElementSource(el);
    src.connect(mixBus);
    hooked.add(el);
  } catch (e) {
    hooked.add(el); // don't retry every play event
  }
}

document.addEventListener("play", (ev) => hookElement(ev.target), true);

// Media that was already playing when the script landed (document_start
// usually precedes it, but SPA navigations can reuse elements).
function scan() {
  for (const el of document.querySelectorAll("audio,video"))
    if (!el.paused) hookElement(el);
}
if (document.readyState === "loading")
  document.addEventListener("DOMContentLoaded", scan);
else
  scan();
