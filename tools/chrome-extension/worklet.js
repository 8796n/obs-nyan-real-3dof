// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 8796n <info@8796.jp>
// PCM tap on the audio rendering thread: never misses a quantum no matter
// how busy the page's main thread is. Batches 512 frames (~10.7 ms) of
// interleaved s16 and posts the buffer (transferred, not copied) to the
// content script. Small batches keep the latency down and feed OBS's
// unbuffered playback at a finer granularity than its 21 ms audio tick.
"use strict";

class NyanPcmTap extends AudioWorkletProcessor {
  constructor() {
    super();
    this.frames = 512;
    this.buf = new Int16Array(this.frames * 2);
    this.fill = 0;
  }

  process(inputs) {
    const input = inputs[0];
    if (!input || input.length === 0 || !input[0]) return true;
    const l = input[0];
    const r = input.length > 1 && input[1] ? input[1] : input[0];
    for (let i = 0; i < l.length; i++) {
      let a = (l[i] * 32767) | 0;
      let b = (r[i] * 32767) | 0;
      if (a > 32767) a = 32767;
      else if (a < -32768) a = -32768;
      if (b > 32767) b = 32767;
      else if (b < -32768) b = -32768;
      this.buf[this.fill * 2] = a;
      this.buf[this.fill * 2 + 1] = b;
      if (++this.fill === this.frames) {
        const out = this.buf;
        this.port.postMessage(out.buffer, [out.buffer]);
        this.buf = new Int16Array(this.frames * 2);
        this.fill = 0;
      }
    }
    return true;
  }
}

registerProcessor("nyan-pcm-tap", NyanPcmTap);
