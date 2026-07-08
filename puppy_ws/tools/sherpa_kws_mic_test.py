#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sounddevice as sd
import sherpa_onnx

BASE = "/app/puppy_ws/models/sherpa_kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

TOKENS = f"{BASE}/tokens.txt"
ENCODER = f"{BASE}/encoder-epoch-13-avg-2-chunk-16-left-64.onnx"
DECODER = f"{BASE}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
JOINER = f"{BASE}/joiner-epoch-13-avg-2-chunk-16-left-64.onnx"
KEYWORDS_FILE = "/app/puppy_ws/models/sherpa_kws/keywords_tokenized.txt"

SAMPLE_RATE = 16000

def main():
    kws = sherpa_onnx.KeywordSpotter(
        tokens=TOKENS,
        encoder=ENCODER,
        decoder=DECODER,
        joiner=JOINER,
        keywords_file=KEYWORDS_FILE,
        num_threads=1,
        provider="cpu",
        max_active_paths=4,
        num_trailing_blanks=1,
        keywords_score=1.0,
        keywords_threshold=0.35,
    )

    stream = kws.create_stream()

    print("sherpa-onnx 真唤醒开始监听，直接说：小狗")
    print("按 Ctrl+C 退出")

    def callback(indata, frames, time_info, status):
        if status:
            print(status)
        samples = indata[:, 0]
        stream.accept_waveform(SAMPLE_RATE, samples)
        while kws.is_ready(stream):
            kws.decode_stream(stream)

        result = kws.get_result(stream)
        if result:
            print("WAKE DETECTED:", result)

    with sd.InputStream(
        channels=1,
        samplerate=SAMPLE_RATE,
        dtype="float32",
        callback=callback,
        blocksize=1600,
    ):
        while True:
            sd.sleep(1000)

if __name__ == "__main__":
    main()
