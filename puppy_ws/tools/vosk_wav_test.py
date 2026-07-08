#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import wave
from vosk import Model, KaldiRecognizer

MODEL_PATH = "/app/puppy_ws/models/vosk-model-small-cn-0.22"
WAV_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/usb_mic_test.wav"

def main():
    wf = wave.open(WAV_PATH, "rb")

    if wf.getnchannels() != 1:
        raise RuntimeError(f"音频不是单声道: channels={wf.getnchannels()}")
    if wf.getsampwidth() != 2:
        raise RuntimeError(f"音频不是16bit: sampwidth={wf.getsampwidth()}")
    if wf.getframerate() != 16000:
        raise RuntimeError(f"音频不是16kHz: rate={wf.getframerate()}")

    print(f"MODEL_PATH={MODEL_PATH}")
    print(f"WAV_PATH={WAV_PATH}")
    print(f"rate={wf.getframerate()}, channels={wf.getnchannels()}, sampwidth={wf.getsampwidth()}")

    model = Model(MODEL_PATH)
    rec = KaldiRecognizer(model, wf.getframerate())

    final_text_parts = []

    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break

        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "").strip()
            if text:
                print(f"[partial-final] {text}")
                final_text_parts.append(text)

    final_result = json.loads(rec.FinalResult())
    final_text = final_result.get("text", "").strip()
    if final_text:
        print(f"[final-tail] {final_text}")
        final_text_parts.append(final_text)

    merged = "".join(final_text_parts).strip()
    print()
    print("===== 最终识别结果 =====")
    print(merged if merged else "<空>")

if __name__ == "__main__":
    main()
