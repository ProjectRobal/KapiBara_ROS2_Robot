#!/bin/bash

install_path=$(echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | grep "voice_assistant")

cd -P "$install_path"/lib/voice_assistant/rhasspy3

# script/run bin/mic_record_sample.py sample.wav

# script/run bin/mic_test_energy.py

# script/run bin/pipeline_run.py --debug