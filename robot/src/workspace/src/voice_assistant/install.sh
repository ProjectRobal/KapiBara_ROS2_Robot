#!/bin/bash


cd rhasspy3

# install VAD
mkdir -p config/programs/vad/
cp -R programs/vad/silero config/programs/vad/
config/programs/vad/silero/script/setup

# install whisper
mkdir -p config/programs/asr/
cp -R programs/asr/faster-whisper config/programs/asr/
config/programs/asr/faster-whisper/script/setup

config/programs/asr/faster-whisper/script/download.py tiny-int8

# install wake word detection

mkdir -p config/programs/wake/
cp -R programs/wake/porcupine1 config/programs/wake/
config/programs/wake/porcupine1/script/setup

# install Piper

mkdir -p config/programs/tts/
cp -R programs/tts/piper config/programs/tts/
config/programs/tts/piper/script/setup.py

config/programs/tts/piper/script/download.py english