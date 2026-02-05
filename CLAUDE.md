# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2-based Text-to-Speech system integrating VOICEVOX CORE as the TTS engine. Provides a containerized ROS2 node that subscribes to text messages and synthesizes Japanese speech with multiple speaker/voice styles.

## Build Commands

### Docker Build
```bash
docker compose build
```

### Run the Application
```bash
docker compose up
```

### Manual Container Build
```bash
docker build -t voicevox_ros2 .
docker run --rm -it --device /dev/snd --network host voicevox_ros2
```

### Inside Container - Build ROS2 Package
```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_voicevox_ws
python3 -m colcon build --symlink-install
```

### Run Tests
```bash
colcon test
colcon test-result --verbose
```

## Architecture

### Container Stack
- **Base:** ros:jazzy-ros-base (Ubuntu 24.04)
- **TTS Engine:** VOICEVOX Core (voicevox_core Python package)
- **Package Manager:** uv (for fast Python dependency management)
- **Audio:** sounddevice + soundfile + numpy

### ROS2 Interface
- **Node Name:** `voicevox_tts_node`
- **Subscription:** `/tts_text` (std_msgs/String)
- **Message Format:**
  - Default style: `"text to speak"`
  - With style_id: `"[8] text to speak"` (speaks with style_id 8)

### Key Files
- `Dockerfile` - Multi-stage build downloading VOICEVOX Core from GitHub releases
- `compose.yaml` - Container configuration with audio device passthrough and host networking
- `entrypoint.sh` - Sets up ROS2 and venv environments
- `ros2_voicevox_ws/src/voicevox_ros2/voicevox_ros2/tts_node.py` - Main TTS node implementation

### Node Parameters
- `engine_dir` (string, default: "/voicevox_engine") - VOICEVOX engine path
- `vvm_file` (string, default: "0.vvm") - Voice model filename
- `style_id` (integer, default: 0) - Default voice style ID

## Key Patterns

1. **Dynamic File Discovery:** Node recursively searches engine_dir for ONNX Runtime lib, OpenJTalk dict, and VVM files rather than hardcoding paths.

2. **Style ID Parsing:** Uses regex `\[(\d+)\]` prefix to parse speaker ID inline from message content.

3. **System Site-Packages:** Python venv uses `--system-site-packages` to access ROS2's system-installed rclpy.

4. **Pre-downloaded VOICEVOX Core:** The `voicevox_core/` directory must be downloaded before building. Use the VOICEVOX downloader to obtain required files.

## Publishing Text for TTS

**Method 1: Connect to container first**
```bash
docker compose exec voicevox_ros2 bash
ros2 topic pub /tts_text std_msgs/msg/String "data: 'こんにちは'" -1
ros2 topic pub /tts_text std_msgs/msg/String "data: '[8] 別の話者です'" -1
```

**Method 2: Run directly from host**
```bash
docker compose exec voicevox_ros2 bash -c "source /etc/profile.d/ros2_setup.sh && ros2 topic pub /tts_text std_msgs/msg/String \"data: 'こんにちは'\" -1"
```
