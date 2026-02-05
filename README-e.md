# voicevox_ros2

A ROS2 Text-to-Speech (TTS) system using VOICEVOX CORE.
Provides Japanese speech synthesis with multiple speakers and styles in a Docker container.

## Requirements

- Host OS: Ubuntu 24.04
- Docker / Docker Compose
- Audio device (/dev/snd)

### Container Environment

- ROS2: Jazzy
- Python package manager: uv
- TTS Engine: VOICEVOX CORE 0.16.x

## Directory Structure

```
voicevox_ros2_docker/
├── compose.yaml              # Docker Compose configuration
├── Dockerfile                # Container build definition
├── entrypoint.sh             # Container startup script
├── voicevox_core/            # VOICEVOX Core files (download required)
├── README.md                 # Japanese documentation
├── README-e.md               # English documentation
├── CLAUDE.md                 # Claude Code guide
└── ros2_voicevox_ws/         # ROS2 workspace
    └── src/
        └── voicevox_ros2/    # ROS2 package
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── voicevox_ros2
            ├── voicevox_ros2/
            │   ├── __init__.py
            │   └── tts_node.py       # TTS node implementation
            └── test/
                ├── test_copyright.py
                ├── test_flake8.py
                └── test_pep257.py
```

## Installation

```bash
git clone https://github.com/okadahiroyuki/voicevox_ros2_docker.git
cd voicevox_ros2_docker
```

### Download VOICEVOX Core

Before building, you need to download VOICEVOX Core files.

```bash
# Get the downloader (via network)
curl -fsSL "https://github.com/VOICEVOX/voicevox_core/releases/download/0.16.2/download-linux-x64" -o download-linux-x64
chmod +x download-linux-x64

# Download VOICEVOX Core (saves to voicevox_core directory)
./download-linux-x64 --output voicevox_core
```

> **Note:** If you already have the downloader (`download-linux-x64`), you can use it directly.

After download, the `voicevox_core/` directory contains:
- `onnxruntime/` - ONNX Runtime library
- `dict/open_jtalk_dic_utf_8-1.11/` - OpenJTalk dictionary
- `models/vvms/` - Voice model files (.vvm)

## Build & Run

### Using Docker Compose (Recommended)

```bash
# Build
docker compose build

# Run
docker compose up
```

### Using Docker Manually

```bash
# Build
docker build -t voicevox_ros2 .

# Run
docker run --rm -it --device /dev/snd --network host voicevox_ros2
```

## Usage

Publish text to the `/tts_text` topic to synthesize speech.

### Speech Test

#### Method 1: Connect to Container First

```bash
# Connect to the container from another terminal
docker compose exec voicevox_ros2 bash

# Run inside the container
ros2 topic pub /tts_text std_msgs/msg/String "data: 'こんにちは'" -1

# Speak with specific style_id
ros2 topic pub /tts_text std_msgs/msg/String "data: '[1] おはようございます'" -1
ros2 topic pub /tts_text std_msgs/msg/String "data: '[8] 別の話者でしゃべります'" -1
```

#### Method 2: Run Directly from Host

```bash
# Default speaker
docker compose exec voicevox_ros2 bash -c "source /etc/profile.d/ros2_setup.sh && ros2 topic pub /tts_text std_msgs/msg/String \"data: 'こんにちは'\" -1"

# Speak with specific style_id
docker compose exec voicevox_ros2 bash -c "source /etc/profile.d/ros2_setup.sh && ros2 topic pub /tts_text std_msgs/msg/String \"data: '[8] 別の話者です'\" -1"
```

## ROS2 Interface

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/tts_text` | std_msgs/String | Text input for speech synthesis |

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `engine_dir` | string | /voicevox_engine | Path to VOICEVOX engine |
| `vvm_file` | string | 0.vvm | Voice model filename |
| `style_id` | int | 0 | Default speaker style ID |

### Message Format

- **Default speaker**: `"data: 'text'"`
- **Specify speaker**: `"data: '[style_id] text'"` (e.g., `"data: '[8] hello'"`)

## Speaker & Style List

Available style_id values for VOICEVOX Ver.0.25.0.

### Shikoku Metan (四国めたん)
| Style | ID |
|-------|-----|
| Normal | 2 |
| Sweet | 0 |
| Tsun-tsun | 6 |
| Sexy | 4 |
| Whisper | 36 |
| Hushed | 37 |

### Zundamon (ずんだもん)
| Style | ID |
|-------|-----|
| Normal | 3 |
| Sweet | 1 |
| Tsun-tsun | 7 |
| Sexy | 5 |
| Whisper | 22 |
| Hushed | 38 |
| Exhausted | 75 |
| Tearful | 76 |

### Other Speakers
| Speaker | Style | ID |
|---------|-------|-----|
| Kasugabe Tsumugi | Normal | 8 |
| Amehare Hau | Normal | 10 |
| Namine Ritsu | Normal | 9 |
| Namine Ritsu | Queen | 65 |
| Geno Takehiro | Normal | 11 |
| Geno Takehiro | Joy | 39 |
| Geno Takehiro | Angry | 40 |
| Geno Takehiro | Sad | 41 |
| Shirakami Kotarou | Normal | 12 |
| Shirakami Kotarou | Happy | 32 |
| Shirakami Kotarou | Nervous | 33 |
| Shirakami Kotarou | Angry | 34 |
| Shirakami Kotarou | Crying | 35 |
| Aoyama Ryusei | Normal | 13 |
| Aoyama Ryusei | Passionate | 81 |
| Aoyama Ryusei | Grumpy | 82 |
| Aoyama Ryusei | Joy | 83 |
| Aoyama Ryusei | Gentle | 84 |
| Aoyama Ryusei | Sad | 85 |
| Aoyama Ryusei | Whisper | 86 |
| Meinari Himari | Normal | 14 |
| Kyushu Sora | Normal | 16 |
| Kyushu Sora | Sweet | 15 |
| Kyushu Sora | Tsun-tsun | 18 |
| Kyushu Sora | Sexy | 17 |
| Kyushu Sora | Whisper | 19 |
| Mochiko-san | Normal | 20 |
| Mochiko-san | Sexy/Anko | 66 |
| Mochiko-san | Crying | 77 |
| Mochiko-san | Angry | 78 |
| Mochiko-san | Happy | 79 |
| Mochiko-san | Relaxed | 80 |
| Kenzaki Mesuo | Normal | 21 |
| WhiteCUL | Normal | 23 |
| WhiteCUL | Happy | 24 |
| WhiteCUL | Sad | 25 |
| WhiteCUL | Crying | 26 |
| Goki | Human ver. | 27 |
| Goki | Plush ver. | 28 |
| Goki | Human (Angry) ver. | 87 |
| Goki | Demon ver. | 88 |
| No.7 | Normal | 29 |
| No.7 | Announce | 30 |
| No.7 | Storytelling | 31 |
| Chibisiki Jii | Normal | 42 |
| Ouka Miko | Normal | 43 |
| Ouka Miko | Second Form | 44 |
| Ouka Miko | Loli | 45 |
| Sayo/SAYO | Normal | 46 |
| Nurse Robot Type-T | Normal | 47 |
| Nurse Robot Type-T | Relaxed | 48 |
| Nurse Robot Type-T | Fear | 49 |
| Nurse Robot Type-T | Secret | 50 |
| Holy Knight Benisakura | Normal | 51 |
| Suzumematsu Shuji | Normal | 52 |
| Kigashima Sourin | Normal | 53 |
| Haruka Nana | Normal | 54 |
| Nekotsuka Aru | Normal | 55 |
| Nekotsuka Aru | Calm | 56 |
| Nekotsuka Aru | Excited | 57 |
| Nekotsuka Aru | Strong | 110 |
| Nekotsuka Aru | Exhausted | 111 |
| Nekotsuka Bi | Normal | 58 |
| Nekotsuka Bi | Calm | 59 |
| Nekotsuka Bi | Shy | 60 |
| Nekotsuka Bi | Strong | 112 |
| Chugoku Usagi | Normal | 61 |
| Chugoku Usagi | Surprised | 62 |
| Chugoku Usagi | Scared | 63 |
| Chugoku Usagi | Exhausted | 64 |
| Kurita Maron | Normal | 67 |
| Aiel-tan | Normal | 68 |
| Manbetsu Hanamaru | Normal | 69 |
| Manbetsu Hanamaru | Energetic | 70 |
| Manbetsu Hanamaru | Whisper | 71 |
| Manbetsu Hanamaru | Cutesy | 72 |
| Manbetsu Hanamaru | Boy | 73 |
| Kotoyomi Nia | Normal | 74 |
| Voidoll | Normal | 89 |
| Zonko | Normal | 90 |
| Zonko | Low Energy | 91 |
| Zonko | Awakened | 92 |
| Zonko | Commentary | 93 |
| Chubu Tsurugi | Normal | 94 |
| Chubu Tsurugi | Angry | 95 |
| Chubu Tsurugi | Hushed | 96 |
| Chubu Tsurugi | Nervous | 97 |
| Chubu Tsurugi | Despair | 98 |
| Rito | Normal | 99 |
| Rito | Serious | 101 |
| Kurosawa Saehaku | Normal | 100 |
| Yurei-chan | Normal | 102 |
| Yurei-chan | Sweet | 103 |
| Yurei-chan | Sad | 104 |
| Yurei-chan | Whisper | 105 |
| Yurei-chan | Tsukumo-chan | 106 |
| Tohoku Zunko | Normal | 107 |
| Tohoku Kiritan | Normal | 108 |
| Tohoku Itako | Normal | 109 |

## License

Please follow VOICEVOX's terms of use.
