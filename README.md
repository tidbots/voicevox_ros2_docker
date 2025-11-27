# voicevox_ros2

## 動作環境
- ホストのOS: Ubuntu 24.04
- Docker
- ROS2: Jazzy すでにインストール済み（rclpy は apt のものを使う）
- Python パッケージ管理: uv
- TTSエンジン:VOICEVOX CORE 0.16.x 系 (Python wheel + engine一式)


## 構成
```
voicevox_ros2_docker/
 ├─ Dockerfile
 ├─ entrypoint.sh
 ├─ voicevox_core-0.16.0-cp310-abi3-manylinux_2_34_x86_64.whl
 ├─ voicevox_engine/
 │   ├─ onnxruntime/
 │   ├─ dict/
 │   └─ models/vvms/
 └─ ros2_voicevox_ws/
     └─ src/
         └─ voicevox_ros2/
             ├─ package.xml
             ├─ setup.py
             ├─ voicevox_ros2/
             │   ├─ __init__.py
             │   └─ tts_node.py   ← 複数話者版
```

## インストール
```
cd ~/docker git clone
```


## 起動
```
cd ~/docker/voicevox_ros2
docker compose up
```
```
docker run --rm -it --device /dev/snd voicevox_ros2:latest
```

## 動作確認
コンテナ内で /tts_text に発話したい文字列を publishする

```
cd ~/docker/voicevox_ros2
docker compose exec voicevox_ros2 bash
```
```

```
コンテナ内で
```
source /opt/ros/jazzy/setup.bash
source /ros2_voicevox_ws/install/setup.bash

ros2 topic pub /tts_text std_msgs/msg/String "data: '[1] おはようございます'" -1
ros2 topic pub /tts_text std_msgs/msg/String "data: '[8] 別の話者でしゃべります'" -1
ros2 topic pub /tts_text std_msgs/msg/String "data: 'デフォルト話者です'" -1
```


