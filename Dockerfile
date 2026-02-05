# ベースイメージ: ROS 2 Jazzy
FROM ros:jazzy-ros-base

# バージョンなどを引数で指定（必要ならここを変える）
ARG VV_CORE_VERSION=0.16.2
ARG VV_CORE_OS_TAG=manylinux_2_34_x86_64

# 必要なライブラリのインストール
# Ubuntu 24.04 (noble) なので libasound2t64 を使う
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv \
    python3-pip \
    curl \
    git \
    ca-certificates \
    libasound2t64 \
    libasound2-dev \
    libportaudio2 \
    libsndfile1 \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# uv のインストール
ADD https://astral.sh/uv/install.sh /uv-installer.sh
RUN sh /uv-installer.sh && rm /uv-installer.sh
# uv が PATH に入るようにしておく
ENV PATH="/root/.local/bin:${PATH}"

# ROS2 ワークスペース
WORKDIR /ros2_voicevox_ws

# ローカルの ROS2 プロジェクトをすべてコピー
# （このディレクトリに Dockerfile と src/voicevox_ros2 などがある想定）
COPY . /ros2_voicevox_ws

# Python 仮想環境を作成して uv 経由でパッケージをインストール
RUN python3 -m venv --system-site-packages .venv
ENV VIRTUAL_ENV=/ros2_voicevox_ws/.venv
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# voicevox_core + 依存パッケージを uv でインストール
RUN uv pip install --upgrade pip && \
    uv pip install \
      sounddevice \
      soundfile \
      numpy && \
    uv pip install \
      "https://github.com/VOICEVOX/voicevox_core/releases/download/${VV_CORE_VERSION}/voicevox_core-${VV_CORE_VERSION}-cp310-abi3-${VV_CORE_OS_TAG}.whl"

# VOICEVOX Core のエンジン用ディレクトリを構築（事前ダウンロード済みファイルを使用）
# /voicevox_engine の中に:
#   - onnxruntime/
#   - dict/open_jtalk_dic_utf_8-1.11
#   - models 内に 0.vvm (など)
COPY voicevox_core /voicevox_engine

# ROS2 パッケージをビルド
RUN . /opt/ros/jazzy/setup.sh && \
    python3 -m colcon build --symlink-install

# 実行時の環境設定
ENV ROS_DISTRO=jazzy
ENV VOICEVOX_ENGINE_DIR=/voicevox_engine

# シェル環境設定（.bashrc と .bash_profile 両方に追加）
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_voicevox_ws/.venv/bin/activate" >> ~/.bashrc && \
    echo "source /ros2_voicevox_ws/install/setup.bash" >> ~/.bashrc && \
    cp ~/.bashrc ~/.bash_profile


# 使い方:
#   docker run --rm -it --device /dev/snd \
#     voicevox_ros2:latest \
#     bash
#
# コンテナ内で:
#   source /opt/ros/jazzy/setup.bash
#   source /ros2_voicevox_ws/install/setup.bash
#   ros2 run voicevox_ros2 tts_node \
#      --ros-args \
#        -p engine_dir:=${VOICEVOX_ENGINE_DIR} \
#        -p vvm_file:=0.vvm \
#        -p style_id:=0
CMD ["bash"]

