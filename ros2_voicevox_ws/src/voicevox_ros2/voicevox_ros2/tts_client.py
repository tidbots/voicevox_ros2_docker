#!/usr/bin/env python3
"""VOICEVOX TTS クライアントクラス.

他のノードから簡単に音声合成を呼び出すためのユーティリティクラスです。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoicevoxClient:
    """VOICEVOX TTS クライアント.

    使用例:
        from voicevox_ros2.tts_client import VoicevoxClient

        # ノード内で使用
        class MyNode(Node):
            def __init__(self):
                super().__init__('my_node')
                self.tts = VoicevoxClient(self)

            def do_something(self):
                self.tts.speak('処理が完了しました')
                self.tts.speak('ずんだもんなのだ', style_id=3)
    """

    def __init__(self, node: Node, topic: str = '/tts_text'):
        """初期化.

        Args:
            node: ROS2ノード
            topic: TTSトピック名
        """
        self._node = node
        self._publisher = node.create_publisher(String, topic, 10)

    def speak(self, text: str, style_id: int = None):
        """テキストを発話する.

        Args:
            text: 発話するテキスト
            style_id: 話者スタイルID（省略時はデフォルト話者）

        主な話者スタイルID:
            0: 四国めたん（あまあま）
            2: 四国めたん（ノーマル）
            3: ずんだもん（ノーマル）
            8: 春日部つむぎ（ノーマル）
        """
        msg = String()
        if style_id is not None:
            msg.data = f'[{style_id}] {text}'
        else:
            msg.data = text

        self._publisher.publish(msg)
        self._node.get_logger().debug(f'TTS: {msg.data}')
