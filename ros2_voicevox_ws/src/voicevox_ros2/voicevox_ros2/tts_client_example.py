#!/usr/bin/env python3
"""VOICEVOX TTS クライアントのサンプルプログラム.

/tts_text トピックにテキストを publish して音声合成を行うサンプルです。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TTSClientExample(Node):
    """TTS クライアントのサンプルノード."""

    def __init__(self):
        super().__init__('tts_client_example')
        self.publisher_ = self.create_publisher(String, '/tts_text', 10)
        self.get_logger().info('TTS Client Example started')

    def speak(self, text: str, style_id: int = None):
        """テキストを発話する.

        Args:
            text: 発話するテキスト
            style_id: 話者スタイルID（省略時はデフォルト話者）
        """
        msg = String()
        if style_id is not None:
            msg.data = f'[{style_id}] {text}'
        else:
            msg.data = text

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSClientExample()

    try:
        # サンプル発話
        import time

        # デフォルト話者で発話
        node.speak('こんにちは、VOICEVOXです。')
        time.sleep(3)

        # ずんだもん（style_id=3）で発話
        node.speak('ずんだもんなのだ！', style_id=3)
        time.sleep(3)

        # 四国めたん（style_id=2）で発話
        node.speak('四国めたんです。よろしくお願いします。', style_id=2)
        time.sleep(3)

        # 春日部つむぎ（style_id=8）で発話
        node.speak('春日部つむぎだよ！', style_id=8)

        # メッセージ送信を待つ
        rclpy.spin_once(node, timeout_sec=1.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
