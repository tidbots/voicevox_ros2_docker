# voicevox_ros2

VOICEVOX COREを使用したROS2 Text-to-Speech（TTS）システムです。
複数の話者・スタイルに対応した日本語音声合成をDockerコンテナで提供します。

## 動作環境

- ホストOS: Ubuntu 24.04
- Docker / Docker Compose
- オーディオデバイス（/dev/snd）

### コンテナ内環境

- ROS2: Jazzy
- Python パッケージ管理: uv
- TTSエンジン: VOICEVOX CORE 0.16.x 系

## ディレクトリ構成

```
voicevox_ros2_docker/
├── compose.yaml              # Docker Compose 設定
├── Dockerfile                # コンテナビルド定義
├── entrypoint.sh             # コンテナ起動スクリプト
├── voicevox_core/            # VOICEVOX Core ファイル（要ダウンロード）
├── README.md                 # 日本語ドキュメント
├── README-e.md               # English documentation
├── CLAUDE.md                 # Claude Code 用ガイド
└── ros2_voicevox_ws/         # ROS2 ワークスペース
    └── src/
        └── voicevox_ros2/    # ROS2 パッケージ
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── voicevox_ros2
            ├── voicevox_ros2/
            │   ├── __init__.py
            │   ├── tts_node.py           # TTS ノード実装
            │   ├── tts_client.py         # TTS クライアントクラス
            │   └── tts_client_example.py # サンプルプログラム
            └── test/
                ├── test_copyright.py
                ├── test_flake8.py
                └── test_pep257.py
```

## インストール

```bash
git clone https://github.com/okadahiroyuki/voicevox_ros2_docker.git
cd voicevox_ros2_docker
```

### VOICEVOX Core のダウンロード

ビルド前に VOICEVOX Core ファイルをダウンロードする必要があります。

```bash
# ダウンローダを取得（ネットワーク経由）
curl -fsSL "https://github.com/VOICEVOX/voicevox_core/releases/download/0.16.2/download-linux-x64" -o download-linux-x64
chmod +x download-linux-x64

# VOICEVOX Core をダウンロード（voicevox_core ディレクトリに保存）
./download-linux-x64 --output voicevox_core
```

> **Note:** ダウンローダ（`download-linux-x64`）が既にある場合は、そのまま使用できます。

ダウンロード後、`voicevox_core/` ディレクトリに以下が含まれます：
- `onnxruntime/` - ONNX Runtime ライブラリ
- `dict/open_jtalk_dic_utf_8-1.11/` - OpenJTalk 辞書
- `models/vvms/` - 音声モデルファイル（.vvm）

## ビルド・起動

### Docker Compose を使用（推奨）

```bash
# ビルド
docker compose build

# 起動
docker compose up
```

### 手動でDockerを使用

```bash
# ビルド
docker build -t voicevox_ros2 .

# 起動
docker run --rm -it --device /dev/snd --network host voicevox_ros2
```

## 動作確認

`/tts_text` トピックに文字列を publish すると発話します。

### 発話テスト

#### 方法1: コンテナに接続してから実行

```bash
# 別ターミナルからコンテナに接続
docker compose exec voicevox_ros2 bash

# コンテナ内で実行
ros2 topic pub /tts_text std_msgs/msg/String "data: 'こんにちは'" -1

# style_id を指定して発話
ros2 topic pub /tts_text std_msgs/msg/String "data: '[1] おはようございます'" -1
ros2 topic pub /tts_text std_msgs/msg/String "data: '[8] 別の話者でしゃべります'" -1
```

#### 方法2: ホストから直接実行

```bash
# デフォルト話者
docker compose exec voicevox_ros2 bash -c "source /etc/profile.d/ros2_setup.sh && ros2 topic pub /tts_text std_msgs/msg/String \"data: 'こんにちは'\" -1"

# style_id を指定して発話
docker compose exec voicevox_ros2 bash -c "source /etc/profile.d/ros2_setup.sh && ros2 topic pub /tts_text std_msgs/msg/String \"data: '[8] 別の話者です'\" -1"
```

## ROS2 インターフェース

### トピック

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| `/tts_text` | std_msgs/String | 発話テキスト入力 |

### ノードパラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `engine_dir` | string | /voicevox_engine | VOICEVOX エンジンのパス |
| `vvm_file` | string | 0.vvm | 音声モデルファイル名 |
| `style_id` | int | 0 | デフォルトの話者スタイルID |

### メッセージ書式

- **デフォルト話者**: `"data: 'テキスト'"`
- **話者指定**: `"data: '[style_id] テキスト'"` （例: `"data: '[8] こんにちは'"`)

## Python からの使用

### サンプルプログラムの実行

```bash
# コンテナ内で
ros2 run voicevox_ros2 tts_client_example
```

### 自作ノードからの使用

```python
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)

    def speak(self, text: str, style_id: int = None):
        msg = String()
        if style_id is not None:
            msg.data = f'[{style_id}] {text}'
        else:
            msg.data = text
        self.tts_pub.publish(msg)

# 使用例
node = MyNode()
node.speak('こんにちは')           # デフォルト話者
node.speak('ずんだもんなのだ', 3)  # ずんだもん
node.speak('春日部つむぎだよ', 8)  # 春日部つむぎ
```

### VoicevoxClient クラスの使用

```python
from voicevox_ros2.tts_client import VoicevoxClient

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.tts = VoicevoxClient(self)

    def do_something(self):
        self.tts.speak('処理が完了しました')
        self.tts.speak('ずんだもんなのだ', style_id=3)
```

## 話者・スタイル一覧

VOICEVOX Ver.0.25.0 での style_id 一覧です。

### 四国めたん
| スタイル | ID |
|---------|-----|
| ノーマル | 2 |
| あまあま | 0 |
| ツンツン | 6 |
| セクシー | 4 |
| ささやき | 36 |
| ヒソヒソ | 37 |

### ずんだもん
| スタイル | ID |
|---------|-----|
| ノーマル | 3 |
| あまあま | 1 |
| ツンツン | 7 |
| セクシー | 5 |
| ささやき | 22 |
| ヒソヒソ | 38 |
| ヘロヘロ | 75 |
| なみだめ | 76 |

### その他の話者
| 話者 | スタイル | ID |
|------|---------|-----|
| 春日部つむぎ | ノーマル | 8 |
| 雨晴はう | ノーマル | 10 |
| 波音リツ | ノーマル | 9 |
| 波音リツ | クイーン | 65 |
| 玄野武宏 | ノーマル | 11 |
| 玄野武宏 | 喜び | 39 |
| 玄野武宏 | ツンギレ | 40 |
| 玄野武宏 | 悲しみ | 41 |
| 白上虎太郎 | ふつう | 12 |
| 白上虎太郎 | わーい | 32 |
| 白上虎太郎 | びくびく | 33 |
| 白上虎太郎 | おこ | 34 |
| 白上虎太郎 | びえーん | 35 |
| 青山龍星 | ノーマル | 13 |
| 青山龍星 | 熱血 | 81 |
| 青山龍星 | 不機嫌 | 82 |
| 青山龍星 | 喜び | 83 |
| 青山龍星 | しっとり | 84 |
| 青山龍星 | かなしみ | 85 |
| 青山龍星 | 囁き | 86 |
| 冥鳴ひまり | ノーマル | 14 |
| 九州そら | ノーマル | 16 |
| 九州そら | あまあま | 15 |
| 九州そら | ツンツン | 18 |
| 九州そら | セクシー | 17 |
| 九州そら | ささやき | 19 |
| もち子さん | ノーマル | 20 |
| もち子さん | セクシー／あん子 | 66 |
| もち子さん | 泣き | 77 |
| もち子さん | 怒り | 78 |
| もち子さん | 喜び | 79 |
| もち子さん | のんびり | 80 |
| 剣崎雌雄 | ノーマル | 21 |
| WhiteCUL | ノーマル | 23 |
| WhiteCUL | たのしい | 24 |
| WhiteCUL | かなしい | 25 |
| WhiteCUL | びえーん | 26 |
| 後鬼 | 人間ver. | 27 |
| 後鬼 | ぬいぐるみver. | 28 |
| 後鬼 | 人間（怒り）ver. | 87 |
| 後鬼 | 鬼ver. | 88 |
| No.7 | ノーマル | 29 |
| No.7 | アナウンス | 30 |
| No.7 | 読み聞かせ | 31 |
| ちび式じい | ノーマル | 42 |
| 櫻歌ミコ | ノーマル | 43 |
| 櫻歌ミコ | 第二形態 | 44 |
| 櫻歌ミコ | ロリ | 45 |
| 小夜/SAYO | ノーマル | 46 |
| ナースロボ＿タイプＴ | ノーマル | 47 |
| ナースロボ＿タイプＴ | 楽々 | 48 |
| ナースロボ＿タイプＴ | 恐怖 | 49 |
| ナースロボ＿タイプＴ | 内緒話 | 50 |
| †聖騎士 紅桜† | ノーマル | 51 |
| 雀松朱司 | ノーマル | 52 |
| 麒ヶ島宗麟 | ノーマル | 53 |
| 春歌ナナ | ノーマル | 54 |
| 猫使アル | ノーマル | 55 |
| 猫使アル | おちつき | 56 |
| 猫使アル | うきうき | 57 |
| 猫使アル | つよつよ | 110 |
| 猫使アル | へろへろ | 111 |
| 猫使ビィ | ノーマル | 58 |
| 猫使ビィ | おちつき | 59 |
| 猫使ビィ | 人見知り | 60 |
| 猫使ビィ | つよつよ | 112 |
| 中国うさぎ | ノーマル | 61 |
| 中国うさぎ | おどろき | 62 |
| 中国うさぎ | こわがり | 63 |
| 中国うさぎ | へろへろ | 64 |
| 栗田まろん | ノーマル | 67 |
| あいえるたん | ノーマル | 68 |
| 満別花丸 | ノーマル | 69 |
| 満別花丸 | 元気 | 70 |
| 満別花丸 | ささやき | 71 |
| 満別花丸 | ぶりっ子 | 72 |
| 満別花丸 | ボーイ | 73 |
| 琴詠ニア | ノーマル | 74 |
| Voidoll | ノーマル | 89 |
| ぞん子 | ノーマル | 90 |
| ぞん子 | 低血圧 | 91 |
| ぞん子 | 覚醒 | 92 |
| ぞん子 | 実況風 | 93 |
| 中部つるぎ | ノーマル | 94 |
| 中部つるぎ | 怒り | 95 |
| 中部つるぎ | ヒソヒソ | 96 |
| 中部つるぎ | おどおど | 97 |
| 中部つるぎ | 絶望と敗北 | 98 |
| 離途 | ノーマル | 99 |
| 離途 | シリアス | 101 |
| 黒沢冴白 | ノーマル | 100 |
| ユーレイちゃん | ノーマル | 102 |
| ユーレイちゃん | 甘々 | 103 |
| ユーレイちゃん | 哀しみ | 104 |
| ユーレイちゃん | ささやき | 105 |
| ユーレイちゃん | ツクモちゃん | 106 |
| 東北ずん子 | ノーマル | 107 |
| 東北きりたん | ノーマル | 108 |
| 東北イタコ | ノーマル | 109 |

## ライセンス

VOICEVOXの利用規約に従ってください。
