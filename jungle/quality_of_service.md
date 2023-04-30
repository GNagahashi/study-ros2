# ROS2のQoSに関する調査とメモ
[2023-04-30] 作成

ROS2 humbleの公式ドキュメントより


## QoSとは
Quality of Serviceの略
ノード間の通信を調節するための仕組み
TCPライクな通信品質から、UDPライクな通信品質まで様々に調節することができる

QoSの設定はプロファイル(QoSプロファイル)と呼び、プロファイルはポリシー(QoSポリシー)で構成される
つまり、複数のポリシーを組み合わせてプロファイルを作成する

よくあるユースケース(例：センサデータを扱いたいなど)に対して定義済みのプロファイルを提供している

QoSはノードごと(subやpubのそれぞれごと)で独立に設定することが可能だが、互換性がないプロファイルを使用した場合は通信が成立しない可能性がある



## QoSポリシー
- `History`(履歴)
    保持するメッセージの個数を表すポリシー
    - `Keep last`: 最大N個のメッセージを保持する
    - `Keep all`: 可能な限り全てのメッセージを保持する
- `Depth`(深さ)
    保持するメッセージの個数を表すポリシー
    HistoryがKeep lastに設定されている場合のみ有効となる
    - `Queue size`: メッセージを保持するキューのサイズ
- `Reliability`(信頼性)
    通信の確かさを表すポリシー
    - `Best effort`: ネットワークが不安定な場合はメッセージを失う可能性がある
    - `Reliable`: 必要に応じてメッセージを複数回送信し、メッセージを確実に届ける
- `Durability`(耐久性)
    後からネットワークに参加してきたノードに対して以前(直前)のメッセージを送信するか否かを表すポリシー
    - `Transient local`: 遅れてきたノードに対してメッセージを送信する
    - `Volatile`: 遅れてきたノードに対してメッセージを送信しない
- `Deadline`(締切)
    後続のメッセージが公開されるまでの時間を表すポリシー
    - `Duration`: 後続のメッセージが公開されるまでの時間
- `Lifespan`(生存期間)
    メッセージの生存期間を表すポリシー
    期間を過ぎたメッセージは削除される(受信されない)
    - `Duration`: メッセージが生存可能な期間
- `Liveliness`(生存状態)
    ノード(送信側)の生存状態を表すポリシー
    - `Automatic`: システムがメッセージの送信を確認した場合、メッセージを送信したノードは生存していると判断する
    - `Manual by topic`: publisher側のAPIを利用し手動にて生存を表現した場合、そのノードが生存していると判断する
- `Lease Duration`(リース期間)
    ノード(送信側)の生存期間を表すポリシー
    - `Duration`: ノードが生存可能な期間


## 定義済みのQoSプロファイル
https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h

- `Default QoS settings for publishers and subscriptions`
- `Services`
- `Sensor data`
- `Parameters`
- `System default`


## 参考文献
- [ROS2 humbleの公式ドキュメント - About Quality of Service settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)