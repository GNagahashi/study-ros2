# ROS2のログ関係の調査とメモ
[2023-03-28] 作成

参考資料

- [Logging](https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html)
- [About logging and logger configuration](https://docs.ros.org/en/foxy/Concepts/About-Logging.html)
- [rcl](https://github.com/ros2/rcl)
- [rcl_logging](https://github.com/ros2/rcl_logging)
- [log4cxx](https://github.com/apache/logging-log4cxx)

## 調査

`rcutils`パッケージと`rcl_logging`パッケージの上に`rcl`パッケージがあり、更にその上に`rclpy`や`rclcpp`がある模様
INFOやWARNといったSeverity levelはROS2のログシステムで使われている(?)`spdlog`ライブラリに依存しているものである可能性がある(要調査)
この場合、現状やろうとしていることはかなり厳しいことになる(spdlogに手を加えることは避けたい)
この仮定が正しい場合、逆に、spdlogに何か丁度良さそうなステータス(INFOみたいなやつ)があればそれを利用しても良い


### 重大度レベル(Severity level)とは
ROS2が出力するログには重大度が設定されており、以下の5種類がある

- DEBUG
- INFO
- WARN
- ERROR
- FATAL

ROS2には重大度を設定することが可能であり、設定した重大度を基準にログの表示/非表示を管理できる
例：重大度レベルとしてINFO以上を指定すると、INFO未満であるDEBUGは出力されなくなる(表示されないだけでログファイルには保存されているかも？)

重大度の値はrcutilsで定義されているものと同じでなければ行けない(`LoggingSeverity`のコメントに書いてある)



### rclpy
`Node.get_logger().info()`の`info()`は`/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/rcutils_logger.py`に定義されている(vscodeで参照)

ログのレベル(INFOとかDEBUGとか)は`/opt/ros/foxy/lib/python3.8/site-packages/rclpy/logging.py`に定義されている(vscodeで参照)  
`LoggingSeverity`という`IntEnum`として実装されている  
ここを編集すればINFOの他にhCT2みたいなやつが作れるかも？

`Node.get_logger().info()`の`get_logger()`は`/opt/ros/foxy/lib/python3.8/site-packages/rclpy/node.py`に定義されている(vscodeで参照)


`LoggingSeverity`ではINFOを以下のように定義している
```py
INFO = _rclpy_logging.rclpy_get_info_logging_severity()
```
`_rclpy_logging`は`from rclpy.impl.implementation_singleton import rclpy_logging_implementation as _rclpy_logging`でimportしている  
ファイルは`/opt/ros/foxy/lib/python3.8/site-packages/rclpy/impl/implementation_singleton.py`

`rclpy_logging_implementation`は以下のように定義している
```py
rclpy_logging_implementation = _import('._rclpy_logging')
```

`_rclpy_logging`は`rclpy/src/rclpy/_rclpy_logging.c`が本体である模様  
`_rclpy_logging.c`には`rclpy_get_unset_logging_severity`が定義されている

つまり、`LoggingSeverity`の
```py
INFO = _rclpy_logging.rclpy_get_info_logging_severity()
```
は`rclpy/src/rclpy/_rclpy_logging.c`で定義している`rclpy_get_info_logging_severity()`を呼び出している模様

`info()`が呼び出している`log()`の**kwargsとして`name`を設定するとノード名が変更できる模様
```py
self.get_logger().info('Publishing: "{}"'.format(msg.data), name = 'node_name')
```

<!-- @todo rclpyのAPIドキュメントを探してみる rclcppではログの設定が自由に変えられなかったっけ？APIを確認する -->

### rcutils
[https://github.com/ros2/rcutils](https://github.com/ros2/rcutils)

ROS2のログシステムのAPIは`rcutils`パッケージが担当している模様  
恐らく、rclcppもこのパッケージをincludeして使っている？(要確認)  
つまり、この`rcutils`を編集することで独自のログレベル(LoggingSeverity)を追加することができる(さらにrclpyの実装も必要だが)

重大度のレベルが設定されているのはここ
- [https://github.com/ros2/rcutils/blob/rolling/include/rcutils/logging.h#L171](https://github.com/ros2/rcutils/blob/rolling/include/rcutils/logging.h#L171)


## 実験01
rclpyだけを編集してみた例
`LoggingSeverity`に新しく`HCT2`を追加し、`RcutilsLogger`に`hct2()`を追加した
IntEnumの値はINFOと同じ20に設定しているため、動かすと[INFO] [1234.5678] [node_name]: messageのように表示される

参考資料

- [console.log](https://gist.github.com/DYGV/fccfdf481e05a77f5cc7407dcaba31ff)

```py
import rclpy

from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import String

LoggingSeverity.NEW = 20  # LoggingSeverityはIntEnum, 20はINFOと同じ値

def new(self, message, **kwargs):
    return self.log(message, LoggingSeverity.NEW, **kwargs)

RcutilsLogger.new = new  # これがinfo()の代わり


class LogTest(Node):

    def __init__(self):
        super().__init__('log_test')
        self.publisher_ = self.create_publisher(String, 'test', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String(data = 'Hello world {}'.format(self.i))
        self.publisher_.publish(msg)
        self.get_logger().new('Publishing: "{}"'.format(msg.data))
        # self.get_logger().info('Publishing: "{}"'.format(msg.data))
        self.i += 1


def main():
    rclpy.init()

    node = LogTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```


## 実装 テスト1
パッケージとして`rclpy`を編集する場合、ファイルの位置は以下の通り

- `info()`が定義されているのは`rclpy/rclpy/impl/rcutils_logger.py`
- `LoggingSeverity`が定義されているのは`rclpy/rclpy/logging.py`
- `_rclpy_logging`が定義されているのは`rclpy/src/rclpy/_rclpy_logging.c`

考え：
- `LoggingSeverity`に新しくレベルを追加
- `node.get_logger().new()`のような形で呼び出せるようにする

### 編集箇所

- `rcutils/rcutils/logging.py`
