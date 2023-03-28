# ROS2のログ関係の調査とメモ
[2023-03-28] 作成

## 調査

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

## 実験01
rclpyだけを編集してみた例
`LoggingSeverity`に新しく`HCT2`を追加し、`RcutilsLogger`に`hct2()`を追加した
IntEnumの値はINFOと同じ20に設定しているため、動かすと[INFO] [1234.5678] [node_name]: messageのように表示される

```py
import rclpy

from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import String

LoggingSeverity.HCT2 = 20

def hct2(self, message, **kwargs):
    return self.log(message, LoggingSeverity.HCT2, **kwargs)

RcutilsLogger.hct2 = hct2


class LogTest(Node):

    def __init__(self):
        super().__init__('log_test')
        self.publisher_ = self.create_publisher(String, 'test', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String(data = 'Hello world {}'.format(self.i))
        self.publisher_.publish(msg)
        self.get_logger().hct2('Publishing: "{}"'.format(msg.data))
        # self.get_logger().info('Publishing: "{}"'.format(msg.data))
        self.i += 1


def main():
    rclpy.init()

    node = LogTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```


## 実装
パッケージとして`rclpy`を編集する場合、ファイルの位置は以下の通り

- `info()`が定義されているのは`rclpy/rclpy/impl/rcutils_logger.py`
- `LoggingSeverity`が定義されているのは`rclpy/rclpy/logging.py`
- `_rclpy_logging`が定義されているのは`rclpy/src/rclpy/_rclpy_logging.c`

考え：
- `LoggingSeverity`に新しく`hCT2`を追加
- `hct2()`を作成して呼び出せるようにする(`node.get_logger().hct2()`のような形)

