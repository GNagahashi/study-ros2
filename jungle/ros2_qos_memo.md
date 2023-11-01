目標：QoSの情報が欲しい(あるトピックに対してpublishしているpublisherのQoSプロファイルが欲しい)

rclpy/rclpy/node.pyに定義されている`Node`クラスのメソッドとして`get_publishers_info_by_topic`というメソッドがある  
このメソッドを呼び出すと`ros2 topic info /topic_name -v`とほぼ同じ情報が手に入る  
= 指定したトピックにpublishしているpublisherのQoSプロファイルが手に入る

https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L1893

上記の`get_publishers_info_by_topic`メソッドは`_get_info_by_topic`メソッドを呼び出しており、`_get_info_by_topic`メソッドではQoSの情報(とその他諸々)を取得するために`_rclpy.rclpy_get_publishers_info_by_topic`関数を呼び出している

`_rclpy`はimportしたライブラリ名であり、これは`from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy`と記述されている

https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L61

`_rclpy`は`rclpy.impl.implementation_singleton`にある`rclpy_implementation`をimportしているわけだが、具体的に`rclpy_implementation`はこのように記述されている

https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/impl/implementation_singleton.py#L32

`rclpy_implementation = import_c_library('._rclpy_pybind11', package)`は`package = rclpy`にある`._rclpy_pybind11`というCライブラリを読み込む(`rclpy_implementation`という名前で読み込む)という意味であると思われる

`._rclpy_pybind11`は`rclpy/rclpy/src/rclpy/_rclpy_pybind11.cpp`のことであり、いま求めている関数`rclpy_get_publishers_info_by_topic`はここで定義されている

https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/_rclpy_pybind11.cpp#L183

`graph_get_publishers_info_by_topic`という関数を`rclpy_get_publishers_info_by_topic`という名前で利用できるようにしていると思われる

`graph_get_publishers_info_by_topic`は実際には`graph.hpp`(`graph.cpp`)に定義されている

`graph.cpp`は`rclpy/rclpy/src/rclpy/graph.cpp`にある

https://github.com/ros2/rclpy/blob/fe8f4a19f0daf264e3f2becc1c5de0959c1560f5/rclpy/src/rclpy/graph.cpp#L263

`graph_get_publishers_info_by_topic`関数は`_get_info_by_topic`関数を呼び出す形になっているが、結局実行している関数は`rcl_get_publishers_info_by_topic`である

`graph.cpp`のincludeを見ると`#include <rcl/graph.h>`があるのがわかる  
つまり、求めている関数`rcl_get_publishers_info_by_topic`は`rcl/rcl/src/rcl/graph.c`に定義されている

https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/graph.c#L683

`rcl_get_publishers_info_by_topic`関数は`__rcl_get_info_by_topic`関数を呼び出している

`__rcl_get_info_by_topic`関数の中を見ると、ifを使って条件判別をしながらQoSの情報(正確にはトピックに対してpublishしているpublisherの情報)を取得しているのがわかる

https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/graph.c#L640

---

まとめると、`rclpy`の`Node`クラスのメソッドとしてある`get_publishers_info_by_topic`は同ライブラリ(`rclpy`)内にあるC++の関数を呼び出している

`rclpy`は`rcl`の関数を呼び出せるように上記のC++のソースコードを含んでおり、`rclpy`が参照している`rcl`の関数、つまり`get_publishers_info_by_topic`メソッドが呼び出している`rcl`の関数は......

`rcl/rcl/src/rcl/graph.c`に定義されている`rcl_get_publishers_info_by_topic`関数ということになる

(実際にはまだ先があるが、恐らくここがちょうどいい場所)

---

次の目標：`rcl/rcl/src/rcl/graph.c`に定義されている`rcl_get_publishers_info_by_topic`関数を呼び出すようなpythonパッケージを作成してみる

