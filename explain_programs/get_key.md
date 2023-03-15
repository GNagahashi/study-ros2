# get_key()

標準入力に入力された最初の一文字を読み取るプログラム
一文字目以降は無視される

## 解説

```py
settings = termios.tcgetattr(sys.stdin)
```

`termios.tcgetattr(fd?)`で
<!-- @note tcgetattrで取得した値を覗いてみる -->
