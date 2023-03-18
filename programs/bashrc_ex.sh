# extra bash setting

# change prompt
export PS1='\[\e]0;\h@\u\a\]\n\[\e[38;2;0;255;162m\]\u: \w\n\$ \[\e[0m\]'

# disable(overwriting): alias ls='ls --color=auto'
alias ls='ls'


# lxterminalの設定ファイルは`~/.config/lxterminal/`にある(ubuntuで後から`sudo apt install lxterminal`をした場合)

# 設定ファイルの読み込みは`/etc/profile`→`/etc/bash.bashrc`→`~/.bashrc`の模様？

# このリポジトリにある`bashrc_ex.sh`の中身を`~/.bashrc`に書き込むとプロンプトが変更され`ls`コマンドの色付けが取れる