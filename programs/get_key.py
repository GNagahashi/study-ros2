import select
import sys
import termios
import tty


def get_key():
    """Get the first character input from the keyboard."""
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    rlist, _, _ = select.select([sys.stdin], [], [])
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    for i in range(10):
        key = get_key()
        print('input: "{}"'.format(key))


if __name__ == '__main__':
    main()