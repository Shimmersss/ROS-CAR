"""Raw 8N1 serial without pyserial (termios); also works on a pseudo-terminal."""
import os
import termios


def open_port(path, baud):
    speed = getattr(termios, f'B{int(baud)}', None)
    if speed is None:
        raise ValueError(f'unsupported baud rate {baud}')
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                                   # iflag: no translation
        attrs[1] = 0                                                   # oflag: raw output
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL        # 8N1, ignore modem lines
        attrs[3] = 0                                                   # lflag: no echo/canonical
        attrs[4] = attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd
