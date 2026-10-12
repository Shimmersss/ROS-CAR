"""Raw 8N1 serial without pyserial (termios); also works on a pseudo-terminal."""
import os
import select
import termios
import time


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


def write_all(fd, data, timeout_s):
    """Write every byte to a nonblocking fd, waiting for writability up to timeout_s in total.

    A short write would otherwise leave a truncated frame on the wire. Raises TimeoutError when
    the deadline passes with bytes still unsent (the caller closes the port), OSError on failure."""
    view, deadline = memoryview(data), time.monotonic()+timeout_s
    while view:
        try:
            view = view[os.write(fd, view):]
            continue
        except BlockingIOError:
            pass
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError(f'{len(view)} of {len(data)} bytes unsent after {timeout_s:.3f} s')
