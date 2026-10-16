"""Internal model worker. The parent owns its lifetime and readiness pipe."""
import argparse
import ctypes
import os
import signal
import sys


def protect_parent_lifetime():
    parent = int(os.environ['ROSCAR_ASR_PARENT_PID'])
    if not sys.platform.startswith('linux'):
        raise RuntimeError('Managed ASR requires Linux process lifetime support')
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Cannot bind ASR worker to its parent')
    if os.getppid() != parent:
        raise RuntimeError('ASR parent exited before worker startup')


def main():
    protect_parent_lifetime()  # Before importing numpy, ROS or a native model.
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['offline', 'xfyun'], required=True)
    args, ros_args = parser.parse_known_args()
    import rclpy
    from rclpy.executors import ExternalShutdownException
    if args.backend == 'offline':
        from .asr_node import OfflineAsrNode as NodeType
    else:
        from xfyun_speech.asr_node import XfyunAsrNode as NodeType
    rclpy.init(args=ros_args)
    node = None
    try:
        node = NodeType()
        # Constructors have now loaded the ASR and VAD successfully.
        fd = int(os.environ.pop('ROSCAR_ASR_READY_FD'))
        try:
            os.write(fd, b'1')
        finally:
            os.close(fd)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can raise RCLError when SIGTERM invalidates a wait set.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
