"""Keep ROS alive during SIGINT/SIGTERM cleanup of owned goals and leases."""
import signal
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run(factory,args=None,threads=6):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    stopped=threading.Event()
    previous={sig:signal.signal(sig,lambda *_:stopped.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
    node=None;executor=None;worker=None
    try:
        node=factory();executor=MultiThreadedExecutor(num_threads=threads);executor.add_node(node)
        def spin():
            try:
                executor.spin()
            finally:
                stopped.set()
        worker=threading.Thread(target=spin,name='capture_ros_spin',daemon=True);worker.start()
        while not stopped.wait(.1):
            pass
    finally:
        if node and hasattr(node,'stop'):
            node.stop()
            # Capture.execute finishes its own release before the executor stops.
            import time
            end=time.monotonic()+4.
            while getattr(node,'job',None) is not None and time.monotonic()<end:
                time.sleep(.02)
        if executor:
            executor.shutdown(timeout_sec=5.)
        if worker:
            worker.join(5.)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig,handler in previous.items():
            signal.signal(sig,handler)
