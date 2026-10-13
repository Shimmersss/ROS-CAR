"""Real C++ serial driver on PTY: explicit .5 limit, clamp and watchdog; no car."""
import os,pty,signal,struct,subprocess,tempfile,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from test_red_serial_runtime import feedback,bcc

def main():
    rclpy.init();n=Node('speed_limit_fixture');pub=n.create_publisher(Twist,'/cmd_vel',10)
    master,slave=pty.openpty();os.set_blocking(master,False);path=os.ttyname(slave)
    args=['ros2','run','turn_on_wheeltec_robot','wheeltec_robot_node','--ros-args','-p',f'usart_port_name:={path}','-p','car_mode:=mini_akm']
    log=tempfile.TemporaryFile(mode='w+');proc=subprocess.Popen(args+['-p','max_linear_mps:=0.5'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    frames=[];wire=bytearray()
    def pump(seconds,v=None):
        end=time.monotonic()+seconds;last=0
        while time.monotonic()<end:
            assert proc.poll() is None
            if time.monotonic()-last>.04:
                os.write(master,feedback())
                if v is not None:
                    m=Twist();m.linear.x=v;pub.publish(m)
                last=time.monotonic()
            rclpy.spin_once(n,timeout_sec=.005)
            try:wire.extend(os.read(master,65536))
            except BlockingIOError:pass
            while len(wire)>=11:
                f=bytes(wire[:11])
                if f[0]!=0x7b or f[10]!=0x7d or bcc(f[:9])!=f[9]:del wire[0];continue
                del wire[:11];frames.append(struct.unpack('>hhh',f[3:9]))
    try:
        pump(1.5,0.);pump(.4,.5);assert (500,0,0) in frames[-10:]
        pump(.4,.8);assert frames[-1]==(500,0,0)
        pump(.8);assert frames[-1]==(0,0,0)
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
        proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        frames.clear();wire.clear();pump(1.5,.8)
        assert frames[-1]==(200,0,0), 'default driver limit must remain .2'
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
        rejected=subprocess.run(args+['-p','max_linear_mps:=0.51'],capture_output=True,text=True,timeout=8)
        assert rejected.returncode!=0 and 'must not exceed 0.5' in rejected.stdout+rejected.stderr
        print('PASS native serial default .2 preserved, explicit .5 m/s frame, overspeed clamp, command watchdog zero, >.5 rejected (PTY)')
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();os.close(master);os.close(slave);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
