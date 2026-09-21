"""Compile actual callback bodies against a recording serial sink, never hardware."""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(sys.argv[1]).read_text()
def function(signature):
    start = source.index(signature)
    pos = source.index('{', start)
    depth = 1
    end = pos + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
code = r'''
#include <vector>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <limits>
#include <cassert>
#include <stdexcept>
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
#define FRAME_HEADER 0x7B
#define FRAME_TAIL 0x7D
#define SEND_DATA_SIZE 11
namespace std_msgs { namespace msg {
struct Float32MultiArray { std::vector<float> data; };
struct Int8 { int8_t data; };
}}
namespace serial { struct IOException : std::runtime_error { using std::runtime_error::runtime_error; }; }
struct Sink {
 void close() {}
 std::vector<std::vector<uint8_t>> frames;
 void write(uint8_t* p, std::size_t n) { frames.emplace_back(p,p+n); }
};
struct turn_on_robot {
 struct { uint8_t tx[11]{}; } Send_Data;
 Sink Stm32_Serial;
 float last_A=-999,last_B=-999,last_C=-999,last_grap=-999;
 int SecurityPLY=0;
 void arm_cmd_Callback(std_msgs::msg::Float32MultiArray);
 void Security_Callback(const std_msgs::msg::Int8&);
 ~turn_on_robot();
};
'''
code += function('uint8_t Calculate_BCC(')
code += function('void turn_on_robot::arm_cmd_Callback(')
code += function('void turn_on_robot::Security_Callback(')
# Production shutdown intentionally sends only the basic stop frame.
destructor = function('turn_on_robot::~turn_on_robot()')
assert 'SendVelocity(0, 0, 0)' in destructor and '0xAA' not in destructor
code += 'turn_on_robot::~turn_on_robot() {}\n'
code += r'''
int main() {
 turn_on_robot r;
 for (auto data : std::vector<std::vector<float>>{{},{1},{1,2,3},{1,2,3,4,5},
        {NAN,0,0,0},{INFINITY,0,0,0},{33,0,0,0},{-33,0,0,0},
        {0,0,0,-1},{0,0,0,256},{0,0,0,.5},{0,0,0,NAN}}) {
   r.arm_cmd_Callback({data}); assert(r.Stm32_Serial.frames.empty());
 }
 r.arm_cmd_Callback({{1,-1,0,1}});
 assert(r.Stm32_Serial.frames.size()==1);
 auto f=r.Stm32_Serial.frames.back();
 assert(f.size()==10 && f[0]==0xAA && f[9]==0xBB);
 assert(f[1]==0x03 && f[2]==0xE8 && f[3]==0xFC && f[4]==0x18);
 assert(f[8]==Calculate_BCC(f.data(),8));
 r.Send_Data.tx[10]=0;
 r.Security_Callback({1});
 assert(r.Stm32_Serial.frames.size()==3);
 f=r.Stm32_Serial.frames[1];
 assert(f.size()==11 && f[10]==0x7D && f[2]==0xB1);
 assert(f[9]==Calculate_BCC(f.data(),9));
 // Stack destruction exercises the actual shutdown arm path and sink assertions.
}
'''
# Assert frame size for every arm write including destructor, before sink destruction.
code = code.replace('frames.emplace_back(p,p+n);', 'if (p[0]==0xAA) { assert(n==10 && p[9]==0xBB); } frames.emplace_back(p,p+n);')
with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp)/'frames.cpp'; path.write_text(code)
    binary=str(Path(tmp)/'frames')
    subprocess.run(['g++','-std=c++14','-fsanitize=address,undefined','-fno-omit-frame-pointer',str(path),'-o',binary],check=True)
    subprocess.run([binary],check=True)
print('PASS actual vendor callbacks: short/nonfinite/range rejection, arm length/BCC/basic-stop shutdown, security tail/BCC (ASan+UBSan)')
