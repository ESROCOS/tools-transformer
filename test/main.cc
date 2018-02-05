#include <iostream>
#include <Transformer.h>

typedef esrocos::transformer::AcyclicTransformer<20,20> atf;

int main(int argc, char ** args) {

atf::Frame world;

atf tf(world);

atf::Frame base_link;
atf::Frame lidar_1;


tf.addFrame(base_link);
tf.addFrame(lidar_1);

std::cout << "hello world" << std::endl;
std::cout << "max frames:" << tf.frames() << std::endl;
std::cout << "string size:" << tf.ssize() << std::endl;

return 1;
}
