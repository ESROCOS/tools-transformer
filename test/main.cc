#include <iostream>
#include <Transformer.h>

typedef esrocos::transformer::AcyclicTransformer<20,20> atf;

int main(int argc, char ** args) {

atf::Frame world("world");

atf tf(world);

atf::Frame base_link("base_link");
atf::Transformation odom("base_link","world","odometry");
base_link.transformToParent = odom;

atf::Frame lidar_1("lidar_1");
atf::Transformation link_1("lidar_1", "base_link", "link_1");
lidar_1.transformToParent = link_1;

tf.addFrame(base_link);
tf.addFrame(lidar_1);


return 1;
}
