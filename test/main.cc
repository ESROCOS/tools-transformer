#include <iostream>
#include <Transformer.h>

typedef esrocos::transformer::AcyclicTransformer<20,20> atf;

int main(int argc, char ** args) {

atf tf("world");

atf::Frame base_link("base_link");
atf::Transformation odom("base_link","world","odometry");
Eigen::Matrix4f m1;
m1 << 1, 0, 0, 1,
      0, 1, 0, 1,
      0, 0, 1, 1,
      0, 0, 0, 1;

odom.atob = m1;
odom.btoa = m1.inverse();
base_link.transformToParent = odom;

std::cout << "odom: " << odom.a_ << " to " << odom.b_ << "\n" << odom.atob << "\n" << odom.b_ << " to " << odom.a_ << "\n" << odom.btoa << "\n" << std::endl;

atf::Frame lidar_1("lidar_1");
atf::Transformation link_1("lidar_1", "base_link", "link_1");
Eigen::Matrix4f m2;
m2 << 1, 0, 0, 2,
      0, 1, 0, 2,
      0, 0, 1, 2,
      0, 0, 0, 1;

link_1.atob = m2;
link_1.btoa = m2.inverse();

std::cout << "link_1: " << link_1.a_ << " to " << link_1.b_ << "\n" << link_1.atob << "\n" << link_1.b_ << " to " << link_1.a_ << "\n" << link_1.btoa << "\n" << std::endl;

lidar_1.transformToParent = link_1;

tf.addFrame(base_link);
tf.addFrame(lidar_1);

Eigen::Matrix4f t;

tf.getTransform("lidar_1","world",t);
std::cout << "template transform:\n" << (m2*m1) << std::endl;
std::cout << "link_1 to world:\n" << t << std::endl;

tf.getTransform("world","lidar_1",t);
std::cout << "template transform:\n" << (m2.inverse()*m1.inverse()) << std::endl;
std::cout << "link_1 to world:\n" << t << std::endl;


return 1;
}
