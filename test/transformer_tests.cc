#include <iostream>
#include <Transformer.h>
#include <boost/test/unit_test.hpp>

using namespace esrocos::transformer;

typedef AcyclicTransformer<20,20> atf20_20;

BOOST_AUTO_TEST_SUITE(transformer_tests)

/*
The following tree is modelled for testing.

[Frame]
(dynamic transformation)
<static transformation>

      [world]
        |
    (odometry)
       |
  [base_link]
    |      \
 <link_1>  (x_gimbal)
   |         \
[lidar_1]    [camera_1]
*/

struct Matrices{
  Eigen::Matrix4f m0;
  Eigen::Matrix4f m1;
  Eigen::Matrix4f m2;

  Matrices(){
    m0 << 1, 0,   0,   2,
          0, 0.3, 0,   0,
          0, 0,   2.3, 2,
          0, 0,   0,   1;

    m1 << 1, 0, 0, 1,
          0, 1, 0, 3,
          0, 0, 1, 0,
          0, 0, 0, 1;

    m2 << 2, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
  }
};

struct Transformations {
  atf20_20::Transformation odom;
  atf20_20::Transformation link_1;
  atf20_20::Transformation x_gimbal;

  Transformations(){
    odom = atf20_20::Transformation("base_link","world","odometry");
    link_1 = atf20_20::Transformation("lidar_1", "base_link", "link_1");
    x_gimbal = atf20_20::Transformation("camera_1", "base_link", "x_gimbal");
  }
};

struct Frames {
  atf20_20::Frame base_link;
  atf20_20::Frame lidar_1;
  atf20_20::Frame camera_1;

  Frames(){
    base_link = atf20_20::Frame("base_link");
    lidar_1 = atf20_20::Frame("lidar_1");
    camera_1 = atf20_20::Frame("camera_1");
  }
};

BOOST_AUTO_TEST_CASE(transformer_construction)
{
  atf20_20 tf("world");
}

BOOST_AUTO_TEST_CASE(transformer_add_frames)
{
  std::cout << "load fixtures" << std::endl;

  atf20_20 tf("world");

  Frames fs;
  Transformations ts;
  Matrices ms;

  std::cout << "build test tree" << std::endl;

  ts.odom.atob(ms.m0);
  fs.base_link.transformToParent = ts.odom;

  ts.link_1.atob(ms.m1);
  fs.lidar_1.transformToParent = ts.link_1;

  ts.x_gimbal.atob(ms.m2);
  fs.camera_1.transformToParent = ts.x_gimbal;

  tf.addFrame(fs.base_link);
  tf.addFrame(fs.lidar_1);
  tf.addFrame(fs.camera_1);

  Eigen::Matrix4f t;
  bool approx;

  tf.getTransform("lidar_1","world",t);
  approx = (ms.m1*ms.m0).isApprox(t);
  BOOST_CHECK_EQUAL(true, approx);

  tf.getTransform("world","lidar_1",t);
  approx = (ms.m1.inverse()*ms.m0.inverse()).isApprox(t);
  BOOST_CHECK_EQUAL(true, approx);

  tf.getTransform("lidar_1","camera_1",t);
  approx = (ms.m1 * ms.m2.inverse()).isApprox(t);
  BOOST_CHECK_EQUAL(true, approx);

  tf.getTransform("camera_1","lidar_1",t);
  approx = (ms.m2 * ms.m1.inverse()).isApprox(t);
  BOOST_CHECK_EQUAL(true, approx);

}

BOOST_AUTO_TEST_SUITE_END()
