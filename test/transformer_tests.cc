#include <iostream>
#include <Transformer.h>

#include <boost/test/unit_test.hpp>

using namespace esrocos::transformer;

typedef AcyclicTransformer<20,20> atf20_20;

BOOST_AUTO_TEST_SUITE(transformer_tests)

struct Frames {
  atf20_20::Frame base_link;
  atf20_20::Frame lidar_1;

  Frames(){
    base_link = atf20_20::Frame("base_link");
    lidar_1 = atf20_20::Frame("lidar_1");
  }
};

struct Transformations {
  atf20_20::Transformation odom;
  atf20_20::Transformation link_1;

  Transformations(){
    odom = atf20_20::Transformation("base_link","world","odometry");
    link_1 = atf20_20::Transformation("lidar_1", "base_link", "link_1");
  }
};

struct Matrices{
  Eigen::Matrix4f m0,m1,m2,m3,m4,m5,m6,m7,m8,m9;

  Matrices(){
    m1 << 1, 0, 0, 1,
          0, 1, 0, 1,
          0, 0, 1, 1,
          0, 0, 0, 1;

    m2 << 1, 0, 0, 2,
          0, 1, 0, 2,
          0, 0, 1, 2,
          0, 0, 0, 1;
  }
};

BOOST_AUTO_TEST_CASE(transformer_construction)
{
  atf20_20 tf("world");
}

BOOST_AUTO_TEST_CASE(transformer_add_frames)
{
  atf20_20 tf("world");
  Frames fs;
  Transformations ts;
  Matrices ms;

  ts.odom.atob(ms.m1);
  fs.base_link.transformToParent = ts.odom;

  ts.link_1.atob(ms.m2);
  fs.lidar_1.transformToParent = ts.link_1;

  tf.addFrame(fs.base_link);
  tf.addFrame(fs.lidar_1);

  Eigen::Matrix4f t;
  bool approx;

  tf.getTransform("lidar_1","world",t);

  approx = (ms.m2*ms.m1).isApprox(t);

  BOOST_CHECK_EQUAL(true, approx);

  tf.getTransform("world","lidar_1",t);

  approx = (ms.m2.inverse()*ms.m1.inverse()).isApprox(t);

  BOOST_CHECK_EQUAL(true, approx);
}

BOOST_AUTO_TEST_SUITE_END()
