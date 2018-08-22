#include <iostream>
#include <Transformer.h>
#include <boost/test/unit_test.hpp>

using namespace esrocos::transformer;

typedef AcyclicTransformer<20,20> atf20_20;
typedef atf20_20::Transformation tsf;
typedef atf20_20::Frame frame;

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
  Eigen::Matrix4d m0;
  Eigen::Matrix4d m1;
  Eigen::Matrix4d m2;

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


struct TestTree {
 atf20_20 tf;
 
 TestTree(){
  std::cout << "load fixtures" << std::endl;

  tf = atf20_20("world");

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

  std::cout << "built test tree" << std::endl;
 }
};

BOOST_AUTO_TEST_CASE(transformer_construction)
{
  atf20_20 tf("world");
}

BOOST_AUTO_TEST_CASE(transformer_get_transform)
{
  Frames fs;
  Transformations ts;
  Matrices ms;
  TestTree tree;
  atf20_20 tf = tree.tf;
	
  Eigen::Matrix4d t,t0;

  t0 << 1,0,0,0,
        0,1,0,0,
	0,0,1,0,
	0,0,0,1;
  bool approx;

  // transformation from a to a should be identity 
  tf.getTransform("lidar_1","lidar_1",t);
  approx = t0.isApprox(t);
  BOOST_CHECK_EQUAL(true, approx); 
  
  t0 << 0,0,0,0,
        0,0,0,0,
	0,0,0,0,
	0,0,0,0;

  // transformation between nonexisting frames should return all zeros matrix
  tf.getTransform("lidar_1","nonsense",t);
  approx = t0.isApprox(t);
  BOOST_CHECK_EQUAL(true,approx);

  tf.getTransform("nonesense","lidar_1",t);
  approx = t0.isApprox(t);
  BOOST_CHECK_EQUAL(true,approx);
  
  tf.getTransform("nonesense","nonsense",t);
  approx = t0.isApprox(t);
  BOOST_CHECK_EQUAL(true,approx);

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

BOOST_AUTO_TEST_CASE(update_tf){
  
  Frames fs;
  Transformations ts;
  Matrices ms;
  TestTree tree;
  atf20_20 tf = tree.tf;

  Eigen::Matrix4d u0,u1,u2,u3;

  u0 << 1,0,0,0,
        0,1,0,0,
	0,0,1,0,
	0,0,0,1;

  u1 << 1,0,0,0,
        0,1,0,0,
        0,0,1,1,
	0,0,0,1;
  
  u2 << 1,0,0,0,
        0,1,0,0,
        0,0,1,2,
	0,0,0,1;
  
  u3 << 1,0,0,0,
        0,1,0,0,
        0,0,1,3,
	0,0,0,1;


  tsf t;
  bool result;
  // update, then check
  tf.updateTransform(ts.odom.id(),u0);
  tf.getTransform(ts.odom.id(),t);
  result = t.atob().isApprox(u0);
  BOOST_CHECK_EQUAL(true,result);
  // update, then check
  tf.updateTransform(ts.odom.id(),u1);
  tf.getTransform(ts.odom.id(),t);
  result = t.atob().isApprox(u1);
  BOOST_CHECK_EQUAL(true,result);
  // update, then check
  tf.updateTransform(ts.odom.id(),u2);
  tf.getTransform(ts.odom.id(),t);
  result = t.atob().isApprox(u2);
  BOOST_CHECK_EQUAL(true,result);
  // update, then check
  tf.updateTransform(ts.odom.id(),u3);
  tf.getTransform(ts.odom.id(),t);
  result = t.atob().isApprox(u3);
  BOOST_CHECK_EQUAL(true,result);

  //should be false
  std::cout << "try to update nonexisting transform" << std::endl;
  result = tf.updateTransform("somerandomname", u0);
  std::cout << "just to be sure" << std::endl;
  BOOST_CHECK_EQUAL(false, result);
}

BOOST_AUTO_TEST_CASE(get_tf){
  
  Frames fs;
  Transformations ts;
  Matrices ms;
  TestTree tree;
  atf20_20 tf = tree.tf;

  tsf t;
  bool result;

  tf.getTransform(ts.odom.id(),t);
  
  result = ms.m0.isApprox(t.atob());
  BOOST_CHECK_EQUAL(true,result);

  // should be false
  result = tf.getTransform("somerandomnonexistingid",t);
  BOOST_CHECK_EQUAL(false,result);
}

BOOST_AUTO_TEST_CASE(transformer_getFrame){
  
  TestTree tree;
  atf20_20 tf = tree.tf;

  frame f;
  bool result;

  result = tf.getFrame("world",f);
  BOOST_CHECK_EQUAL(true,result);

  result = tf.getFrame("nonsense",f);
  BOOST_CHECK_EQUAL(false,result);
}



BOOST_AUTO_TEST_SUITE_END()



