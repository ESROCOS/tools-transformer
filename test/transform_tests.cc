#include <iostream>
#include <Transformer.h>

#include <boost/test/unit_test.hpp>

using namespace esrocos::transformer;

typedef AcyclicTransformer<> atf20_20;
typedef AcyclicTransformer<> atf20_256;



BOOST_AUTO_TEST_SUITE(transformation_tests)

BOOST_AUTO_TEST_CASE(transform_construction)
{
  atf20_20::Transformation t;

  BOOST_CHECK_EQUAL(0,std::strcmp(t.a_,""));
  BOOST_CHECK_EQUAL(0,std::strcmp(t.b_,""));
  BOOST_CHECK_EQUAL(0,std::strcmp(t.id_,""));

  BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob_));
  BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob_));
}

BOOST_AUTO_TEST_CASE(transform_initialization)
{
 const char * a = "from";
 const char * b = "to";
 const char * id = "some_id";

 atf20_20::Transformation t(a,b,id);

 BOOST_CHECK_EQUAL(0,std::strcmp(t.a_,a));
 BOOST_CHECK_EQUAL(0,std::strcmp(t.b_,b));
 BOOST_CHECK_EQUAL(0,std::strcmp(t.id_,id));

 BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob_));
 BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob_));
}


BOOST_AUTO_TEST_SUITE_END()
