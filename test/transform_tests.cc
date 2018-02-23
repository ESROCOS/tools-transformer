#include <iostream>
#include <Transformer.h>

#include <boost/test/unit_test.hpp>

using namespace esrocos::transformer;

typedef AcyclicTransformer<> atf20_20;

BOOST_AUTO_TEST_SUITE(transformation_tests)

BOOST_AUTO_TEST_CASE(transform_construction)
{
  atf20_20::Transformation t;

  BOOST_CHECK_EQUAL(0,std::strcmp(t.a(),""));
  BOOST_CHECK_EQUAL(0,std::strcmp(t.b(),""));
  BOOST_CHECK_EQUAL(0,std::strcmp(t.id(),""));

  BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob()));
  BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob()));
}

BOOST_AUTO_TEST_CASE(transform_initialization)
{
 const char * a = "from";
 const char * b = "to";
 const char * id = "some_id";

 atf20_20::Transformation t(a,b,id);

 BOOST_CHECK_EQUAL(0,std::strcmp(t.a(),a));
 BOOST_CHECK_EQUAL(0,std::strcmp(t.b(),b));
 BOOST_CHECK_EQUAL(0,std::strcmp(t.id(),id));

 BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob()));
 BOOST_CHECK_EQUAL(true,identity.isApprox(t.atob()));
}

BOOST_AUTO_TEST_SUITE_END()
