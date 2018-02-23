#include <iostream>
#include <Transformer.h>

#include <boost/test/unit_test.hpp>

typedef esrocos::transformer::AcyclicTransformer<> atf20_20;

BOOST_AUTO_TEST_SUITE(frame_tests)

BOOST_AUTO_TEST_CASE(frame_construction)
{
   atf20_20::Frame f;
}

BOOST_AUTO_TEST_CASE(frame_correct_initialization)
{
   const char * test_id = "test_id";

   atf20_20::Frame f(test_id);

   BOOST_CHECK_EQUAL(std::strcmp("test_id",f.id()),0);
}

BOOST_AUTO_TEST_CASE(frame_incorrect_initialization)
{
   const char * test_id = "veryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryveryverylongstring";

   atf20_20::Frame f(test_id);

   BOOST_CHECK_EQUAL(std::strcmp("",f.id()),0);
}

BOOST_AUTO_TEST_SUITE_END()
