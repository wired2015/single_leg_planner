#include <boost/test/unit_test.hpp>
#include <single_leg_planner/Dummy.hpp>

using namespace single_leg_planner;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    single_leg_planner::DummyClass dummy;
    dummy.welcome();
}
