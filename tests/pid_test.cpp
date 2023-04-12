#include <random>

#include "tracking/PIDController.hpp"

#include "gtest/gtest.h"

TEST(pid_tests, pid_test_1)
{
  tracking::PIDController pid(0.0, 1.0, 0.0, 1.0);

  ASSERT_NEAR(pid.get_output(0.0), 0.0, 0.05);
  ASSERT_LT(pid.get_output(0.1), 0.099);
  ASSERT_GT(pid.get_output(0.1), -0.4);
  ASSERT_LT(pid.get_output(0.1), 0.3);
}

TEST(pid_tests, pid_test_2)
{
  tracking::PIDController pid(0.0, 1.0, 0.0, 1.0);
  pid.set_pid(1.0, 0.0, 0.0);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-5.0, 5.0);

  for (int n = 0; n < 100000; n++) {
    double random_number = dis(gen);
    double output = pid.get_output(random_number);

    ASSERT_LE(output, 1.0);
    ASSERT_GE(output, -1.0);

    if (output < -2.0) {
      ASSERT_NEAR(output, -1.0, 0.01);
    }
    if (output > 2.0) {
      ASSERT_NEAR(output, 1.0, 0.01);
    }
    if (output > 0.0) {
      ASSERT_GT(output, 0.0);
    }
    if (output < 0.0) {
      ASSERT_LT(output, 0.0);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}