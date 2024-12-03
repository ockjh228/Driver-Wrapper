#include <gtest/gtest.h>

#include <memory>

#include <chrono>
//#include "core/calculator.hpp" //*Please import the target class for the unit test.

class TestFixture : public ::testing::Test {
protected:
    /**
     * @brief Declare variables for the test
     * @example
        Calculator calc;  //Target Class 
        int a, b;  //Input Variables 
        int expected_sum, expected_difference;  //Expected Returns 
        std::chrono::milliseconds max_duration;  //Excution Time Limit
     */

    virtual void SetUp() {
    /**
     * @brief Initialize the variables for the test
     * @example
     *  a = 1;
        b = 2;
        expected_sum = 3;
        expected_difference = -1;
        max_duration = std::chrono::milliseconds(100);
     */
    }

    virtual void TearDown() {
      /**
       * @brief Reset the variables for the test
       * 
       */
    }
};

TEST_F(TestFixture, Check_ExpectedValue) {
    /**
     * @brief Test the expected value is same as the return value
     * @example
        int sum = calc.Add(a, b); //When
        EXPECT_EQ(sum, expected_sum); //Then
     */
    EXPECT_EQ(0,1);
}

TEST_F(TestFixture, Check_ExecutionTime) {
    /**
     * @brief Test the execution time is less than the time limit
     * @example
        auto start_time = std::chrono::high_resolution_clock::now();  // Start time counting
        SomeFunction();  // Call the target function
        auto end_time = std::chrono::high_resolution_clock::now();  // End time counting
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        EXPECT_LE(duration, max_duration);
     */

    auto start_time = std::chrono::high_resolution_clock::now();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    EXPECT_LE(duration.count(), 1000);
}

int main(int argc, char **argv) {
  /**
   * @brief Test all unit test cases
   */
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}