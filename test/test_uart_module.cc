#include "gtest/gtest.h"

extern "C"
{
#include "uart_module.h"
}

namespace my
{
  namespace project
  {
    namespace
    {

      // The fixture for testing class Foo.
      class UartModuleTest : public ::testing::Test
      {
      protected:
        // Code here will be called immediately after the constructor (right
        // before each test).
        void SetUp() override
        {
        }

        // Code here will be called immediately after each test (right
        // before the destructor).
        void TearDown() override
        {
        }
      };

      // Tests that the Foo::Bar() method does Abc.
      TEST_F(UartModuleTest, MethodBarDoesAbc)
      {
        const std::string input_filepath = "this/package/testdata/myinputfile.dat";
        const std::string output_filepath = "this/package/testdata/myoutputfile.dat";
        // EXPECT_EQ(f.Bar(input_filepath, output_filepath), 0);
        EXPECT_EQ(input_filepath, output_filepath);
      }

      // // Tests that Foo does Xyz.
      // TEST_F(UartModuleTest, DoesXyz)
      // {
      //   // Exercises the Xyz feature of Foo.
      // }

    } // namespace
  }   // namespace project
} // namespace my

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}