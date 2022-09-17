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
      uint32_t virtualUART2;
      uint32_t virtualRX;
      uint32_t virtualTX;
        // Code here will be called immediately after the constructor (right
        // before each test).
        void SetUp() override
        {
          // reset RX/TX buffers (?)
          // virtualRX = ALL_PINS_OFF;
          // virtualTX = ALL_PINS_OFF;

          // reset UART pins
          virtualUART2 = 0;

          // placeholder message to send
          char txMessage[] = "Write anything on Serial Terminal\r\n";

          // placeholder message to receive
          char rxMessage[] = "Hello, World!\r\n";
          
        }

        // Code here will be called immediately after each test (right
        // before the destructor).
        void TearDown() override
        {
        }
      };
      // Tests that UART_Enable function enables UART2 in CR1 register
      TEST_F(UartModuleTest, UART_EnableUART2)
      {
        UART_Enable(&virtualUART2);
        EXPECT_EQ(0x1000, virtualUART2);
      }

      // Tests that UART_SetWordLength function sets correct word length of 8
      TEST_F(UartModuleTest, UART_Set8WordLength)
      {
        // UART_Enable(&virtualUART2);
        UART_SetWordLength(&virtualUART2, 8);
        EXPECT_EQ(0, virtualUART2);
      }

      // Tests that UART_SetWordLength function sets correct word length of 9
      TEST_F(UartModuleTest, UART_Set9WordLength)
      {
        // UART_Enable(&virtualUART2);
        UART_SetWordLength(&virtualUART2, 9);
        EXPECT_EQ(0x800, virtualUART2);
      }
      
      // Tests that UART_SetWordLength function does not set an out of bounds word length
      TEST_F(UartModuleTest, UART_DoesNotSetWordLengthOutOfBounds)
      {
        // UART_Enable(&virtualUART2);
        UART_SetWordLength(&virtualUART2, 10);
        UART_SetWordLength(&virtualUART2, -1);
        UART_SetWordLength(&virtualUART2, 7);
        EXPECT_EQ(0, virtualUART2);
      }

      // Tests that Enabling and Setting Word Length correctly sets CR1 register
      TEST_F(UartModuleTest, UART_EnableAndWordLength8)
      {
        UART_Enable(&virtualUART2);
        UART_SetWordLength(&virtualUART2, 9);
        EXPECT_EQ(0x1800, virtualUART2);
      }

      // // Tests that UART_Transmit function places correct message in buffer for TX
      // TEST_F(UartModuleTest, UART_CorrectMessageInBuffer)
      // {

      //   EXPECT_EQ(input_filepath, output_filepath);
      // }

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