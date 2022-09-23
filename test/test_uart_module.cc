#include "gtest/gtest.h"

extern "C"
{
#include "uart_module.h"
#include "test_uart_module.h"
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
        // Code here will be called immediately after the constructor (right
        // before each test).
        void SetUp() override
        {
          // reset UART pins
          virtualUART2 = 0;
          // (virtualUART2 | UART_SR_OFFSET) = 0xffff;
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
        EXPECT_EQ(0x2000, virtualUART2);
      }
      // Tests enabling of RX in USARTx_CR1 
      TEST_F(UartModuleTest, UART_EnableRX)
      {
        UART_EnableRX(&virtualUART2);
        EXPECT_EQ(4, virtualUART2);
      }

      // Tests enabling of TX in USARTx_CR1 
      TEST_F(UartModuleTest, UART_EnableTX)
      {
        UART_EnableTX(&virtualUART2);
        EXPECT_EQ(8, virtualUART2);
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
        EXPECT_EQ(0x2800, virtualUART2);
      }

      // Tests that UART_SetStopBit function sets correct stop bit of 0 in CR2
      TEST_F(UartModuleTest, UART_Set1StopBit)
      {
        // UART_Enable(&virtualUART2);
        UART_SetStopBits(&virtualUART2, 1);
        EXPECT_EQ(0, virtualUART2);
      }

      // Tests that UART_SetStopBit function sets correct stop bit of 1.5 in CR2
      TEST_F(UartModuleTest, UART_Set15StopBit)
      {
        UART_SetStopBits(&virtualUART2, 1.5);
        EXPECT_EQ(0x1800, virtualUART2);
      }

      // Tests that UART_EnableRXNEIE function sets correct bit in CR1
      TEST_F(UartModuleTest, UART_EnableRXNEIE)
      {
        UART_EnableRXNEIE(&virtualUART2);
        EXPECT_EQ(32, virtualUART2);
      }

      // Tests that UART_EnableTXEIE function sets correct bit in CR1
      TEST_F(UartModuleTest, UART_EnableTXEIE)
      {
        UART_EnableTXEIE(&virtualUART2);
        EXPECT_EQ(128, virtualUART2);
      }

      // // Tests that  ART_Config correctly sets UE bit in 
      // TEST_F(UartModuleTest, UART_ConfigTest)
      // {
      //   UART_Config(&virtualUART2);
      //   EXPECT_EQ(0x57, virtualRx);
      // }

      // Tests that UART_Transmit stores the correct message
      TEST_F(UartModuleTest, UART_SetCorrectTxBuffer)
      {
        virtualUART2 = (1 << 6);
        UART_Transmit(&virtualUART2, 'W');
        EXPECT_EQ((1 << 6), virtualUART2);
      }
      // Tests that UART_Read reads the correct message
      TEST_F(UartModuleTest, UART_ReadCorrectMessage)
      {
        uint32_t virtualRx = (1 << 5);
        UART_Receive(&virtualRx);
        EXPECT_EQ((1 << 5), virtualRx);
      }


    } // namespace
  }   // namespace project
} // namespace my

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}