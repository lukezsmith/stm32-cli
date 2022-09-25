#include "gtest/gtest.h"

extern "C"
{
#include "gpio_module.h"
}

namespace my
{
  namespace project
  {
    namespace
    {

      // The fixture for testing class Foo.
      class GPIOModuleTest : public ::testing::Test
      
      
      {
      protected:
      uint32_t virtualGPIOs;

        // Code here will be called immediately after the constructor (right
        // before each test).
        void SetUp() override
        {
          virtualGPIOs = ALL_PINS_OFF;
        }

        // Code here will be called immediately after each test (right
        // before the destructor).
        void TearDown() override
        {
        }
      };

      // Tests that the correct mode is set on specified GPIO bit
      TEST_F(GPIOModuleTest, GPIO_SetPin15ToOutput)
      {
        GPIO_SetPortMode(&virtualGPIOs, GPIO_PORT_MODE_OUTPUT, 15 );
        EXPECT_EQ((GPIO_PORT_MODE_OUTPUT << 2 * 15), (virtualGPIOs));
      }

      // Tests that the correct speed is set on specified GPIO bit
      TEST_F(GPIOModuleTest, GPIO_SetPin15ToHighSpeed)
      {
        GPIO_SetPortSpeed(&virtualGPIOs, GPIO_PORT_SPEED_HIGH, 15);
        EXPECT_EQ((GPIO_PORT_SPEED_HIGH << 2 * 15), (virtualGPIOs));
      }      

      // Tests that the pin 1 is correctly set to AF1
      TEST_F(GPIOModuleTest, GPIO_SetPin1AF)
      {
        GPIO_SetPortAF(&virtualGPIOs, 1, 1);
        EXPECT_EQ(16, (virtualGPIOs));
      }     


      // Tests that the selected GPIO bit is 1 after writing
      TEST_F(GPIOModuleTest, GPIO_TurnOnBitTwelve)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        EXPECT_EQ(0x800, virtualGPIOs);
      }

      // Tests that the selected GPIO bit is 0 after resetting
      TEST_F(GPIOModuleTest, GPIO_TurnOffBitTwelve)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        EXPECT_EQ(ALL_PINS_OFF, virtualGPIOs);
      }
      // Tests that the multiple selected GPIO bits are 1 after setting
      TEST_F(GPIOModuleTest, GPIO_TurnOnMultipleBits)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(13));
        EXPECT_EQ(0x1800, virtualGPIOs);
      }

      // Tests that the selected GPIO bit is 0 after resetting
      TEST_F(GPIOModuleTest, GPIO_TurnOffAnyBit)
      {
        GPIO_SetAll(&virtualGPIOs);
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(8));
        EXPECT_EQ(0xff7f, virtualGPIOs);
      }

      // Tests that all bits are set
      TEST_F(GPIOModuleTest, GPIO_TurnOnAllBits)
      {
        GPIO_SetAll(&virtualGPIOs);
        EXPECT_EQ(ALL_PINS_ON, virtualGPIOs);
      }
      
      // Tests an unset bit is toggled to set
      TEST_F(GPIOModuleTest, GPIO_ToggleUnsetBit)
      {
        GPIO_Toggle(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        EXPECT_EQ(0x800, virtualGPIOs);
      }

      // Tests a set bit is toggled to off
      TEST_F(GPIOModuleTest, GPIO_TurnSetBit)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        GPIO_Toggle(&virtualGPIOs, convertGPIOPinNumberToBit(12));
        EXPECT_EQ(ALL_PINS_OFF, virtualGPIOs);
      }

      // Tests multiple unset bits toggled to set in one call
      TEST_F(GPIOModuleTest, GPIO_ToggleMultipleUnsetBits)
      {
        GPIO_Toggle(&virtualGPIOs, (convertGPIOPinNumberToBit(12) | convertGPIOPinNumberToBit(13)));
        EXPECT_EQ(0x1800, virtualGPIOs);
      }

      // Tests valid bits
      TEST_F(GPIOModuleTest, GPIO_UpperAndLowerBounds)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(1));
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(16));
        EXPECT_EQ(0x8001, virtualGPIOs);
      }

      // Tests Out of Bounds Set does not affect memory
      TEST_F(GPIOModuleTest, GPIO_OutOfBoundsSetChangesNothing)
      {
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(-1));
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(0));
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(17));
        GPIO_Set(&virtualGPIOs, convertGPIOPinNumberToBit(3141));
        EXPECT_EQ(ALL_PINS_OFF, virtualGPIOs);
      }      

      // Tests Out of Bounds Reset does not affect memory
      TEST_F(GPIOModuleTest, GPIO_OutOfBoundsResetChangesNothing)
      {
        GPIO_SetAll(&virtualGPIOs);
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(0));
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(-1));
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(17));
        GPIO_Reset(&virtualGPIOs, convertGPIOPinNumberToBit(3141));
        EXPECT_EQ(ALL_PINS_ON, virtualGPIOs);
      }    

      // Tests that pin 7 is correctly read as a 1 
      TEST_F(GPIOModuleTest, GPIO_ReadPin7As1)
      {
        virtualGPIOs = 128;
        int pin = GPIO_Read(&virtualGPIOs, 7);
        EXPECT_EQ(128, pin);
      }     

      // // Tests that External Interrupt correctly configured for pin  
      // TEST_F(GPIOModuleTest, GPIO_ReadPin7As1)
      // {
      //   virtualGPIOs = 128;
      //   int pin = GPIO_Read(&virtualGPIOs, 8);
      //   EXPECT_EQ(128, pin);
      // }     


    } // namespace
  }   // namespace project
} // namespace my

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}