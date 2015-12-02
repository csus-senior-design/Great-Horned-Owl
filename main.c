/* 
 * File:   main.c
 * Author: Sean Kennedy
 *
 * Created on October 28, 2015, 11:44 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include <plib.h>

 
// Clock Constants
#if defined (__32MX460F512L__) || defined (__32MX360F512L__) || defined (__32MX795F512L__)
#define SYS_CLOCK (80000000L)
#elif defined (__32MX220F032D__) || defined (__32MX250F128D__)
#define SYS_CLOCK (40000000L)
#endif
//Information related to MAX32 System
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK/2) //origianlly 2
#define GetInstructionClock()       (SYS_CLOCK)
#define I2C_CLOCK_FREQ              5000 //Originally 5000

//Information related to GYRO IC
#define GYRO_I2C_BUS              I2C1
#define GYRO_ADDRESS              0x69 //GYRO 0x69, ACCELEROMETER 0x53,
#define CTRL_REG_1                0x20
#define CTRL_REG_2                0x21
#define CTRL_REG_3                0x22
#define CTRL_REG_4                0x23
#define CTRL_REG_5                0x24
#define XUPPER_REG                0x28      
#define XLOWER_REG                0x29
#define YUPPER_REG                0x2B
#define YLOWER_REG                0x2A
#define ZUPPER_REG                0x2D
#define ZLOWER_REG                0x2C

//Config Settings for GYRO
#define DATA_CTRL_REG1            0x0F  
#define DATA_CTRL_REG2            0x00
#define DATA_CTRL_REG3            0x08
#define DATA_CTRL_REG4_250        0x00
#define DATA_CTRL_REG4_500        0x10
#define DATA_CTRL_REG4Default    0x30

//GRYO DSP Constants
#define SAMPLE_RATE_GYRO          100
#define OFFSET_GYRO               136
#define SENSITIVITY_GYRO          5.5


INT8 readFromI2C(UINT8 AddrByte, UINT8 ICAddr);
void writeFromI2C(UINT8 AddrByte, UINT8 ICAddr, UINT8 DatatoWrite);
void delayCycles(int numberofcycles);
/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the IMU.

  Description:
    This routine starts (or restarts) a transfer to/from the IMU, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition
    
  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling
    
  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(GYRO_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(GYRO_I2C_BUS) );

        if(I2CStart(GYRO_I2C_BUS) != I2C_SUCCESS)
        {
            printf("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(GYRO_I2C_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}

/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the IMU.

  Description:
    This transmits one byte to the IMU, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(GYRO_I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(GYRO_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        printf("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(GYRO_I2C_BUS));

    return TRUE;
}

/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the IMU.

  Description:
    This routine Stops a transfer to/from the IMU, waiting (in a 
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.
    
  Returns:
    None.
    
  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(GYRO_I2C_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(GYRO_I2C_BUS);

    } while ( !(status & I2C_STOP) );
}

INT8 XLOWER, XUPPER, YLOWER, YUPPER, ZLOWER, ZUPPER;
int xrate, yrate, zrate;
float xangle = 0.0;
float yangle = 0.0;
float zangle = 0.0;

int main(int argc, char** argv) {
    I2C_7_BIT_ADDRESS   SlaveAddress;
   
    UINT32              actualClock;

    
    OpenUART1(UART_EN, UART_TX_ENABLE, 1); // enable UART1
    __XC_UART = 1;
    SYSTEMConfigPerformance(SYS_CLOCK);
    
    
    OpenTimer2( T2_ON | T2_SOURCE_INT | T2_PS_1_32, 0xFFFF);
        // This statement says: turn on timer2 | have it use an internal clock source | have it
        // use a prescaler of 1:256, and use a period of 0xFFFF or 2^16 cycles
 
        // Timer2 as configured would trigger an interrupt at a frequency of (80MHZ/256/65535), or 4.77
        // times a second.
 
        ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_2);// This statement configured the timer to produce an interrupt with a priority of 2
 
    INTEnableSystemMultiVectoredInt();// Use multi-vectored interrupts
    
    TRISAbits.TRISA3 = 0; //Sets RA3 as a Output


    // Initialize debug messages (when supported)
    DBINIT();

    // Set the I2C baudrate
    printf("Initializing the I2C Bus\n");
    actualClock = I2CSetFrequency(GYRO_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        printf("Error: I2C1 clock frequency (%u) error exceeds 10%%.\n", (unsigned)actualClock);
    }

    // Enable the I2C bus
    I2CEnable(GYRO_I2C_BUS, TRUE);
    
    writeFromI2C(CTRL_REG_1, GYRO_ADDRESS, DATA_CTRL_REG1); // Enable x, y, z and turn off power down
    //writeFromI2C(CTRL_REG_2, GYRO_ADDRESS, DATA_CTRL_REG2);// If you'd like to adjust/use the HPF, you can edit this line to configure CTRL_REG2
    
    // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
   // writeFromI2C(CTRL_REG_3, GYRO_ADDRESS, DATA_CTRL_REG3);
    
    //writeFromI2C(CTRL_REG_4, GYRO_ADDRESS, DATA_CTRL_REG4_500);
    
    delayCycles(1500);
    printf("X,Y,Z (Degrees per Second)");
    while(1) //Infinite Loop
    {
        //printf("Inside Infinite loop\n");
    }
   
    return (EXIT_SUCCESS);
}

void delayCycles(int numberofcycles)
{
    int count = 0;
    do
    {
        count++;
        Nop();
    }while (count != numberofcycles);
}
// Timer2 Interrupt Service Routine
void __ISR(_TIMER_2_VECTOR, ipl2) handlesTimer2Ints(void){
        // **make sure iplx matches the timer?s interrupt priority level
        UINT8 i2cData[10];
        INT8 i2cbyte;
        
        LATAINV = 0x0008; //Make the LED Blink when Timer INT code runs
        
        //Grab X,Y,Z rotational data from gyro
        XUPPER = readFromI2C(XUPPER_REG, GYRO_ADDRESS);      
        XLOWER = readFromI2C(XLOWER_REG, GYRO_ADDRESS);
   
        YUPPER = readFromI2C(YUPPER_REG, GYRO_ADDRESS);
        YLOWER = readFromI2C(XLOWER_REG, GYRO_ADDRESS);
                
        ZUPPER = readFromI2C(ZUPPER_REG, GYRO_ADDRESS);
        ZLOWER = readFromI2C(ZLOWER_REG, GYRO_ADDRESS);
        
        xrate = XLOWER | (XUPPER << 8); //Concatenates the 2 bytes together
        yrate = YLOWER | (YUPPER << 8);
        zrate = ZLOWER | (ZUPPER << 8);
        
        //Calculate the Angles of the axis
        
        
        printf("%d,%d,%d\n", xrate, yrate, zrate);
        
        mT2ClearIntFlag(); // Clears the interrupt flag so that the program returns to the main loop
} // END Timer2 ISR

INT8 readFromI2C(UINT8 AddrByte, UINT8 ICAddr)
{
     int Index;
     int  DataSz;
     UINT8 i2cData[10];
     I2C_7_BIT_ADDRESS   SlaveAddress;
     BOOL Success = TRUE;
     INT8 i2cbyte;
      
      //
      // Read the data back from the IMU.
      //

        // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, ICAddr, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = AddrByte;              // IMU location to read (high address byte)
        DataSz = 2;
    
        // Start the transfer to read the IMU.
        if( !StartTransfer(FALSE) )
        {
            while(1);
        }
    
        // Address the IMU.
        Index = 0;
        while( Success & (Index < DataSz) )
        {
            // Transmit a byte
            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
            }
            else
            {
                Success = FALSE;
            }

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(GYRO_I2C_BUS))
            {
                printf("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }

        // Restart and send the GYRO's internal address to switch to a read transfer
        if(Success)
        {
            // Send a Repeated Started condition
            if( !StartTransfer(TRUE) )
            {
                while(1);
            }

            // Transmit the address with the READ bit set
            I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, GYRO_ADDRESS, I2C_READ);
            if (TransmitOneByte(SlaveAddress.byte))
            {
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(GYRO_I2C_BUS))
                {
                    printf("Error: Sent byte was not acknowledged\n");
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }

        // Read the data from the desired address
        if(Success)
        {
            if(I2CReceiverEnable(GYRO_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
            {
                printf("Error: I2C Receive Overflow\n");
                Success = FALSE;
            }
            else
            {
                while(!I2CReceivedDataIsAvailable(GYRO_I2C_BUS));
                i2cbyte = I2CGetByte(GYRO_I2C_BUS);
            }

        }

        // End the transfer (stop here if an error occurred)
        StopTransfer();
        if(!Success)
        {
            printf("Error in Transmission\n");
            while(1);
        }
        
        return i2cbyte;
}

void writeFromI2C(UINT8 AddrByte, UINT8 ICAddr, UINT8 DatatoWrite)
{
     int Index;
     int  DataSz;
     UINT8 i2cData[10];
     I2C_7_BIT_ADDRESS   SlaveAddress;
     BOOL Success = TRUE;
     BOOL Acknowledged;
     INT8 i2cbyte;
    
      // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, ICAddr, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = AddrByte;              // IMU location to read (high address byte)
        i2cData[2] = DatatoWrite;              // Data to write
        DataSz = 3;
     
     
        // Start the transfer to write data to the EEPROM
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }

    // Transmit all data
    Index = 0;
    while( Success && (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(GYRO_I2C_BUS))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }


    // Wait for EEPROM to complete write process, by polling the ack status.
    Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
            while(1);
        }
        
        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(GYRO_I2C_BUS);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        StopTransfer();
        if(!Success)
        {
            while(1);
        }

    } while (Acknowledged != TRUE);

}