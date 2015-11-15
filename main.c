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
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK/2) //origianlly 2
#define GetInstructionClock()       (SYS_CLOCK)
#define I2C_CLOCK_FREQ              5000 //Originally 5000

#define GYRO_I2C_BUS              I2C1
#define GYRO_ADDRESS              0x69 //GYRO 0x69, ACCELEROMETER 0x53, 

INT8 readFromI2C(UINT8 AddrByte, UINT8 ICAddr);
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

INT8 XLOWER, XUPPER;
int x;

int main(int argc, char** argv) {
    I2C_7_BIT_ADDRESS   SlaveAddress;
   
    UINT32              actualClock;
    BOOL                Acknowledged;

    
    OpenUART1(UART_EN, UART_TX_ENABLE, 1); // enable UART1
    __XC_UART = 1;
    SYSTEMConfigPerformance(SYS_CLOCK);
    
    
    OpenTimer2( T2_ON | T2_SOURCE_INT | T2_PS_1_8, 0xFFFF);
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
    

    while(1) //Infinite Loop
    {
        //printf("Inside Infinite loop\n");
    }
   
    return (EXIT_SUCCESS);
}


// Timer2 Interrupt Service Routine
void __ISR(_TIMER_2_VECTOR, ipl2) handlesTimer2Ints(void){
        // **make sure iplx matches the timer?s interrupt priority level
        UINT8 i2cData[10];
        INT8 i2cbyte;
        
        LATAINV = 0x0008;
        // This statement looks at pin RA3, and latches RA3 the inverse of the current state.
        // In other words, it toggles the LED that is attached to RA3
        
        i2cData[1] = 0x28;              // IMU location to read (high address byte) 28
        i2cData[2] = 0x29;              // IMU location to read (low address byte) 29
        
        XUPPER = readFromI2C(i2cData[1], GYRO_ADDRESS);
        
        XLOWER = readFromI2C(i2cData[2], GYRO_ADDRESS);
   
        x = ((XUPPER << 8) | XLOWER);
        
        printf("X axis = %d\n", x);
        
        mT2ClearIntFlag();
        // Clears the interrupt flag so that the program returns to the main loop
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