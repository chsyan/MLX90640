/****************************************************************************************************************
MSP-EXP430F5529LP I2C to USB bridge, customized for MLX90640 starting with SimpleUSBBackchannel example in CCS
Resource Explorer.

Description: Receives commands from a USB to read or write I2C device registers at a 16-bit address.
In the case of reading, will send the requested number of bytes starting at a specified I2C register address back
over UART.
In the case of writing, will write one 16-bit register with one 16-bit value per command.

The USCIB0 I2C peripheral is controlled by an interrupt driven state machine. The USB peripheral uses
pre-written TI USB library functions.

ACLK = NA, MCLK = SMCLK = DCO 16MHz.



                 MSP-EXP430F5529LP                                         MLX90640
                 -----------------                                     ----------------
            /|\ |              VCC|---+---+------ +3.3V --------------|PIN 2-VDD       |
             |  |                 |   |  2.2k                         |                |
             ---|RST              |  2.2k |                           |                |
                |                 |   |   |                           |                |
                |             P3.1|---|---+- I2C Clock (UCB0SCL) -----|PIN 4-SCL       |
                |                 |   |                               |                |
                |             P3.0|---+----- I2C Data (UCB0SDA) ------|PIN 1-SDA       |
                |                 |                                   |                |
                |                 |                                   |                |
                |                 |                                   |                |
                |              GND|-------------- 0VDC ---------------|PIN 3-GND       |
                 -----------------                                     ----------------

   UBC PHAS E-Lab
   Nov 2022
   Built with CCS V12.1
******************************************************************************************************************/

// Basic MSP430 and driverLib #includes
#include "msp430.h"
#include "driverlib/MSP430F5xx_6xx/wdt_a.h"
#include "driverlib/MSP430F5xx_6xx/ucs.h"
#include "driverlib/MSP430F5xx_6xx/pmm.h"
#include "driverlib/MSP430F5xx_6xx/sfr.h"

// USB API #includes
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"

#include "USB_app/usbConstructs.h"

// Application #includes
#include "BCUart.h"           // Include the backchannel UART "library"
#include "hal.h"              // Modify hal.h to select your hardware


#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SLAVE_ADDR  0x33 //Default I2C slave address, 0x33 for MLX90620
#define TX_BUFFER_SIZE 30 //Size of the I2C TX and USB RX buffer
#define RX_BUFFER_SIZE 64 //Size of the I2C RX buffer


//I2C State Machine Modes
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;



//Globals accessed by the I2C state machine and interrupts
uint8_t g_ReceiveBuffer[RX_BUFFER_SIZE]; //Buffer used to receive data in the I2C ISR
uint16_t g_RXByteCtr; //Number of bytes left to receive
uint16_t g_ReceiveIndex; //The index of the next byte to be received in g_ReceiveBuffer
uint8_t g_TransmitBuffer[TX_BUFFER_SIZE]; //Buffer used to transmit data in the I2C ISR
uint16_t g_TXByteCtr; //Number of bytes left to transfer
uint16_t g_TransmitIndex; //The index of the next byte to be transmitted in g_TransmitBuffer
uint16_t g_TransmitRegAddr; //2 byte I2C register address to access
uint8_t g_RegByteCtr; //Index for sending two byte register address
I2C_Mode MasterMode = IDLE_MODE; //Used to track the state of the I2C state machine

uint8_t g_USBReceiveBuffer[TX_BUFFER_SIZE]; //Buffer used to receive data in the USB ISR
uint8_t g_USBReceiveIndex; //The index of the next byte to be received in g_USBReceiveIndex


//Function prototypes
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint16_t count);
uint8_t ProcessUSB(uint8_t *SlaveAddress, uint16_t *RegAddress, uint16_t *ValueorBytes);
void CopyArray(uint8_t *source, uint8_t *dest, uint16_t count);
void initClockTo16MHz();
void initGPIO();
void initI2C();
void SetVcoreUp(unsigned int level);

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count);

/*
Main:
Initialize all peripherals, and loop checking for commands from USB, and execute the command if necessary
*/

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; //Stop watchdog timer
    //Core voltage must be increased, step-by-step, for stable operation at higher clk frequencies
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);
    SetVcoreUp (0x03);
    initClockTo16MHz(); //Set main clock to 16MHz
    initGPIO(); //Configure GPIO for I2C operation
    initI2C(); //Configure the I2C hardware peripheral


    uint8_t Command = 0; //Selected command returned from ProcessUSB()
    uint8_t SlaveAddress = DEFAULT_SLAVE_ADDR; //Slave address returned from ProcessUSB()
    uint16_t NumBytesValue = 400; //Number of bytes remaining to be read, or value to write, returned from ProcessUSB()
    uint16_t RegAddress = 0x2000; //2-byte register address returned from ProcessUSB()
    uint16_t NumBytesRequested = 0; // If buffer size is smaller than total number of bytes to be read, split read command into smaller sections
    uint8_t WriteValue[2] = {0}; //2-byte array to pass write value to I2C_Master_WriteReg
    uint32_t Timeout = 0;
    uint8_t done[] = {'D', 'O', 'N', 'E'};

    USB_setup(TRUE,TRUE);  // Init USB; if a USB host (PC) is present, connect

    __enable_interrupt();


    while(1)
    {

        Command = ProcessUSB(&SlaveAddress, &RegAddress, &NumBytesValue); //Check if a command was received over USB
        if(Command == 1) //Received a read command
        {
            NumBytesRequested = 0;

            while(NumBytesValue > 0) //Keep looping until all bytes are sent
            {
                if(NumBytesValue > RX_BUFFER_SIZE)//If necessary, split remaining requests to fit the buffer size
                {
                    NumBytesRequested = RX_BUFFER_SIZE;
                }
                else
                {
                    NumBytesRequested = NumBytesValue;
                }

                I2C_Master_ReadReg(SlaveAddress, RegAddress, NumBytesRequested);//Start the I2C read request

                Timeout = 0;
                while(g_RXByteCtr > 0 && Timeout < 100000)
                    {
                    Timeout++;
                    }
                if(Timeout >= 100000)
                {

                }

                cdcSendDataInBackground(g_ReceiveBuffer, NumBytesRequested, CDC0_INTFNUM, 1000);

                NumBytesValue -= NumBytesRequested;
                RegAddress += (NumBytesRequested / 2);

            }
            //Send a 'DONE' message at the end of the read data
            cdcSendDataInBackground(done, 4, CDC0_INTFNUM, 1000);

        }
        else if(Command == 2)//Received a write command
        {
            WriteValue[0] = (NumBytesValue >> 8) & 0xFF; //Convert 16-bit value to 2-byte array
            WriteValue[1] = NumBytesValue & 0xFF;
            I2C_Master_WriteReg(SlaveAddress, RegAddress, WriteValue, 2); //Write I2C register
        }
    }

    return 0;
}


void SetVcoreUp (unsigned int level)
{
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  // Wait till SVM is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
}

/*
ProcessUSB:
Checks for valid commands coming in on the USB buffer, two different commands are supported:
Read command: 8 bytes in the format
[1-byte I2C slave address][2-byte I2C register address to start reading][2-byte Number of bytes to read][3-char escape sequence 'rdg']
Write command: 8 bytes in the format
[1-byte I2C slave address][2-byte I2C register address to write to][2-byte value to write to address][3-char escape sequence 'wrt']

Input Parameters:
None

Returns:
USBCommand:
    0: If no command detected
    1: If read command detected
    2: If write command detected
SlaveAddress: I2C slave to read or write
RegAddress: 2-byte I2C register to read or write
ValueorBytes: If read command, number of bytes to read, if write command, value to be written

*/

uint8_t ProcessUSB(uint8_t *SlaveAddress, uint16_t *RegAddress, uint16_t *ValueorBytes)
{
    uint8_t USBCommand = 0;
    static unsigned int LocalIndex = 0;
    // Look for received bytes over USB. If any, send over backchannel USB.
    g_USBReceiveIndex = cdcReceiveDataInBuffer(g_USBReceiveBuffer, sizeof(g_USBReceiveBuffer), CDC0_INTFNUM);
    while(LocalIndex < g_USBReceiveIndex)//Step through USB buffer and check for command
    {
        if(LocalIndex >= 7)//If we received at least 8 characters
        {
            if(g_USBReceiveBuffer[LocalIndex] == 'g' && g_USBReceiveBuffer[LocalIndex -1] == 'd' && g_USBReceiveBuffer[LocalIndex - 2] == 'r')//Received a read command
            {
                *SlaveAddress = g_USBReceiveBuffer[LocalIndex - 7];
                *RegAddress = (g_USBReceiveBuffer[LocalIndex - 6] << 8) | g_USBReceiveBuffer[LocalIndex - 5];
                *ValueorBytes = (g_USBReceiveBuffer[LocalIndex - 4] << 8) | g_USBReceiveBuffer[LocalIndex - 3];
                g_USBReceiveIndex = 0;
                LocalIndex = 0;
                USBCommand = 1;
            }
            else if(g_USBReceiveBuffer[LocalIndex] == 't' && g_USBReceiveBuffer[LocalIndex -1] == 'r' && g_USBReceiveBuffer[LocalIndex - 2] == 'w')//received a write command
            {
                *SlaveAddress = g_USBReceiveBuffer[LocalIndex - 7];
                *RegAddress = (g_USBReceiveBuffer[LocalIndex - 6] << 8) | g_USBReceiveBuffer[LocalIndex - 5];
                *ValueorBytes = (g_USBReceiveBuffer[LocalIndex - 4] << 8) | g_USBReceiveBuffer[LocalIndex - 3];
                g_USBReceiveIndex = 0;
                LocalIndex = 0;
                USBCommand = 2;
            }

        }
        LocalIndex++;

        if((LocalIndex >= TX_BUFFER_SIZE) || (LocalIndex > g_USBReceiveIndex))
        {
            LocalIndex = 0;
        }


    }

    return USBCommand;

}



/*
I2C_Master_ReadReg
For slave device with dev_addr, read the data specified in slaves reg_addr. Set up for 16 bit register addresses
The received data is available in ReceiveBuffer[]

Parameters:
dev_addr: The slave device address.
reg_addr: The register or command to send to the slave.
count: The length of data to read

Returns:
MasterMode: The mode of the I2C master state machine
*/
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint16_t reg_addr, uint16_t count)
{
    //Initialize state machine
    MasterMode = TX_REG_ADDRESS_MODE;
    g_TransmitRegAddr = reg_addr;

    //Initialize counters
    g_RegByteCtr = 0;
    g_RXByteCtr = count;
    g_TXByteCtr = 0;
    g_ReceiveIndex = 0;
    g_TransmitIndex = 0;

    //Initialize slave address and interrupts
    UCB0I2CSA = dev_addr;

    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT; //I2C TX, start condition

    __enable_interrupt(); //Unmask interrupts

    return MasterMode;

}


/*
I2C_Master_WriteReg
For slave device with dev_addr, writes the data specified in *reg_data, it is set up for 16 bit register addresses

Parameters:
dev_addr: The slave device address.
reg_addr: The 16-bit register or command to send to the slave.
*reg_data: The buffer to write
count: The length of *reg_data

Returns:
Mastermode: The current mode of the I2C state machine
 */

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    //Initialize state machine
    MasterMode = TX_REG_ADDRESS_MODE;
    g_TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, g_TransmitBuffer, count);

    //Initialize counters
    g_RegByteCtr = 0;
    g_TXByteCtr = count;
    g_RXByteCtr = 0;
    g_ReceiveIndex = 0;
    g_TransmitIndex = 0;

    //Initialize slave address and interrupts
    UCB0I2CSA = dev_addr;
    /*
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG); //Clear any pending interrupts
    IE2 &= ~UCB0RXIE; //Disable RX interrupt
    IE2 |= UCB0TXIE; //Enable TX interrupt
    */
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTL1 |= UCTR + UCTXSTT; //I2C TX, start condition

    return MasterMode;
}


/*
CopyArray
Copy number number of bytes specified by count, from array *source to array *dest

Parameters:
*source: Pointer to array to be copied from
*dest: Pointer to array to be copied to
count: Number of bytes to copy

Returns:
None
 */

void CopyArray(uint8_t *source, uint8_t *dest, uint16_t count)
{
    uint16_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
    UCSCTL2 = FLLD_0 + 487;                   // Set DCO Multiplier for 16MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (487 + 1) * 32768 = 16MHz
                                              // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);//
    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                          // Clear fault flags
    }while (SFRIFG1&OFIFG);                         // Test oscillator fault flag
}

void initGPIO()
{
    P3SEL |= BIT0 | BIT1 | BIT3 | BIT4; // P3.0,1 = UCB0SDA, UCB0SCL
    P4SEL |= BIT4 | BIT5; //P4.4,5 UCA1TXD/RXD

}
void initI2C()
{
    UCB0CTL1 |= UCSWRST; //SW reset while configuring peripheral
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC; //I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST; //Select SMCLK as I2C clock source, keep SW reset
    //UCB0BR0 = 160; //fSCL = SMCLK/160 = ~100kHz
    UCB0BR0 = 40; //fSCL = SMCLK/40 = ~400kHz
    //UCB0BR0 = 16; //fSCL = SMCLK/16 = ~1MHz
    //UCB0BR0 = 20;
    UCB0BR1 = 0;
    UCB0I2CSA = DEFAULT_SLAVE_ADDR; //Set default slave address
    UCB0CTL1 &= ~UCSWRST; // Clear SW reset, resume operation
}
//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t rx_val = 0;

    switch(__even_in_range(UCB0IV,0xC))
    {
        case USCI_NONE:break;                             // Vector 0 - no interrupt
        case USCI_I2C_UCALIFG:break;                      // Interrupt Vector: I2C Mode: UCALIFG
        case USCI_I2C_UCNACKIFG:break;                    // Interrupt Vector: I2C Mode: UCNACKIFG
        case USCI_I2C_UCSTTIFG:break;                     // Interrupt Vector: I2C Mode: UCSTTIFG
        case USCI_I2C_UCSTPIFG:break;                     // Interrupt Vector: I2C Mode: UCSTPIFG
        case USCI_I2C_UCRXIFG:

        rx_val = UCB0RXBUF; //Receive Data Interrupt

        if (g_RXByteCtr > 0) //Still some bytes left to receive for this transaction
        {
            g_ReceiveBuffer[g_ReceiveIndex++] = rx_val; //Write received byte to buffer
            g_RXByteCtr--;
        }

        if (g_RXByteCtr == 1) //Last byte to receive, send stop condition
        {
            UCB0CTL1 |= UCTXSTP;
        }
        else if (g_RXByteCtr == 0) //No more bytes left
        {
            UCB0IE &= ~UCRXIE; //Disable receive interrupt
            MasterMode = IDLE_MODE; //Place state machine in idle
        }
        break;

        case USCI_I2C_UCTXIFG: //Transmit Data Interrupt

        switch (MasterMode) //State machine determines which section of the I2C transaction we are in
        {
          case TX_REG_ADDRESS_MODE: //Transmitting register address
              if(g_RegByteCtr == 0) //We have to transmit the MSB of the address
              {
                  UCB0TXBUF = (uint8_t)((g_TransmitRegAddr >> 8) & 0xff);

                  g_RegByteCtr++;
              }
              else //We have to transmit the LSB of the address
              {
                  UCB0TXBUF = (uint8_t)(g_TransmitRegAddr & 0xff);
                  if (g_RXByteCtr)
                      MasterMode = SWITCH_TO_RX_MODE; //Need to start receiving now
                  else
                      MasterMode = TX_DATA_MODE; //Continue to transmission of data in Transmit Buffer
              }
          break;

          case SWITCH_TO_RX_MODE: //Transition to receive mode
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTL1 &= ~UCTR; //Switch to receiver
              MasterMode = RX_DATA_MODE; //Change to receive data state
              UCB0CTL1 |= UCTXSTT; //Send repeated start condition (very important for MLX90640!)
              if (g_RXByteCtr == 1) //If this is already the last byte
              {
                  while((UCB0CTL1 & UCTXSTT)); //Wait for last byte to get sent
                  UCB0CTL1 |= UCTXSTP; //Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (g_TXByteCtr > 0) //Still some bytes in buffer to send
              {
                  UCB0TXBUF = g_TransmitBuffer[g_TransmitIndex++];
                  g_TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTL1 |= UCTXSTP; //Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;  // disable TX interrupt

              }
              break;

          default:
              __no_operation();
              break;
      }
  }
}

