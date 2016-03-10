/*******************************************************************************
 * FileName:    UARTIO.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Drivers for serial I/O
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
#include "R1D1.h"
#include "Firmata.h"

//String retSymb = "+RTINQ=";//start symble when there's any return
//String slaveName = ";SeeedBTSlave";//Set the Slave name ,caution that ';'must be included
//int nameIndex = 0;
//int addrIndex = 0;

//String recvBuf;
//String slaveAddr;

//String connectCmd = "\r\n+CONN=";

void uartio_intialize( void )
{
  setup_control_port();
}

void vUARTRxControl()
{
  const char NO_CMD = 0;
  const char PDA_CMD = 1;
  const char IROBOT_CMD = 2;
  const char FIRMATA_CMD = 3;
  const char FIRMATA_IO = 4;

  static char rx_task_state = NO_CMD;
  static int debug_counter = 16;
 
  while(SerialPort.available() > 0)
  {
    byte incomingByte = SerialPort.read();

    //debug_counter--;

    //if(debug_counter > 0)
    //{
    //  DebugPort.print(incomingByte, HEX);
    //  DebugPort.print(" ");
    //}
    //else
    //{
    //  DebugPort.println(incomingByte, HEX);
    //  debug_counter = 16;
    //}
    processInput(incomingByte);
#if(0)    
    if(NO_CMD == rx_task_state)
    {
      //SerialPort.println("PDA command 1");
      if(0x01 == incomingByte)
      {
        //#ifdef SERIAL_DEBUG
        //DebugPort.print("PDA command 1 ");
        //#endif
        rx_task_state = PDA_CMD;
      }
      else if((incomingByte > 127) && (incomingByte < 154))
      {
        #ifdef SERIAL_DEBUG
        //DebugPort.print("Roomba command ");
        //DebugPort.print(incomingByte);
        //DebugPort.print(' ');
        //DebugPort.write(incomingByte);
        #endif
        rx_task_state = IROBOT_CMD;
      }
      else if(START_SYSEX == incomingByte)
      {
        rx_task_state = FIRMATA_CMD;
      }
      else if(REPORT_ANALOG == (incomingByte & 0xF0))
      {
        rx_task_state = FIRMATA_IO;
      }
      else if(REPORT_DIGITAL == (incomingByte & 0xF0))
      {
        rx_task_state = FIRMATA_IO;
      }
      else if(SYSTEM_RESET == incomingByte)
      {
        processInput(incomingByte);
      }
    }
    else if(PDA_CMD == rx_task_state)
    {
      if(true == vPDACmd(incomingByte))
      {
        rx_task_state = NO_CMD;
      }
    }
    
    if(IROBOT_CMD == rx_task_state)
    {
      iRobotCmd(incomingByte);
      rx_task_state = NO_CMD;
    }
    
    if(FIRMATA_CMD == rx_task_state)
    {
      processInput(incomingByte);

      if(END_SYSEX == incomingByte)
      {
        rx_task_state = NO_CMD;
      }
    }
    
    if(FIRMATA_IO == rx_task_state)
    {
      processInput(incomingByte);

      if(REPORT_ANALOG != (incomingByte & 0xf0))
      {
        if(REPORT_DIGITAL != (incomingByte & 0xf0))
        {
          rx_task_state = NO_CMD;
        }
      }
    }
#endif
  }
}
  
boolean xUARTIOGetChar(byte port_number, byte *pcRxedChar, long timeout)
{
  /* Get the next character from the buffer.  Return false if no characters
     are available, or arrive before xBlockTime expires. */
  while(SerialPort.available() == 0);
  *pcRxedChar = SerialPort.read();
  return(true);
}

#ifdef SERIAL_DEBUG
void setupDebugPort()
{
#ifdef BLUETOOTH_SUPPORTED
  setup_bluetooth_port();
#else
  DebugPort.begin(57600);
  delay(100);  
  DebugPort.flush();
#endif
}
#endif

#ifdef BLUETOOTH_SUPPORTED
void setup_bluetooth_port()
{
  BluetoothPort.begin(38400);                               //Set BluetoothBee BaudRate to default baud rate 38400
  BluetoothPort.print("\r\n+STWMOD=0\r\n");                 //set the bluetooth work in slave mode
  BluetoothPort.print("\r\n+STNA=R1D1\r\n");             //set the bluetooth name
  //BluetoothPort.print("\r\n+STPIN=0000\r\n");               //Set SLAVE pincode"0000"
  BluetoothPort.print("\r\n+STOAUT=1\r\n");                    // Permit Paired device to connect me
  BluetoothPort.print("\r\n+STAUTO=0\r\n");                    // Auto-connection should be forbidden here
  delay(2000); // This delay is required.
  BluetoothPort.print("\r\n+INQ=1\r\n");                       //make the slave bluetooth inquirable 
  //BluetoothPort.println("The slave bluetooth is inquirable!");
  //BluetoothPort.print("\r\n+STAUTO=1\r\n");                  // Auto-connection
  delay(2000); // This delay is required.
  BluetoothPort.flush();
  //BluetoothPort.print("\r\n+CONN=00,80,98,44,78,88\r\n"); // Auto-connection should be forbidden here
  //BluetoothPort.print("\r\n+STAUTO=1\r\n"); // Auto-connection should be forbidden here
}
#endif


void setup_control_port()
{
#ifdef BLUETOOTH_CONTROL
  setup_bluetooth_port();
#else
  //wait 1s and flush the serial buffer
  //Serial.begin(57600);
  //delay(1000);  
  //Serial.flush();
#endif
}

/********************************************************************
*    Function Name:  vUARTIOWrite                                	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    This routine transmits a byte out the USART.   *
********************************************************************/
void vUARTIOWrite( const unsigned char *data, unsigned int byteCount )
{
  SerialPort.write(data, byteCount);
}

