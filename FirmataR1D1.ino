#include "R1D1.h"
#include "Firmata.h"
//static bool parsingSysex = false;
static byte firmwareVersionCount;
static byte *firmwareVersionVector;
    /* input message handling */
static byte waitForData; // this flag says the next serial input will be data
static byte executeMultiByteCommand; // execute this after getting multi-byte data
static byte multiByteChannel; // channel data for multiByteCommands
static byte storedInputData[MAX_DATA_BYTES]; // multi-byte data
    /* sysex */
static boolean parsingSysex = false;
static int sysexBytesRead;
    /* pin configuration */
    byte pinConfig[TOTAL_PINS];
    int pinState[TOTAL_PINS];

#if 1
//------------------------------------------------------------------------------
// Serial Send Handling

/**
 * @param pin The pin to get the configuration of.
 * @return The configuration of the specified pin.
 */
byte getPinMode(byte pin)
{
  return pinConfig[pin];
}

/**
 * Set the pin mode/configuration. The pin configuration (or mode) in Firmata represents the
 * current function of the pin. Examples are digital input or output, analog input, pwm, i2c,
 * serial (uart), etc.
 * @param pin The pin to configure.
 * @param config The configuration value for the specified pin.
 */
void setPinMode(byte pin, byte config)
{
  if (pinConfig[pin] == PIN_MODE_IGNORE)
    return;

  pinConfig[pin] = config;
}

/**
 * @param pin The pin to get the state of.
 * @return The state of the specified pin.
 */
int getPinState(byte pin)
{
  return pinState[pin];
}

/**
 * Set the pin state. The pin state of an output pin is the pin value. The state of an
 * input pin is 0, unless the pin has it's internal pull up resistor enabled, then the value is 1.
 * @param pin The pin to set the state of
 * @param state Set the state of the specified pin
 */
void setPinState(byte pin, int state)
{
  pinState[pin] = state;
}

// send an analog message
void sendAnalog(byte pin, int value)
{
  // pin can only be 0-15, so chop higher bits
  SerialPort.write(ANALOG_MESSAGE | (pin & 0xF));
  sendValueAsTwo7bitBytes(value);
}

// send a single digital pin in a digital message
void sendDigital(byte pin, int value)
{
  /* TODO add single pin digital messages to the protocol, this needs to
   * track the last digital data sent so that it can be sure to change just
   * one bit in the packet.  This is complicated by the fact that the
   * numbering of the pins will probably differ on Arduino, Wiring, and
   * other boards.  The DIGITAL_MESSAGE sends 14 bits at a time, but it is
   * probably easier to send 8 bit ports for any board with more than 14
   * digital pins.
   */

  // TODO: the digital message should not be sent on the serial port every
  // time sendDigital() is called.  Instead, it should add it to an int
  // which will be sent on a schedule.  If a pin changes more than once
  // before the digital message is sent on the serial port, it should send a
  // digital message for each change.

  //    if(value == 0)
  //        sendDigitalPortPair();
}


// send 14-bits in a single digital message (protocol v1)
// send an 8-bit port in a single digital message (protocol v2)
void sendDigitalPort(byte portNumber, int portData)
{
  SerialPort.write(DIGITAL_MESSAGE | (portNumber & 0xF));
  SerialPort.write((byte)portData % 128); // Tx bits 0-6
  SerialPort.write(portData >> 7);  // Tx bits 7-13
}

void sendSysex(byte command, byte bytec, byte *bytev)
{
  byte i;
  startSysex();
  SerialPort.write(command);
  for (i = 0; i < bytec; i++) {
    sendValueAsTwo7bitBytes(bytev[i]);
  }
  endSysex();
}

void sendString(byte command, const char *string)
{
  sendSysex(command, strlen(string), (byte *)string);
}


// send a string as the protocol string type
void sendString(const char *string)
{
  sendString(STRING_DATA, string);
}

// resets the system state upon a SYSTEM_RESET message from the host software
void systemReset(void)
{
  byte i;

  waitForData = 0; // this flag says the next serial input will be data
  executeMultiByteCommand = 0; // execute this after getting multi-byte data
  multiByteChannel = 0; // channel data for multiByteCommands

  for (i = 0; i < MAX_DATA_BYTES; i++) {
    storedInputData[i] = 0;
  }

  parsingSysex = false;
  sysexBytesRead = 0;

  systemResetCallback();
}

void InitFirmata(void)
{
  firmwareVersionCount = 0;
  firmwareVersionVector = 0;
  systemReset();
  setFirmwareNameAndVersion(__FILE__, FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);
}

// output the protocol version message to the serial port
void printVersion(void)
{
  SerialPort.write(REPORT_VERSION);
  SerialPort.write(FIRMATA_MAJOR_VERSION);
  SerialPort.write(FIRMATA_MINOR_VERSION);
}

void sendValueAsTwo7bitBytes(int value)
{
  SerialPort.write(value & B01111111); // LSB
  SerialPort.write(value >> 7 & B01111111); // MSB
}

void startSysex(void)
{
  SerialPort.write(START_SYSEX);
}

void endSysex(void)
{
  SerialPort.write(END_SYSEX);
}

void printFirmwareVersion(void)
{
  byte i;

  if (firmwareVersionCount)
  { // make sure that the name has been set before reporting
    startSysex();
    //digitalWrite(8, HIGH);

    SerialPort.write(REPORT_FIRMWARE);
    SerialPort.write(firmwareVersionVector[0]); // major version number
    SerialPort.write(firmwareVersionVector[1]); // minor version number
    
    for (i = 2; i < firmwareVersionCount; ++i)
    {
      sendValueAsTwo7bitBytes(firmwareVersionVector[i]);
    }
    endSysex();
  }
}

void setFirmwareNameAndVersion(const char *name, byte major, byte minor)
{
  const char *firmwareName;
  const char *extension;

  // parse out ".cpp" and "applet/" that comes from using __FILE__
  extension = strstr(name, ".cpp");
  firmwareName = strrchr(name, '/');

  if (!firmwareName) {
    // windows
    firmwareName = strrchr(name, '\\');
  }
  if (!firmwareName) {
    // user passed firmware name
    firmwareName = name;
  } else {
    firmwareName ++;
  }

  if (!extension) {
    firmwareVersionCount = strlen(firmwareName) + 2;
  } else {
    firmwareVersionCount = extension - firmwareName + 2;
  }

  // in case anyone calls setFirmwareNameAndVersion more than once
  free(firmwareVersionVector);

  firmwareVersionVector = (byte *) malloc(firmwareVersionCount + 1);
  firmwareVersionVector[firmwareVersionCount] = 0;
  firmwareVersionVector[0] = major;
  firmwareVersionVector[1] = minor;
  strncpy((char *)firmwareVersionVector + 2, firmwareName, firmwareVersionCount - 2);
}

void processSysexMessage(void)
{
  switch (storedInputData[0]) 
  { //first byte in buffer is command
    case REPORT_FIRMWARE:
      printFirmwareVersion();
      break;
    case STRING_DATA:
    #if 0
      if (currentStringCallback)
      {
        byte bufferLength = (sysexBytesRead - 1) / 2;
        byte i = 1;
        byte j = 0;
        while (j < bufferLength)
        {
          // The string length will only be at most half the size of the
          // stored input buffer so we can decode the string within the buffer.
          storedInputData[j] = storedInputData[i];
          i++;
          storedInputData[j] += (storedInputData[i] << 7);
          i++;
          j++;
        }
        // Make sure string is null terminated. This may be the case for data
        // coming from client libraries in languages that don't null terminate
        // strings.
        if (storedInputData[j - 1] != '\0') {
          storedInputData[j] = '\0';
        }
        (*currentStringCallback)((char *)&storedInputData[0]);
      }
      #endif
      break;
    default:
      sysexCallback(storedInputData[0], sysexBytesRead - 1, storedInputData + 1);
    break;
    #if 0
      if (currentSysexCallback)
        (*currentSysexCallback)(storedInputData[0], sysexBytesRead - 1, storedInputData + 1);
    #endif
  }
}

void processInput(unsigned char inputData)
{
  //int inputData = FirmataStream->read(); // this is 'int' to handle -1 when no data
  int command;

  // TODO make sure it handles -1 properly

  if (parsingSysex)
  {
    if (inputData == END_SYSEX) {
      //stop sysex byte
      parsingSysex = false;
      //fire off handler function
      processSysexMessage();
    } else {
      //normal data byte - add to buffer
      storedInputData[sysexBytesRead] = inputData;
      sysexBytesRead++;
    }
  } 
  else if ( (waitForData > 0) && (inputData < 128) )
  {
    waitForData--;
    storedInputData[waitForData] = inputData;
    if ( (waitForData == 0) && executeMultiByteCommand ) 
    { // got the whole message
      switch (executeMultiByteCommand) {
        case ANALOG_MESSAGE:
          analogWriteCallback(multiByteChannel,(storedInputData[0] << 7) + storedInputData[1]);
        break;
        
        case DIGITAL_MESSAGE:
          digitalWriteCallback(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
        break;
        
        case SET_PIN_MODE:
          setPinModeCallback(storedInputData[1], storedInputData[0]);
        break;
        
        case REPORT_ANALOG:
          reportAnalogCallback(multiByteChannel, storedInputData[0]);
         break;
         
        case REPORT_DIGITAL:
          reportDigitalCallback(multiByteChannel, storedInputData[0]);
        break;
      }
      executeMultiByteCommand = 0;
    }
  }
  else
  {
    // remove channel info from command byte if less than 0xF0
    if (inputData < 0xF0) {
      command = inputData & 0xF0;
      multiByteChannel = inputData & 0x0F;
    } else {
      command = inputData;
      // commands in the 0xF* range don't use channel data
    }
    switch (command) {
      case ANALOG_MESSAGE:
      case DIGITAL_MESSAGE:
      case SET_PIN_MODE:
        waitForData = 2; // two data bytes needed
        executeMultiByteCommand = command;
        break;
      case REPORT_ANALOG:
      case REPORT_DIGITAL:
        waitForData = 1; // one data byte needed
        executeMultiByteCommand = command;
        break;
      case START_SYSEX:
        parsingSysex = true;
        sysexBytesRead = 0;
        break;
      case SYSTEM_RESET:
        systemReset();
        break;
      case REPORT_VERSION:
        printVersion();
        break;
    }
  }
}
#endif
