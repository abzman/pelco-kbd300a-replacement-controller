
#include "KerbalSimpit.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Keypad.h>
#include <ADS1115_WE.h>
#define I2C_ADDRESS 0x48

int shiftLedPin = 25;     //native pin
uint8_t ackLedPin = 14;   //on IO expander
uint8_t prevLedPin = 15;  //on IO expander

bool shiftMode = 0;
bool ackMode = 0;
bool prevMode = 0;

const byte ROWS = 8;  //eight rows
const byte COLS = 5;  //five columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  { 'N', '4', 'P', '!', 'F' },
  { '^', '3', 'E', 'M', '%' },
  { '&', '2', '9', 'C', 'A' },
  { 'S', '1', '8', 'L', 'O' },
  { ')', '0', '+', '_', 'Z' },
  { '$', '7', '[', ']', '*' },
  { '(', '6', 'T', '#', 'G' },
  { '?', '5', 'H', '@', '=' }
};
byte rowPins[ROWS] = { 23, 19, 18, 5, 17, 16, 4, 12 };  //connect to the row pinouts of the keypad
byte colPins[COLS] = { 15, 13, 14, 26, 27 };            //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

int throttleState = 0;
unsigned long startTime;

enum indicatorLED {
  shift,
  ack,
  prev
};
enum stickAxis {
  X,
  Y,
  R
};
const uint8_t SevenSegmentASCII[96] = {
  0b00000000, /* (space) */
  0b10000110, /* ! */
  0b00100010, /* " */
  0b01111110, /* # */
  0b01101101, /* $ */
  0b11010010, /* % */
  0b01000110, /* & */
  0b00100000, /* ' */
  0b00101001, /* ( */
  0b00001011, /* ) */
  0b00100001, /* * */
  0b01110000, /* + */
  0b00010000, /* , */
  0b01000000, /* - */
  0b10000000, /* . */
  0b01010010, /* / */
  0b00111111, /* 0 */
  0b00000110, /* 1 */
  0b01011011, /* 2 */
  0b01001111, /* 3 */
  0b01100110, /* 4 */
  0b01101101, /* 5 */
  0b01111101, /* 6 */
  0b00000111, /* 7 */
  0b01111111, /* 8 */
  0b01101111, /* 9 */
  0b00001001, /* : */
  0b00001101, /* ; */
  0b01100001, /* < */
  0b01001000, /* = */
  0b01000011, /* > */
  0b11010011, /* ? */
  0b01011111, /* @ */
  0b01110111, /* A */
  0b01111100, /* B */
  0b00111001, /* C */
  0b01011110, /* D */
  0b01111001, /* E */
  0b01110001, /* F */
  0b00111101, /* G */
  0b01110110, /* H */
  0b00110000, /* I */
  0b00011110, /* J */
  0b01110101, /* K */
  0b00111000, /* L */
  0b00010101, /* M */
  0b00110111, /* N */
  0b00111111, /* O */
  0b01110011, /* P */
  0b01101011, /* Q */
  0b00110011, /* R */
  0b01101101, /* S */
  0b01111000, /* T */
  0b00111110, /* U */
  0b00111110, /* V */
  0b00101010, /* W */
  0b01110110, /* X */
  0b01101110, /* Y */
  0b01011011, /* Z */
  0b00111001, /* [ */
  0b01100100, /* \ */
  0b00001111, /* ] */
  0b00100011, /* ^ */
  0b00001000, /* _ */
  0b00000010, /* ` */
  0b01011111, /* a */
  0b01111100, /* b */
  0b01011000, /* c */
  0b01011110, /* d */
  0b01111011, /* e */
  0b01110001, /* f */
  0b01101111, /* g */
  0b01110100, /* h */
  0b00010000, /* i */
  0b00001100, /* j */
  0b01110101, /* k */
  0b00110000, /* l */
  0b00010100, /* m */
  0b01010100, /* n */
  0b01011100, /* o */
  0b01110011, /* p */
  0b01100111, /* q */
  0b01010000, /* r */
  0b01101101, /* s */
  0b01111000, /* t */
  0b00011100, /* u */
  0b00011100, /* v */
  0b00010100, /* w */
  0b01110110, /* x */
  0b01101110, /* y */
  0b01011011, /* z */
  0b01000110, /* { */
  0b00110000, /* | */
  0b01110000, /* } */
  0b00000001, /* ~ */
  0b00000000, /* (del) */
};
void displayChar(int index, char charachter, uint16_t brightness = 4095) {
  byte firstSsegmentPins[] = { 7, 8, 9, 10, 11, 13, 12 };  //a,b,c,d,e,f,g
  byte secondSsegmentPins[] = { 0, 1, 2, 3, 4, 6, 5 };     //a,b,c,d,e,f,g
  uint8_t toPrint = SevenSegmentASCII[charachter - 0x20];
  for (int i = 0; i < 7; i++) {
    if (index == 0) {
      if ((toPrint & (1 << i)) > 0) {
        pwm.setPWM(firstSsegmentPins[i], 0, brightness);

      } else {
        pwm.setPWM(firstSsegmentPins[i], 0, 0);
      }
    } else if (index == 1) {
      if ((toPrint & (1 << i)) > 0) {
        pwm.setPWM(secondSsegmentPins[i], 0, brightness);

      } else {
        pwm.setPWM(secondSsegmentPins[i], 0, 0);
      }
    } else {
      Serial.println("error");
    }
  }
}
void displayNum(int number, uint16_t brightness = 4095) {
  displayChar(1, (number % 10) + 0x30, brightness);
  displayChar(0, (number / 10 % 10) + 0x30, brightness);
}
void setup() {
  Serial.begin(115200);
  Serial.println("pelco kerbal controller");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);

  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);  //comment line/change parameter to change range
  adc.setCompareChannels(ADS1115_COMP_0_GND);  //comment line/change parameter to change channel
  adc.setConvRate(ADS1115_860_SPS);            //uncomment if you want to change the default
  adc.setMeasureMode(ADS1115_CONTINUOUS);      //comment line/change parameter to change mode


  LedBrightness(shift, 0);
  LedBrightness(ack, 0);
  LedBrightness(prev, 0);

  displayChar(0, 'H', 4095);
  displayChar(1, 'I', 4095);
  delay(1000);

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
  }
  displayChar(0, ' ', 4095);
  displayChar(1, ' ', 4095);

  // Display a message in KSP to indicate handshaking is complete.
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Altitude channel.
  // The plugin will now regularly send Altitude messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(WHEEL_MESSAGE);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(LF_MESSAGE);
  throttleDown();  //display 00
}

void loop() {
  char customKey = customKeypad.getKey();

  if (customKey) {
    switch (customKey) {
      case 'S':
        modeToggle(shift);
        break;
      case 'A':
        modeToggle(ack);
        break;
      case 'P':
        modeToggle(prev);
        break;
      case 'N':
        //Serial.println("near");
        throttleState = 99;
        throttleUp();
        break;
      case 'F':
        //Serial.println("far");
        throttleUp();
        break;
      case '=':
        //Serial.println("open");
        throttleState = 0;
        throttleDown();
        break;
      case '?':
        //Serial.println("close");
        throttleDown();
        break;
      case '!':
        //Serial.println("F1");
        break;
      case '@':
        //Serial.println("F2");
        break;
      case '#':
        //Serial.println("F3");
        break;
      case ']':
        //Serial.println("aux on");
        break;
      case '[':
        //Serial.println("aux off");
        break;
      case 'O':
        //Serial.println("mon");
        break;
      case 'E':
        //Serial.println("next");
        mySimpit.activateAction(STAGE_ACTION);
        break;
      case 'H':
        //Serial.println("hold");
        break;
      case 'T':
        //Serial.println("pattern");
        break;
      case 'Z':
        //Serial.println("preset");
        break;
      case 'M':
        //Serial.println("macro");
        break;
      case 'G':
        //Serial.println("pgm");
        break;
      case 'C':
        //Serial.println("cam");
        break;
      case 'L':
        //Serial.println("clear");
        break;
      default:
        //Serial.print("number key: ");
        //Serial.println(customKey);
        break;
    }
  }
  //readAxes();
  rotationUpdate();
  throttleRepeat();
  // Call the library update() function to check for new messages.
  mySimpit.update();
#ifdef ESP8266
  yield();  // take a breather, required for ESP8266
#endif
}
void throttleRepeat() {
  if (customKeypad.findInList('F') > -1)  //up
  {
    if (customKeypad.key[customKeypad.findInList('F')].kstate == HOLD) {
      throttleUp();
      delay(10);
    }
  }
  if (customKeypad.findInList('?') > -1)  //down
  {
    if (customKeypad.key[customKeypad.findInList('?')].kstate == HOLD) {
      throttleDown();
      delay(10);
    }
  }
}
void throttleUp() {
  throttleMessage throttle_msg;
  if (throttleState > 98) {
    throttleState = 99;
  } else {
    throttleState += 1;
  }

  //displayNum(throttleState);
  // Convert it in KerbalSimpit Range
  throttle_msg.throttle = map(throttleState, 0, 99, 0, INT16_MAX);
  // Send the message
  mySimpit.send(THROTTLE_MESSAGE, throttle_msg);
}
void throttleDown() {
  throttleMessage throttle_msg;
  if (throttleState < 1) {
    throttleState = 0;
  } else {
    throttleState -= 1;
  }

  //displayNum(throttleState);
  // Convert it in KerbalSimpit Range
  throttle_msg.throttle = map(throttleState, 0, 99, 0, INT16_MAX);
  // Send the message
  mySimpit.send(THROTTLE_MESSAGE, throttle_msg);
}
void rotationUpdate() {
  rotationMessage rot_msg;
  rot_msg.setPitch(readAxis(X));
  rot_msg.setRoll(readAxis(R));
  rot_msg.setYaw(readAxis(Y));
  // Send the message
  mySimpit.send(ROTATION_MESSAGE, rot_msg);
}
void readAxes() {

  Serial.print("Xaxis:");
  Serial.print(readAxis(X));

  Serial.print(",Yaxis:");
  Serial.print(readAxis(Y));

  Serial.print(",Raxis:");
  Serial.print(readAxis(R));
  Serial.println();

  delay(20);
}
int16_t readAxis(stickAxis toRead) {
  float xDeadH = 2.65;
  float xDeadL = 2.55;
  float yDeadH = 2.64;
  float yDeadL = 2.54;
  float rDeadH = 2.50;
  float rDeadL = 2.34;

  float xMaxH = 3.37;
  float xMaxL = 1.88;
  float yMaxH = 3.32;
  float yMaxL = 1.86;
  float rMaxH = 3.62;
  float rMaxL = 1.50;

  float voltage = 0.0;
  int16_t tempV = 0;
  switch (toRead) {
    case X:
      voltage = readChannel(ADS1115_COMP_0_GND);
      if (voltage > xMaxH) {
        voltage = xMaxH;
      }
      if (voltage < xMaxL) {
        voltage = xMaxL;
      }
      if (voltage > xDeadH) {
        tempV = 1 + 32767 * ((voltage - xDeadH) / (xMaxH - xDeadH));
      } else if (voltage < xDeadL) {
        tempV = -1 - 32767 * ((voltage - xDeadL) / (xMaxL - xDeadL));
      }
      break;
    case Y:
      voltage = readChannel(ADS1115_COMP_1_GND);
      if (voltage > yMaxH) {
        voltage = yMaxH;
      }
      if (voltage < yMaxL) {
        voltage = yMaxL;
      }
      if (voltage > yDeadH) {
        tempV = 1 + 32767 * ((voltage - yDeadH) / (yMaxH - yDeadH));
      } else if (voltage < yDeadL) {
        tempV = -1 - 32767 * ((voltage - yDeadL) / (yMaxL - yDeadL));
      }
      tempV = -tempV;  //reverse the polarity
      break;
    case R:
      voltage = readChannel(ADS1115_COMP_2_GND);
      if (voltage > rMaxH) {
        voltage = rMaxH;
      }
      if (voltage < rMaxL) {
        voltage = rMaxL;
      }
      if (voltage > rDeadH) {
        tempV = 1 + 32767 * ((voltage - rDeadH) / (rMaxH - rDeadH));
      } else if (voltage < rDeadL) {
        tempV = -1 - 32767 * ((voltage - rDeadL) / (rMaxL - rDeadL));
      }
      return tempV;
      break;
    default:
      break;
  }
  return tempV;
}
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V();  // alternative: getResult_mV for Millivolt
  return voltage;
}
void modeToggle(indicatorLED index) {
  switch (index) {
    case shift:
      modeSet(shift, !shiftMode);
      break;
    case ack:
      modeSet(ack, !ackMode);
      break;
    case prev:
      modeSet(prev, !prevMode);
      break;
    default:
      //Serial.println("error");
      break;
  }
}

void modeSet(indicatorLED index, bool mode) {
  LedBrightness(index, mode * 4095);
  switch (index) {
    case shift:
      shiftMode = mode;
      break;
    case ack:
      ackMode = mode;
      if (mode) {
        mySimpit.printToKSP("Activate RCS!");
        mySimpit.activateAction(RCS_ACTION);
      } else {
        mySimpit.printToKSP("Desactivate RCS!");
        mySimpit.deactivateAction(RCS_ACTION);
      }
      break;
    case prev:
      prevMode = mode;
      if (mode) {
        mySimpit.printToKSP("Activate SAS!");
        mySimpit.activateAction(SAS_ACTION);
      } else {
        mySimpit.printToKSP("Desactivate SAS!");
        mySimpit.deactivateAction(SAS_ACTION);
      }
      break;
    default:
      //Serial.println("error");
      break;
  }
}

//0-4095 range
void LedBrightness(indicatorLED index, uint16_t pwmValue) {
  switch (index) {
    case shift:
      analogWrite(shiftLedPin, (pwmValue / 16));
      break;
    case ack:
      pwm.setPWM(ackLedPin, 0, (4095 - pwmValue));
      break;
    case prev:
      pwm.setPWM(prevLedPin, 0, (4095 - pwmValue));
      break;
    default:
      // statements
      break;
  }
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
    case ALTITUDE_MESSAGE:
      // Checking if the message is the size we expect is a very basic
      // way to confirm if the message was received properly.
      if (msgSize == sizeof(altitudeMessage)) {
        // Create a new Altitude struct
        altitudeMessage myAltitude;
        // Convert the message we received to an Altitude struct.
        myAltitude = parseMessage<altitudeMessage>(msg);
        // Turn the LED on if the vessel is higher than 500 metres
        // above sea level. Otherwise turn it off.
        if (myAltitude.sealevel > 500) {
          digitalWrite(25, HIGH);
        } else {
          digitalWrite(25, LOW);
        }
      }
      break;
    case WHEEL_MESSAGE:
      if (msgSize == sizeof(wheelMessage)) {

        wheelMessage myWheel;

        myWheel = parseMessage<wheelMessage>(msg);

        //throttleState = map(myWheel.throttle, 0, INT16_MAX, 0, 99);
        //displayNum(throttleState);
      }
      break;
    case LF_MESSAGE:
      if (msgSize == sizeof(resourceMessage)) {

        resourceMessage myFuel;

        myFuel = parseMessage<resourceMessage>(msg);
        int fuelState = map(myFuel.available, 0, myFuel.total, 0, 99);
        displayNum(fuelState);
      }
      break;
  }
}


//LED segment finder
/*
  // Drive each pin in a 'wave'
  for (uint8_t pin = 0; pin < 16; pin++) {
    pwm.setPWM(pin, 4096, 0);  // turns pin fully on
    Serial.print("pin number ");
    Serial.print(pin);
    Serial.println(" fully on");
    delay(1000);
    pwm.setPWM(pin, 0, 4096);  // turns pin fully off
  }
  */

//led fade test
/*
/*
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 96; i++) {
      displayChar(j, i + 0x20, 4095);
      delay(1000);
    }
  }
*/
/*
displayNum(55);
delay(1000);
displayNum(0);
delay(1000);
displayNum(7);
delay(1000);
displayNum(123);
delay(1000);
displayNum(9898);
delay(1000);
*/

/*
// Private : Hardware scan hacked to other polarity
void Keypad::scanKeys() {
	// Re-intialize the row pins. Allows sharing these pins with other hardware.
	for (byte r=0; r<sizeKpd.rows; r++) {
		pin_mode(rowPins[r],INPUT);
	}

	// bitMap stores ALL the keys that are being pressed.
	for (byte c=0; c<sizeKpd.columns; c++) {
		pin_mode(columnPins[c],OUTPUT);
		pin_write(columnPins[c], HIGH);	// Begin column pulse output.
		for (byte r=0; r<sizeKpd.rows; r++) {
			bitWrite(bitMap[r], c, pin_read(rowPins[r]));  // keypress is active low so invert to high.
		}
		// Set pin to high impedance input. Effectively ends column pulse.
		pin_write(columnPins[c],LOW);
		pin_mode(columnPins[c],INPUT);
	}
}*/