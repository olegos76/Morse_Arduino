#include <SoftwareSerial.h>
#include "MsTimer2.h"
#include "PetitFS.h"
#include "PetitSerial.h"
#include "EERTOS.h"
#include <avr/pgmspace.h>


#define SIM800 0
#define RX  6
#define TX  5

#if SIM800 == 2
#include <Sim800L.h>

Sim800L GSM(RX,TX,7);
String sms_text;             // to save the text of the sms
uint8_t sms_index;          // to indicate the message to read.
#endif //SIM800

#define TimerPrecision 5

#define DEBUG 1

#define DOTGAP       75
#define DASHGAP      DOTGAP*3
#define SIGNGAP      DOTGAP*2
#define CHARGAP      DOTGAP*8
#define OUTCHARGAP   DOTGAP*5
#define WORDGAP      DOTGAP*8
#define RETURN_MORSE B10111011

#define outPin  4
#define button1 3
#define button2 2
#define ScanSpeed 25  // время опроса кнопки
#define USEBUTTONS 2

#if SIM800 == 1
SoftwareSerial mySerial(RX, TX);  // RX, TX
#endif //SIM800

u08 stopall=0;

PetitSerial PS;
#define Serial PS

const char msg0[] PROGMEM = "INDEX";
const char msg1[] PROGMEM =  ".TXT";
const char msg2[] PROGMEM = "SOS open file";
const char msg3[] PROGMEM = "SOS pf mount";
const char msg4[] PROGMEM = "SOS pf read";
const char msg5[] PROGMEM = "SOS pf not mounted";

#if DEBUG == 1

#define DEBUG_BUFSZ 20
u08 deb_ind=0,deb[DEBUG_BUFSZ];

void debug_task(void)
{
  if (!deb_ind) return;
  Serial.write((char)deb[0]);
  noInterrupts();
  deb_ind--;
  for (int i=0;i<deb_ind;i++)
      deb[i]=deb[i+1];
  interrupts();
}

void debug3(u08 d1)
{
  if (deb_ind==DEBUG_BUFSZ-1) return;
  noInterrupts();
  deb[deb_ind++]=d1;
  interrupts();
}
#endif // DEBUG

#if DEBUG == 1
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif //DEBUG

int squeeze_spaces(unsigned char *instr, int numchars)
{
    int dst=0;

    for (int i=0, cnt=0; i<numchars; i++){
        if (instr[i] == ' ') cnt++;
        else cnt=0;
        if (cnt < 2) {
            if (i != dst) instr[dst]=instr[i];
            dst++;
        }
    }

    if (dst < numchars) instr[dst]='\0';
    return dst;
}

static const unsigned char morse_encodings[] =
{
  // digits
  B10111111, // '0', "-----"
  B10101111, // '1', ".----"
  B10100111, // '2', "..---"
  B10100011, // '3', "...--"
  B10100001, // '4', "....-"
  B10100000, // '5', "....."
  B10110000, // '6', "-...."
  B10111000, // '7', "--..."
  B10111100, // '8', "---.."
  B10111110, // '9', "----."
  
  // --- Alphab
  B01001000, // 'A' ".-"       10
  B10010000, // 'B' "-..."
  B10010100, // 'C' "-.-."
  B01110000, // 'D' "-.."
  B00100000, // 'E' "."
  B10000100, // 'F' "..-."
  B01111000, // 'G' "--."
  B10000000, // 'H' "...."
  B01000000, // 'I' ".."
  B10001110, // 'J' ".---"
  B01110100, // 'K' "-.-"
  B10001000, // 'L' ".-.."
  B01011000, // 'M' "--"
  B01010000, // 'N' "-."
  B01111100, // 'O' "---"
  B10001100, // 'P' ".--."
  B10011010, // 'Q' "--.-"
  B01101000, // 'R' ".-."
  B01100000, // 'S' "..."
  B00110000, // 'T' "-"
  B01100100, // 'U' "..-"
  B10000010, // 'V' "...-"
  B01101100, // 'W' ".--"
  B10010010, // 'X' "-..-"
  B10010110, // 'Y' "-.--"
  B10011000, // 'Z' "--.."    35
  
  B10000110, // '.' "..--"    36
  B10001010, // ',' ".-.-"    37
  B10011100, // space      "---."  38
  B10100100, // apostrophe "..-.." 39
  RETURN_MORSE  // return  "--.--" 40
};

//  num: 48-57  alpha:  65-90   ,: 44  .: 46 CR: 10 space: 32
//                      97-122

inline unsigned char char2morse(unsigned char inchar)  // it is so complex because of small ram
{
  if      (inchar>64 && inchar<91)  return morse_encodings[inchar-55];
  else if (inchar>96 && inchar<123) return morse_encodings[inchar-87];
  else if (inchar==32 || inchar==9) return 9;  // spaces stay unchanged
  else if (inchar>47 && inchar <58) return morse_encodings[inchar-48];
  else if (inchar==',') return morse_encodings[37];
  else if (inchar=='.') return morse_encodings[36];
  else if (inchar=='\'' || inchar=='`') return morse_encodings[39];
  else if (inchar==10) return morse_encodings[40];

  return 0;
}

inline unsigned char morse2char(unsigned char morse_data)  // it is so complex because of small ram
{
  int i=0;
  if (morse_data>>5 > 5 || !(morse_data>>5)) return 0; // wrong size in input data
  while (1) {
    if (morse_encodings[i]==morse_data) {
      if      (i<10) return i+48;
      else if (i<36) return i+55;
      else {
        switch (i) {
          case 36: return '.';
          case 37: return ',';
          case 38: return ' ';
          case 39: return '\'';
          case 40: return '\n';
        }
      }
    }
    if (morse_encodings[i]==RETURN_MORSE) break;
    i++;
  }
  return 0;  // incorrect morse data
}

#define BLINK_BUF_SIZE 50

unsigned char blink_buffer[BLINK_BUF_SIZE];
unsigned char blink_buf_index=0;

void timer_blink_char(void)
{
  static unsigned char cur_data=0;  // byte to blink
  static unsigned char cur_bit=0;   // position on byte, if 0 - no selected byte
  static unsigned char cur_on=0;    // flag of PIN state

  if (stopall) {
      noInterrupts();                     
      cur_bit=0; blink_buf_index=0;
      interrupts();
      return;
  }

  if (cur_on) {                                 // need to off, then gap
    digitalWrite(outPin, LOW);
    cur_on=0;
    SetTimerTask(timer_blink_char,cur_bit?SIGNGAP:OUTCHARGAP);
    return;
  }
  if (!cur_bit) {                               // new byte
    do {
      if (!blink_buf_index) {                               // no data, waiting at ScanSpeed
        SetTimerTask(timer_blink_char,ScanSpeed);
        return;
      }
      noInterrupts();                     // get next byte from buffer 
      cur_data=blink_buffer[0];
      blink_buf_index--;
      for (int i=0; i<blink_buf_index;i++) blink_buffer[i]=blink_buffer[i+1];
      interrupts();
      if (cur_data == 9) {  // next byte is interword gap
#if DEBUG == 1
          debug3('|');
#endif  // DEBUG
        SetTimerTask(timer_blink_char,WORDGAP);
        return;
      } else if (cur_data==RETURN_MORSE) {          // next byte is return
#if DEBUG == 1
          debug3('=');
#endif  // DEBUG
        SetTimerTask(timer_blink_char,WORDGAP*2);
        return;
      }
    } while (!(cur_data>>5) || (cur_data>>5)>5);  //wrong byte, incorrect length in first 3 bits

    cur_bit=B00010000;
#if DEBUG == 1
    debug3(morse2char(cur_data));
#endif  // DEBUG
  }

  digitalWrite(outPin, HIGH);                   // next sign
  cur_on=cur_data&cur_bit;
  cur_bit>>=1;
  if (cur_bit == B00010000>>(cur_data>>5)) {// it was last bit 
    cur_bit=0;
  }
  SetTimerTask(timer_blink_char,cur_on?DASHGAP:DOTGAP);
  cur_on=1;
}

int blinkChar(unsigned char inchar)
{
  unsigned char data=char2morse(inchar);
  if (!data) {
//#if DEBUG == 1
//    Serial.write("blinkchar: no data");
//#endif  // DEBUG
    return;
  }
  if (blink_buf_index>BLINK_BUF_SIZE-1) {//overflow, wait to free
    return 0;
  }
//#if DEBUG == 1
//  Serial.write("blinkchar: ");
//  Serial.write((char)inchar);
//  Serial.write(' ');
//  if (blink_buf_index>9)
//    Serial.write(48+blink_buf_index/10);
//  Serial.write(48+blink_buf_index%10);
//  Serial.write('\n');
//#endif  // DEBUG
  noInterrupts();
  blink_buffer[blink_buf_index++]=data;
  interrupts();
  return 1;
}

int blinkString(unsigned char *instr, int numchars)  // returns number of characters added to buffer
{
  int i;
  for (i = 0; i < numchars; i++) {
    if (instr[i] && !blinkChar(instr[i])) break;
  }
  return i;
}

FATFS fs;
u08 pf_fs_mounted=0;

void blinkFile() {
  static u08 buf[33];
  static u16 buf_ind=0;
  u08 blinked;

  if (stopall) {     // alarm
    pf_lseek(0);
    buf_ind=0;
    return;
  }

  if (!pf_fs_mounted) {  // mount check
    strcpy_P(buf,msg5);
    return;
  }

  if (!buf_ind) {
    if (pf_read(buf, 32, &buf_ind)) {
      strcpy_P(buf,msg4);
      blinkString(buf, strlen(buf));
      return;
    }
    buf_ind=squeeze_spaces(buf,(int) buf_ind);
    if (buf_ind == 0) return;  // blinked all
#if DEBUG == 1
    Serial.write("read file: ");
    Serial.write(buf, buf_ind);
    Serial.write('\n');
#endif //DEBUG
  }
  blinked=blinkString(buf, buf_ind);
  if (blinked && blinked<=buf_ind) {
    buf_ind-=blinked;
    for (int i=0; i<buf_ind; i++) buf[i]=buf[i+blinked];
  }
  SetTimerTask(blinkFile,ScanSpeed);
}

void stopallio(void)
{
  if (!stopall) {
    stopall=1;
    SetTimerTask(stopallio,1000);
  } else stopall=0;
}

#define PRST_NOST  0
#define PRST_PRINT 1
#define PRST_STOP1 2
#define PRST_STOP2 3
#define PRST_STOP3 4
#define PRST_ECHO  5
#define PRST_SMS   6
#define PRST_SMSR  7

u08 scan_data=0;

void process_char(void)
{
  static u08 proc_state=PRST_NOST,buf[32],buf_ind=0;

  u08 newchar=morse2char(scan_data);
#if DEBUG == 1
  Serial.write(48+proc_state);
  Serial.write(" ic: ");
  Serial.print((char)newchar);
  Serial.write('\n');
#endif // DEBUG

  if (newchar == ',') {
    blinkChar(proc_state+48);
    return;
  }
  switch (proc_state) {
    case PRST_NOST:
      switch (newchar) {
        case 'P':
          proc_state=PRST_PRINT;
          buf_ind=0;
#if DEBUG == 1
          Serial.println("Print mode");
#endif // DEBUG
          break;
        case 'S':
          proc_state=PRST_STOP1;
#if DEBUG == 1
          Serial.println("Stop mode1");
#endif // DEBUG          
         break;
        case 'E':
          proc_state=PRST_ECHO;
#if DEBUG == 1
          Serial.println("Echo mode");
#endif // DEBUG
          break;
        case 'M':
          proc_state=PRST_SMS;
#if DEBUG == 1
          Serial.println("SMS mode");
#endif // DEBUG
          break;
      }
      break;
    case PRST_ECHO:
      if (newchar=='.') {
        proc_state=PRST_NOST;
      } else blinkChar(newchar);
      break;
    case PRST_STOP1:
      if (newchar=='.') {
        proc_state=PRST_NOST;
      } else if (newchar=='T') {
        proc_state=PRST_STOP2;
      }
      break;
    case PRST_STOP2:
      if (newchar=='.') {
        proc_state=PRST_NOST;
      } else if (newchar=='O') {
        proc_state=PRST_STOP3;
      }
      break;
    case PRST_STOP3:
      if (newchar=='.') {
        proc_state=PRST_NOST;
      } else if (newchar=='P') {
        buf_ind=0;
        stopallio();
      }
      break;
    case PRST_PRINT:
      if (newchar=='\n') {
#if DEBUG == 1
        Serial.println('prn1');
#endif //DEBUG
        if (!buf_ind) {              // on empty filename open index.txt
          strcpy_P(buf,msg0);
          buf_ind=strlen(buf);
        }
        strcpy_P(&buf[buf_ind],msg1);   // every file has txt extension
        buf_ind=strlen(buf);
        if (!pf_fs_mounted) {           // mount
          if (pf_mount(&fs)) {
#if DEBUG == 1
        Serial.println('err mnt');
#endif //DEBUG
            buf_ind=0;
            strcpy_P(buf,msg3);
            blinkString(buf, strlen(buf));
            break;
          }
          pf_fs_mounted=1;
        }
        if (pf_open(buf)) {             // open file
#if DEBUG == 1
        Serial.println('err open');
#endif //DEBUG
          buf_ind=0;
          strcpy_P(buf,msg2);
          blinkString(buf, strlen(buf));
          break;
        }
        blinkFile();                  // BLINK file
      } else if (newchar=='.') {      // exit to main menu
        proc_state=PRST_NOST;
      } else if (newchar!=' ') {
        if (buf_ind<32) buf[buf_ind++]=newchar;  // enter filename
      }
      break;
    case PRST_SMS:
      if (newchar=='.') {      // exit to main menu
        proc_state=PRST_NOST;
      } else if (newchar=='N') {
        // get number of sms'
//        String numberSms=GSM.getNumberSms(1);  // why is it sring?
      } else if (newchar=='R') {
        proc_state=PRST_SMSR;
      }
      break;
    case PRST_SMSR:
      if (newchar=='.') {      // exit to main menu
        proc_state=PRST_NOST;
      } else {
        if (newchar-48>=0 && newchar-48<10) {
//          sms_text=GSM.readSms(1);
        }
      }
      break;
  }
}

#define BAT1_open   digitalRead(button1)
#define BAT1_close !digitalRead(button1)
#define BAT2_open   digitalRead(button2)
#define BAT2_close !digitalRead(button2)
#define ST_open  0
#define ST_close 1

#if USEBUTTONS == 1
void bt_scan1(void)
{
  static u08 bt1_state=0;		 // Переменная состояния автомата
  static u08 bt1_time=0;	// Переменная времени работы автомата
  static u08 bt_data=0,bt_index=0; // Morse data

  switch(bt1_state) {		// Собственно автомат
    case ST_open: {
      if (BAT1_close) {			// Если нажата кнопка
        bt1_state = ST_close;
  			bt1_time = 0;
  		} else {
        if (bt_index) {   // something entered
          if (bt1_time > (CHARGAP/ScanSpeed-1)) { //char ended
            bt_data |= bt_index<<5;
            bt_index = 8;             // it is a flag for data ready
          }
        }
        if (bt1_time<100) bt1_time++;  // check long no push
  		}
  		break;
  	}

  	case ST_close: {
  		if(BAT1_open)	{		    // Кнопка отпущена? 
  			bt1_state = ST_open;
        if (bt1_time) {
          if (bt1_time >= (DASHGAP/ScanSpeed-1)) { // DASH detected
#if DEBUG == 1
        debug3('_');
#endif //DEBUG
            bt_data |= 1<<(4-bt_index);
          }                                // else it was dot
#if DEBUG == 1
          else debug3('.');
#endif //DEBUG
          if (++bt_index == 5) {  // 5 signs entered, that enough for one char
            bt_data |= 5<<5;
            bt_index = 8;             // it is a flag for data ready
          }
        }
  			bt1_time = 0;
      } else { // Все еще нажато? 
          if (bt1_time<100) bt1_time++;  // check very long push
  		}
    	break;
  	}
	}
  if (bt_index==8) {
#if DEBUG == 1
    debug3('\n');
#endif //DEBUG
    scan_data=bt_data;
    bt_index=0;
    bt_data=0;
    SetTask(process_char);  //characer inputed
  }
  SetTimerTask(bt_scan1,ScanSpeed);		// Зациклились через диспетчер.
}
#endif // USEBUTTONS == 1

#if USEBUTTONS == 2
#define MINSCAN 2  // *ScanSpeed - min time to consider state as one pressure
void bt_scan2(void)
{
  static u08 bt1_state=0;     // Переменная состояния автомата
  static u08 bt1_time=0;  // Переменная времени работы автомата
  static u08 bt2_state=0;     // Переменная состояния автомата
  static u08 bt2_time=0;  // Переменная времени работы автомата
  static u08 bt_data=0,bt_index=0; // Morse data

  switch(bt1_state) {  // кнопка 1
    case ST_open: {
      if (BAT1_close) {     // Если нажата кнопка
        bt1_state = ST_close;
        bt1_time = 0;
      } else {
        if (bt1_time<100) bt1_time++;  // check long no push
      }
      break;
    }

    case ST_close: {
      if(BAT1_open) {       // Кнопка отпущена? 
        bt1_state = ST_open;
        if (bt1_time>=MINSCAN) {   // *ScanSpeed 
          if (BAT2_open) {
#if DEBUG == 1
            debug3('.');
#endif //DEBUG
            if (++bt_index == 5) {  // 5 signs entered, that enough for one char
              bt_data |= 5<<5;
              bt_index = 8;             // it is a flag for data ready
            }
          }
        }
        bt1_time = 0;
      } else { // Все еще нажато? 
        if (bt1_time<100) bt1_time++;  // check very long push
      }
      break;
    }
  }

  switch(bt2_state) {   // кнопка 2
    case ST_open: {
      if (BAT2_close) {     // Если нажата кнопка
        bt2_state = ST_close;
        bt2_time = 0;
      } else {
        if (bt2_time<100) bt2_time++;  // check long no push
      }
      break;
    }

    case ST_close: {
      if(BAT2_open) {       // Кнопка отпущена? 
        bt2_state = ST_open;
        if (bt2_time>=MINSCAN) {   // *ScanSpeed otherwise it is rattling
          if (BAT1_open) {
#if DEBUG == 1
            debug3('_');
#endif //DEBUG
            bt_data |= 1<<(4-bt_index);
            if (++bt_index == 5) {  // 5 signs entered, that enough for one char
              bt_data |= 5<<5;
              bt_index = 8;             // it is a flag for data ready
            }
          }
        }
        bt2_time = 0;
      } else { // Все еще нажато? 
        if (bt2_time<100) bt2_time++;  // check very long push
      }
      break;
    }
  }

// check for GAP
  if (bt_index && bt_index<8 && bt1_state==ST_open && bt2_state==ST_open && 
      bt1_time > (CHARGAP/ScanSpeed) && bt2_time > (CHARGAP/ScanSpeed)) {   // something entered
        bt_data |= bt_index<<5;
        bt_index = 8;             // it is a flag for data ready
  }

  if (bt_index==8) {
#if DEBUG == 1
    debug3('\n');
#endif //DEBUG
    scan_data=bt_data;
    bt_index=0;
    bt_data=0;
    SetTask(process_char);  //characer inputed
  }
  
  SetTimerTask(bt_scan2,ScanSpeed);   // Зациклились через диспетчер.
}
#endif // USEBUTTONS == 2

#if SIM800 == 1

void sms(String text, String phone) {
#if DEBUG == 1
  Serial.println("SMS send started");
#endif //DEBUG
  mySerial.println("AT+CMGS=\"" + phone + "\"");
  delay(1000);
  mySerial.print(text);
  delay(300);
  mySerial.print((char)26);
  delay(300);
  Serial.println("SMS send finish");
  delay(3000);
}

#endif //SIM800

void loop()
{
  TaskManager();
#if DEBUG == 1
  debug_task();
#endif // DEBUG
}

void setup()
{
#if DEBUG == 1
  Serial.begin(9600);
#endif //DEBUG
  pinMode(outPin, OUTPUT);
  pinMode(button1, INPUT);
  digitalWrite(button1, HIGH);
  pinMode(button2, INPUT);
  digitalWrite(button2, HIGH);
#if SIM800 == 1
  mySerial.begin(19200);
#endif //SIM800

#if DEBUG == 1
  Serial.println("GSM INIT.");
#endif //DEBUG

#if SIM800 == 2
  GSM.begin(19200);
#endif //SIM800

#if DEBUG == 1
  Serial.println("GSM OK.");
#endif //DEBUG

  InitRTOS(TimerPrecision);
#if USEBUTTONS == 1
  SetTimerTask(bt_scan1,ScanSpeed);               // one button scan
#endif // USEBUTTONS == 1
#if USEBUTTONS == 2
  SetTimerTask(bt_scan2,ScanSpeed);               // two button scan
#endif // USEBUTTONS == 2

  SetTimerTask(timer_blink_char,ScanSpeed);      //  blink task
  
  MsTimer2::set(TimerPrecision, TimerService);
  MsTimer2::start();
#if DEBUG == 1
  Serial.println("Setup complete.");
#endif //DEBUG
}

