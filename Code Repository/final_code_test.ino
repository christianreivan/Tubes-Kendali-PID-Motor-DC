#define outMax 50.0
#define outMin 0.0

// PID parameters
#define P 20
#define I 0.15
#define D 25
#define setPoint 1.0

bool manual_flag = false;
bool autokatz_flag = true;

//periode timer 5 ms
unsigned long int SampleTime = 100;

// Create a union to easily convert float to byte
typedef union{
  byte bytes[64];
  float number;
} FLOATUNION_t;

// Create the variable you want to send
FLOATUNION_t myValue;
const int buffer_size = 64;
byte buff[buffer_size];

// Create stored data from MATLAB
float input;
float output;

// Necessary variables
float error, errorI, errorD, prevError;
float kp, ki, kd;

//timer intrerrupt flag
volatile int timer_intr=0;

// filtered signal for reducing spike due to derivative term
float filt_input, last_filt_input, lastinput;

// automatic tuning
bool AutoKatz = false;

void SetTuningsOwn(float Kp, float Ki, float Kd)
{
  float SampleTimeInSec = ((float) SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTimeOwn(unsigned long int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float) NewSampleTime/(float) SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long int) NewSampleTime;
   }
}

void SetModeOwn(bool Mode)
{
    bool newAutoKatz = (Mode == autokatz_flag);
    if(newAutoKatz && !AutoKatz) {
      InitializeOwn();
    }
    AutoKatz = newAutoKatz;
}
 
void InitializeOwn()
{
   lastinput = input;
   errorI = output;
   if(errorI > outMax) {
    errorI = outMax;
   }
   else if(errorI < outMin) {
    errorI = outMin;
   }
}

void PIDCompute(){
  if (!AutoKatz) {
    return;
  }
  error = setPoint - input;
  errorI += (ki*error);
  errorD = (filt_input-last_filt_input);
  
  ////
  if (errorI > outMax) {
    errorI = outMax;
  }
  else if (errorI < outMin) {
    errorI = outMin;
  }
  ////
  if (output > outMax) {
    output = outMax;
  }
  else if (output < outMin) {
    output = outMin;
  }
  else {
    output = (kp*error) + errorI - (kd*errorD);
  }
  lastinput = input;
  last_filt_input = filt_input;
}

// Register Settings
void RegSet() {
  cli();
  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 15,625 kHz
  // Mode: CTC top=OCR1A
  // OC1A output: Disconnected
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 5 ms
  // Timer1 Overflow Interrupt: Off
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: On
  // Compare B Match Interrupt: Off
  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x00;
  ICR1L=0x00;
  OCR1AH=0x00;
  OCR1AL=0x4E;
  OCR1BH=0x00;
  OCR1BL=0x00;
  
  // Timer/Counter 1 Interrupt(s) initialization
  TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

  sei();
}

ISR (TIMER1_COMPA_vect){
  timer_intr=1;
}

float readFromMATLAB() {
  int reln = Serial.readBytesUntil('\r\n', buff, buffer_size);
  for (int i=0; i<buffer_size; i++) {
    myValue.bytes[i] = buff[i];
  }
  float out = myValue.number;
  return out;
}

void writeToMATLAB(float num) {
  byte *b = (byte *) &num;
  Serial.write(b, 4);
  Serial.write(13);
  Serial.write(10);
}

void setup() {
  RegSet();
  // initialize serial, use the same baudrate in the Simulink Config block
  Serial.begin(115200);
  SetTuningsOwn(P,I,D);
  SetModeOwn(true);
}

void loop(){
  if (Serial.available() > 0) {
    input = readFromMATLAB();
    filt_input = (0.73310279*last_filt_input) + (0.1334486*input) + (0.1334486*lastinput);
    
    if(timer_intr == 1){
      PIDCompute();
    }
    timer_intr = 0;
    writeToMATLAB(output);
  }
}
