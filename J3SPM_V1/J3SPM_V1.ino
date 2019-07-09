//ADS1256 Pin connection.
// ADC channel (0~6), Sig2ADC : pin 7=> 2.5V (ref)
// ch0 => FE-P(amplified & offset), ch1=>VTO (ori vtotal or offset on Jumper set),
// ch2=>FE_A(amplified), ch3=>RMSDC, ch4=> Phase
// ch5=> FE, ch6=> Vtotal,

//DAC8554, ch0: x axis, ch1: y axis, ch2: z axis, ch3: OSC(function generator)

//DRDY: Pin 49 on Arduino Mega  (ADS1256.h)
//CS: Pin 36 on Arduino Mega  : modified library.
//RESET: MCU (3.3V)

#define FNC_PIN 37 
#include <AD9833.h>
#include <J3SPMDAQ.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.5; // voltage reference
// Define ADS1256 PIN that is connected to Arduino
J3SPMDAQ daq(clockMHZ, vRef, false );
// Function generator
AD9833 gen(FNC_PIN); 

float FE, Fld_FE, TOT, AMP, PHA, contTresh = 0.01, Setpoint;
double Pgain = 0, Igain = 0, Dgain = 0;

uint16_t Output,zv_offset = 0, Pgaini, Igaini, Dgaini,OutputM;

double  dvx = 0, dvy = 0, dx = 0, dy = 0;
volatile uint16_t PID_V[270];
uint16_t stepX, stepY, stepZ;
uint16_t ResX = 256, ResY = 256;
uint16_t i, j, zt, dz,Z_freq=2000,xcon_rate=2000;

double Outputf,OutputfM, PreError=0, I_OUT=0,  I_OUTM=0, I_OUTmax=1;
volatile boolean scanflag = false, zconflag = false;
volatile float Error;

boolean FEsig, ACON,  LowZoff, CONTACT = false, TOTON = false, FE_T=true;
boolean LSCANDONE = false, YSTEPDONE = false, RETRDONE = false, ORI = false, BSCANDONE = true;
char Str_buf[2000], Str_buf2[30];

char COM = 'E', b[150], flag = 'X';
String read_rx;

int FE_t = 0, SLOPE_t = 0, ACON_t = 0, range = 0, Lat = 0, Height = 0, ampl=0, PHON_d=0;
int yindex;

int scan_duration_t = 0 ;
int save_index = 0, save_period = 0;
int set_point_t = 0;
int scan_lines = 0;
int length2 = 0, jj, ch0 = 0, ch1 = 1;
boolean SET_ACH0=false, SET_ACH1=false, SET_ACH3=false, Data_write=true, AD98ON=false, 
DAQINIT=false, FSET=true, OFF_FUNC=false, PHON=false;
const int DPCS = 38;
float fset_freq;
long int dFI, Start_freqI, End_freqI, tempFreq,fset_freqI;

ISR(TIMER1_COMPA_vect)   //for scanner (2kHz)
{
  scanflag = true;
}
ISR(TIMER3_COMPA_vect)  //for z controller. (2.5Hz)
{
  zconflag = true;
}

void setup() {
  Serial.begin(115200);
  MCP41010Write(2);
  gen.Begin();
  gen.EnableOutput(false);
  
  daq.sendCommand(SDATAC);      
  Serial.println("SDATAC command sent");
  Serial.println("Starting ADC");
  

  daq.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_1, false);  

  daq.setDAC(0);
  daq.setPins(52, 53, 51, -1, -1, -1, -1);
  daq.initializePins();
  daq.updateAllChannels(0);
  
  daq.updateChannel(0, 0);
  daq.updateChannel(1, 0);
  daq.updateChannel(2, 0);
  daq.updateChannel(3, 0);


  Serial.println("ADC Started");
  cli();

  // OCR#A value = (16*10^6) / (sampling rate*256) - 1 (must be <65536)   30: 2kHz, 60: 1kHz, 82:750Hz, 124:500Hz

  //set timer1 interrupt at 2kHz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 30;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);// | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  //set timer3 interrupt at 2kHz
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A =124;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31); TCCR3B |= (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);

  FEsig = true;   LowZoff = true;   TOTON = false;   ACON = false;
  stepZ = 200; 
  pinMode (DPCS, OUTPUT);
  sei();
}

void loop() {

  while (Serial.available()) {
    read_rx = Serial.readString(); // read the incoming data as string
    read_rx.toCharArray(b, 100);
    DAQINIT=false;AD98ON=false;FSET=true;
    SET_ACH0=false; SET_ACH1=false;SET_ACH3=false;OFF_FUNC=false;
    
    if (read_rx.length() >= 40)
    {
      sscanf(b, "[%s %01d;%01d;%01d;%02d;%04d;%02d;%05d;%05d;%03d;%04d;%04d;%04d;%01d;%01d;%05d;%03d;%05d;%05d;%05d;%02d", &COM, &FE_t, &SLOPE_t, &ACON_t, &range, &scan_lines, 
      &scan_duration_t, &zv_offset, &set_point_t, &ResX, &Pgaini, &Igaini, &Dgaini, &Lat, &Height, &fset_freqI, &ampl,&dFI, &Start_freqI,&End_freqI,&PHON_d);
      ACON = ACON_t;FE_T=FE_t;fset_freq=fset_freqI;PHON=PHON_d;
      
      Setpoint = set_point_t * 0.0001;
      Pgain = (double)Pgaini * 0.001; Igain = (double)Igaini * 0.1; Dgain = (double)Dgaini * 0.001;
      
      if (ACON) {    
        FEsig = true; FE_T = false; TOTON = false;
      }
      else {
        FEsig = SLOPE_t;
        if (FE_T) TOTON = false;
        else TOTON = true;
      }
      if (Lat == 0)
      {
        ch0 = 0;
        ch1 = 1;
      }
      else
      {
        ch0 = 1;
        ch1 = 0;
      }

      dvx = ((131.07 * (double)(range))) / (((double)(scan_duration_t) * (double) (xcon_rate)*0.01)); 
      dvy = (((1310.7 * (double)(range)) / ((double)(scan_lines)))) * 10;
      save_period = (scan_duration_t) * Z_freq / ResX; //  ms, or 512, control rate(z axis)(?)
      daq.updateChannel(ch0, 0);
      daq.updateChannel(ch1, 0);

    }
    
    else if (read_rx.length() >= 2)
    {
     sscanf(b, "%s", &COM);
    }

   }

  if (COM == 'S') //Scanning mode
  {
    if(ACON){
      gen.Begin();
      gen.ApplySignal(SINE_WAVE,REG0,fset_freqI*10);
      MCP41010Write((byte) ampl);
      gen.EnableOutput(true);
      FSET=false;
      delay(10);
    }
    
    if(!DAQINIT){ 
      SPI.beginTransaction(SPISettings(7.68 * 1000000 / 4, MSBFIRST, SPI_MODE1));
      daq.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_1, false); 
      DAQINIT=true;
     }
      
    CONTACT = true; 
    for (yindex = 0; yindex < scan_lines + 1; yindex++)
    {
      
      save_index = 0;
      dz=0;

      LSCANDONE = false;
      Data_write=true;
      while (!LSCANDONE)
      {
        z_controller();  
        forward_scan();
      }
      OutputM=Output;
      Send_Ldata(ResX);

      Data_write=false;
      BSCANDONE = false;
      while (!(BSCANDONE))
      {
        z_controller();
        backward_scan();
      }

      YSTEPDONE = false;
      while (!YSTEPDONE)
      {
        step_motion_y();
      }
    }
    ORI = false;
    while (!ORI)
    {
      origin();             
    }
    COM = 'E';              
  }
  
  else if (COM == 'M')   //Monitoring mode
  {
    if(!OFF_FUNC){
    MCP41010Write(2);
    gen.EnableOutput(false);
    OFF_FUNC=true;}
      
    if (ACON) {
      if(!AD98ON) {
      gen.Begin();
      gen.ApplySignal(SINE_WAVE,REG0,fset_freqI*10);
      AD98ON=true;
      MCP41010Write((byte) ampl);
      gen.EnableOutput(true);
      ACON=true;}
        
      if(!DAQINIT){ 
        SPI.beginTransaction(SPISettings(7.68 * 1000000 / 4, MSBFIRST, SPI_MODE1));
        daq.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_1, true); 
       }
      FE_T = false; TOTON = false;
      read_adc(true,false,false);
      PHA=fset_freqI;
    }
    else {
      if(!DAQINIT){ 
        SPI.beginTransaction(SPISettings(7.68 * 1000000 / 4, MSBFIRST, SPI_MODE1));
        daq.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_1, true); }
      FE_T = true;  TOTON = true;
      read_adc(false,true,true);
    }
    daq.updateChannel(2,zv_offset);
    DAQINIT=true;
    delay(85);
    Send_Mdata();
 }
  
  else if (COM == 'F')  //Frequency sweeping
  {
      
    if(!AD98ON) {
      gen.Begin();
      gen.ApplySignal(SINE_WAVE,REG0,Start_freqI*10);
      AD98ON=true;
      MCP41010Write((byte) ampl);
      gen.EnableOutput(true);
      ACON=true;}
      
      for(int level=0;level<(End_freqI-Start_freqI)/dFI;level++)
      {
        MCP41010Write((byte) ampl);
        gen.IncrementFrequency(REG0,dFI*10);
        tempFreq=(Start_freqI+dFI*level)*10;
        delay(100);
        
        SPI.beginTransaction(SPISettings(7.68 * 1000000 / 4, MSBFIRST, SPI_MODE1));
        daq.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_1, false);
        if(PHON)
        {
          daq.setChannel(3);  daq.setChannel(3);
          delay(10);
          daq.waitDRDY();
          
          AMP = daq.readChannel();
          daq.setChannel(4);daq.setChannel(4);
          delay(10);
          daq.waitDRDY();
          PHA = daq.readChannel();  
        }
        else
        {
          read_adc(true,false,false);
          PHA=tempFreq/10;
        }
                
        delay(35);
        Send_Mdata();
      }
      COM='E';
   }

 else if (COM == 'E')  //End scanning
  {
   if(!OFF_FUNC){
    MCP41010Write(2);
    gen.EnableOutput(false);
    OFF_FUNC=true;
   }
  }

}

void z_controller()
{
  if (CONTACT)
  {
    if (zconflag)
    {
      control();
      zt += 1;
      zconflag = false; 
    }
    if (zt >= save_period)
    {
      if (Height == 0) PID_V[save_index] = Output;
      else {
            if (FE_T) PID_V[save_index] = FE * 10000;
            else PID_V[save_index] = TOT * 10000;
           }
      save_index += 1;
      zt = 0;
      if (save_index >= ResX) save_index = ResX;
     }
   }
}

void control() {
  z_control();
}

void z_control() {
   read_adc(ACON,FE_T,TOTON);
   if(Height == 1) daq.updateChannel(2, zv_offset);
   else{
    if(ACON) Error = (double) Setpoint-(double) AMP;
    else {
     if(FE_T) Error = (double) Setpoint-(double) FE ;
     else Error = (double) Setpoint-(double)TOT ;

     I_OUT = I_OUT + Error/(Z_freq);
    }
 
    if(I_OUT > I_OUTmax) I_OUT = I_OUTmax;
    else if(I_OUT < I_OUTmax * -1.0) I_OUT = I_OUTmax * -1.0;

    Outputf = Pgain * Error + Igain*I_OUT + Dgain * (Error - PreError);

    if (FEsig){
    Outputf = Outputf * -1.0;
    }
    Output = (Outputf *13107.0) + zv_offset;  
    if (Output >= 65000) Output = 65000;
    else if (Output <= 100) Output = 100;
  
    PreError = Error;
    daq.updateChannel(2, Output);
    }
}

void read_adc(boolean A,boolean F,boolean T) {
  if (A){
    if(!SET_ACH3){daq.setChannel(3);daq.setChannel(3);daq.setChannel(3);SET_ACH3=true;}
    daq.waitDRDY();
    AMP = daq.readChannel();
  }
  else {
    if(COM=='M'){
      daq.setChannel(0);
      daq.waitDRDY(); 
      daq.setChannel(1);
      FE = daq.readChannel();
      daq.waitDRDY(); 
      TOT = daq.readChannel();      
      }
    else {
      if (FE_T) {
        if(!SET_ACH0){daq.setChannel(0);SET_ACH0=true;}
        daq.waitDRDY(); // wait for DRDY to go low before changing multiplexer register
        FE = daq.readChannel();
      }
      else {
        if(!SET_ACH1){daq.setChannel(1); SET_ACH1=true;}
        daq.waitDRDY();
        TOT = daq.readChannel();
    }
  }
 }
}

void forward_scan()
{
  if (scanflag)
  {

    daq.updateChannel(ch0, (unsigned int)(dx * dvx));
    dx += 1;
      
    if (dx >= xcon_rate * scan_duration_t)
    {
      dx = 0;
      LSCANDONE = true;
    }
   scanflag = false;
  }
}

void retreat_z(uint8_t move_zstep)
{
  unsigned int temp = 1;
  if (zconflag)
  {
    temp = Output - (dz * move_zstep);
    daq.updateChannel(2, temp);
    dz += 1;
    zconflag = false;
  }
  if (dz >= 1000 || temp <= zv_offset)
  {
    CONTACT = false;
    YSTEPDONE = false;
    RETRDONE = true;
  }
}

void step_motion_y()
{
  if (scanflag)
  {
    dy += 1;
    daq.updateChannel(ch1, dvy / 201 * dy + yindex * dvy);
    scanflag = false;
  }
  if (dy > 201)
  {
    dy = 0;
    BSCANDONE = false;
    YSTEPDONE = true;
  }

}

void backward_scan()
{
  unsigned int temp = 1;
 
  if (scanflag)
  {
    temp = (float)(13107 * (float)(range)) - (dx * dvx);
    daq.updateChannel(ch0, temp);
    dx += 1;            
    scanflag = false;
  }
  if ((dx > scan_duration_t * xcon_rate) || temp <= 0)
  {
    dx = 0;
    BSCANDONE = true;
  }
}

void contacting()
{
  if (zconflag)
  {
     if (ACON) daq.setChannel(3);
     else{ if (FE_t)  daq.setChannel(0);
          else daq.setChannel(1);}
    
      daq.waitDRDY(); 
 
      if (Height == 1) {
        if (dz * stepZ >= zv_offset+50)
        {
          CONTACT = true;
          dz = 0;
          LSCANDONE = false;
        }}
     else{ 
      if(ACON){
         daq.setChannel(3);
         AMP = daq.readChannel();
         Error = (double) Setpoint-(double) AMP;           
       }
      else{
         if(FE_T){
           daq.setChannel(0);
           FE = daq.readChannel();
           Error = (double) Setpoint-(double) FE ;  
          }
         else {
           daq.setChannel(1);
           TOT = daq.readChannel();
           Error = (double) Setpoint-(double)TOT ;  
          }
         }
       }
      I_OUTM = I_OUTM + Error/(Z_freq);
      OutputfM = Pgain * Error + Igain*I_OUTM + Dgain * (Error - PreError);

      if (FEsig)
      {
        OutputfM = OutputfM * -1.0;
      }
 
      OutputM = (OutputfM *13107.0) + zv_offset;  
      if (OutputM >= 65000) Output = 65000;
      else if (OutputM <= 100) OutputM = 100;
      
      PreError = Error;
      daq.updateChannel(2, OutputM);
      if((fabs(Error)<=0.001)||(dz>=5000)) {CONTACT=true; LSCANDONE=false;dz=0;} 
      dz+=1;
      zconflag = false;
  }
}

void origin()
{
  if (scanflag)
  {
    daq.updateChannel(ch0, 0);
    if ((yindex - dy/200) <= 0)
    {
      ORI = true;
      yindex = dy = 0;
      daq.updateChannel(ch1,0);
    }
     else
     {daq.updateChannel(ch1, dvy * (yindex - dy/200)); }

    scanflag = false;
    dy += 1;
  }
}

void Send_Ldata(int pixels)
{
  length2 = 0;
  for (jj = 0; jj < pixels - 1; jj++) {
    length2 += sprintf(Str_buf + length2, "%6ld", PID_V[jj]);
  }
  length2 += sprintf(Str_buf + length2, "%6ld\r", PID_V[pixels - 1]);
  Serial.println(Str_buf);
}

void Send_Mdata()
{
  if (ACON) sprintf(Str_buf2, "%5ld %5ld\r", (long int)(AMP * 10000), (long int)(PHA * 10000));
  else sprintf(Str_buf2, "%5ld %5ld\r", (long int)(FE * 10000), (long int)(TOT * 10000));
  Serial.println(Str_buf2);
}

void MCP41010Write(byte value) 
{
  SPI.beginTransaction(SPISettings(7.68 * 1000000 / 4, MSBFIRST, SPI_MODE0));
  digitalWrite(DPCS,LOW);
  SPI.transfer(B00010001); 
  SPI.transfer(value);     
  digitalWrite(DPCS,HIGH); 
}
