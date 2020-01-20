#include "XWheels.hpp"
#include "mbed.h"
// VEHICLE CONTROL , UGV DRIVE

#define MIN_STICK 360       
#define MAX_STICK 1673      
#define MIN_DEADBAND 1014
#define MAX_DEADBAND 1034
#define MID_STICK 1024
#define DIVIDER 2           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

float MAX_RPM = 144.0;         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
float ZERO_RPM = 0.0;          // No speed

//RawSerial uart(PD_1,PD_0,9600);


XWheels::XWheels(RawSerial *uart)
{   
    _uart = uart;
    
    startTick = true;
    ReadOK = false;
    Reply[0] = 1;
    Reply[1] = 1;
    Reply[2] = 1;
    Reply[3] = 1;
    Reply[4] = 1;
    Reply[5] = 1;

    // Sending Command
    Header1 = 0x02;
    Header2 = 0x09;

    // Initialized of 17 bytes
    InitHeader1 = 0x01;     // always constant
    InitHeader2 = 0x11;     // always constant
    ForwardAcc = 0x32;      // 0x00 to 0x64   [0-100]   An acceleration when changing speed value
    ForwardDelay = 0x00;    // 0x00 t0 0x05   [0-5]     A delay before start to go
    BrakeDis = 0x0A;        // 0x00 to 0x64   [0-100]   Brake distance, should be as short as possible (rigid brake)
    TurnAcc = 0x32;         // 0x00 to 0x64   [0-100]   Turning acceleration, when two wheels has reverse direction
    TurnDelay = 0x01;       // 0x00 t0 0x05   [0-5]     A delay before start turning
    AccTimeOfStart = 0x00;  // 0x00 to 0x32   [0-50]    increase this will make wheels slower
    SenRocker = 0x83;       // Don't need to change     this is for curving motion, we have our own calculation.
    UnderVolt1 = 0x14;      // 0x12 -> 18.0V, 0x13 -> 19.0V, 0x14 -> 20.0V, 0x15 -> 21.0V, 0x16 -> 22.0V
    UnderVolt2 = 0x05;      // 0x01 to 0x09 -> 0.1V tp 0.9V
    StartSpeed = 0x0A;      // 0x00 to 0x64   [0-100]   starting speed if too high you will hear some cogging sound out from gear, set not too high
    DriveMode = 0x01;       // 0x01 is Sine wave, 0x00 is square wave. Don't need to change, square wave seems not working well...
    PhaseLMotor = 0x03;     // Don't need to change     not sure what is this, so leave it alone 
    PhaseRMotor = 0x04;     // Don't need to change     not sure what is this, so leave it alone
    MotorConfig = 0x07;     // Don't need to change     This is about choosing which wheel will be reverse
    InitCheckSum = InitHeader1 + InitHeader2 + ForwardAcc + ForwardDelay + BrakeDis + TurnAcc + TurnDelay + AccTimeOfStart + SenRocker 
                    + UnderVolt1 + UnderVolt2 + StartSpeed + DriveMode + PhaseLMotor + PhaseRMotor + MotorConfig;
    
}

int XWheels::Init()
{
    //pc.printf("Initialized OK...\n");
    waitUntilFourZero();
    wait_ms(219);
    ESCHandShake();
    for(i=1;i<10;i++)
    {
        zeroSpeed();
    }
    return 1;
}

void XWheels::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
  // unsigned int can store 16 bit int 
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
}

long XWheels::map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float XWheels::IntToFloat(int intValue)
{
    // Convert positive number from 0.01 to 144.00
    if (intValue <= 14400 && intValue >= 0)
    {
        return intValue/100.00;

    }
    // Convert Negative number from -0.01 to -144.00
    else if(intValue >= 51136 && intValue <= 65535)
    {
        return -(65536 - intValue)/100.0;
    }
    // In case more than 144.0 a bit 
    else if (intValue >= 14401 && intValue < 30000)
    {
        return 144.00;
    }
    else
    {   // intValue is out of range
        return 0.0;
    }
}

unsigned int XWheels::RPMToRaw(float rpm)
{
  int raw_int;
  unsigned int out_raw;


    // map rpm to raw value
    raw_int = (int)map(rpm, 0.0, 144.0, 0, 3200);   // for linear approximate, but low RPM is not collect
 
  
    // In case of negative number, shift mapped number from 32768 to 35968 (for 0.0 to -146.0)
    if (rpm < 0.0)
    {
      out_raw = 32768 + abs(raw_int);
      }
    // In case of positive number, use the mapped number for unsigned int type
    else
    {
      out_raw = raw_int;
      }
  
  return out_raw;
}

unsigned int XWheels::RPMToRaw2(float rpm)
{
  int raw_int;
  unsigned int out_raw;

    // First linear 
    if (rpm >= 0.0 && rpm < 40.0)
    {
        // map rpm to raw value
        raw_int = (int)map(rpm, 0.0, 39.0, 70, 866);
    }
    else if(rpm < 0.0 && rpm > -40.0)
    {
        raw_int = (int)map(rpm, 0.0, -39.0, 32845, 33634); 
    }
    else if(rpm >= 40.0)
    {
        raw_int = (int)map(rpm, 40.0, 144.0, 888, 3200); 
    }
    else if(rpm <= -40.0)
    {
        raw_int = (int)map(rpm, -40.0, -144.0, 33656, 35968); 
        //out_raw = 32768 + abs(raw_int);
    }

    out_raw = raw_int;
   
    
  return out_raw;
}

void XWheels::waitUntilFourZero()
{
    while (startTick)
    {
        //t.start();
        //pc.printf("Readable : %d\n",_uart->readable());
        //t.stop();
        //pc.printf("Time: %f seconds\n", t.read());
        while (_uart->readable() == true) {
            char ReadByte = _uart->getc();
            Reply[i] = ReadByte;
            //t.start();
            //pc.printf("ReadByte: %X\n", ReadByte);     // print whole string
            //t.stop();
            //pc.printf("Time: %f seconds\n", t.read());
            i++;
            ReadOK = true;
            wait_us(1000);  // DONT CHANGE THIS DELAY   This delay act as "pc.printf("ReadByte: %X\n", ReadByte);"
            }

        if (ReadOK == true){
            
            if ((Reply[0] == 0) && (Reply[1] == 0) && (Reply[2] == 0) && (Reply[3] == 0)){
                startTick = false;
                //pc.printf("Ready to drive...\n");
            }
            
            // reset value
            i = 0;
            ReadOK = false;

            }
            wait_ms(1);    // wait_ms(15); DONT CHANGE THIS DELAY   This delay act as "pc.printf("Readable : %d\n",_uart->readable());"
                           // wait_ms(1) mimic all of the printf behavior and it works
    }
}

void XWheels::ESCHandShake()
{
    printf("InitCheckSum %X\n", InitCheckSum);
    for(int k=1;k<=20;k++)
    {   
        // This is like initial setup for the ESC
        _uart->putc(InitHeader1);
        _uart->putc(InitHeader2);
        _uart->putc(ForwardAcc);
        _uart->putc(ForwardDelay);
        _uart->putc(BrakeDis);
        _uart->putc(TurnAcc);
        _uart->putc(TurnDelay);
        _uart->putc(AccTimeOfStart);
        _uart->putc(SenRocker);
        _uart->putc(UnderVolt1);
        _uart->putc(UnderVolt2);
        _uart->putc(StartSpeed);
        _uart->putc(DriveMode);
        _uart->putc(PhaseLMotor);
        _uart->putc(PhaseRMotor);
        _uart->putc(MotorConfig);
        _uart->putc(InitCheckSum);

        if (k==1){
            wait_us(300);
        }
        else{
            wait_ms(14);
            
        }
    }
}

void XWheels::zeroSpeed()
{
    _uart->putc(Header1);
    _uart->putc(Header2);
    _uart->putc(0x00);            // Motor1 speed hibyte
    _uart->putc(0x00);            // Motor1 speed lobyte
    _uart->putc(0x00);            // Motor2 speed hibyte
    _uart->putc(0x00);            // Motor2 speed lobyte
    _uart->putc(0xB4);            // Mode hibyte (don't care)
    _uart->putc(0x00);            // Mode lobyte (don't care)
    _uart->putc(0xBF);            // Check sum
    wait_ms(23);

}

void XWheels::DriveWheels(float rpm1, float rpm2)
{  
    float Out_RPM_Right;
    float Out_RPM_Left;
    unsigned char Motor1SpeedByte[2];
    unsigned char Motor2SpeedByte[2];

    //printf("Out_Right %f\n", Out_RPM_Right);
    //printf("Out_Left %f\n", Out_RPM_Left);
    
    Out_RPM_Right = rpm1;
    Out_RPM_Left = rpm2;
    
    RawInt1 = RPMToRaw2(Out_RPM_Right);
    RawInt2 = RPMToRaw2(Out_RPM_Left);
    Int16ToByteData(RawInt1,Motor1SpeedByte);
    Int16ToByteData(RawInt2,Motor2SpeedByte);

    unsigned char Motor1hibyte = Motor1SpeedByte[0];
    unsigned char Motor1lobyte = Motor1SpeedByte[1];

    unsigned char Motor2hibyte = Motor2SpeedByte[0];
    unsigned char Motor2lobyte = Motor2SpeedByte[1];

    unsigned char Modehibyte = 0x00;   
    unsigned char Modelobyte = 0x00;

    unsigned char CheckSum = Header1 + Header2 + Motor1hibyte + Motor1lobyte + Motor2hibyte + Motor2lobyte + Modehibyte + Modelobyte;

    _uart->putc(Header1);
    _uart->putc(Header2);
    _uart->putc(Motor1hibyte);
    _uart->putc(Motor1lobyte);
    _uart->putc(Motor2hibyte);
    _uart->putc(Motor2lobyte);
    _uart->putc(Modehibyte);
    _uart->putc(Modelobyte);
    _uart->putc(CheckSum);

    wait_ms(23);                      // DON'T change this delay, it's from hacking

  
}

void XWheels::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;
    

    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND)
    {
        MotorRPM[0] = 0.0;
        MotorRPM[1] = 0.0;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && (UD_ch > MAX_DEADBAND || UD_ch < MIN_DEADBAND))
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorRPM[1] = (float)map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM/2, MAX_RPM/2);
        MotorRPM[0] = -MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }  
   
}