// microHomePowerMonitor
// Sistema di controllo carichi elettrici per domotica
// con visualizzazione su display OLED 0.94"
// By Foini Giorgio Ottobre 2015

#include <Wire.h>					// Richiama la libreria per il protocollo di comunicazione I2C
#include <EmonLib.h>				// Richiama la libreria di gestione dei TA


#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

int SCL_PIN=6;						// Assegnazione pin SCL display (pin D0 board OLED)
int SDA_PIN=5;						// Assegnazione pin SDA display (pin D1 board OLED)
int RST_PIN=8;						// Assegnazione pin RESET display
int DC_PIN=7;						// Assegnazione pin DC display
const int TATotali = A0;			// Assegnazione pin per TA linea totale 
const int TABox = A2;				// Assegnazione pin per TA box-lavanderia

char StringaOLED[6];				// Stringa di visualizzazione valori su OLED

int Digit1, Digit2, Digit3;			// 3 Digit per la visualizzazione di Kw e I su OLED
int OLEDDigit;						// Variabile di appoggio da passare in stringa su OLED
int PercentualePotenza, PercPot;	// Gestione percentuale potenza
int Ciclo, FiltroON, FiltroOFF;		// Filtri di ritardo per attivazione-spegnimento allarmi
int Refresh;						// Pulizia totale OLED antiburn pixel
const int Volts = 220;				// Costante di tensione per il calcolo dei KW 
const int CalTotali = 22;			// Costante di calibrazione TA Linea
const int CalBox = 22;				// Costante di calibrazione TA Box - Lavanderia
float ITotali = 0;					// Corrente misurata di linea
float IBox = 0;						// Corrente misurata box -lavanderia
float KWTotali = 0;					// KWatt di linea
float KWBox = 0;				    // KWatt box - lavanderia

EnergyMonitor TA1;					// Abbina istanza di controllo consumi a variabile TA1
EnergyMonitor TA2;					// Abbina istanza di controllo consumi a variabile TA2

const unsigned char Allarme [] = {	// Bitmap segnalazione superamento soglia massima consumi
0x00, 0x00, 0xFF, 0xFF, 0x06, 0x06, 0x0C, 0x08, 0x18, 0x10, 0x30, 0x60, 0x40, 0xC0, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81,
0x83, 0x83, 0x86, 0x84, 0x8C, 0x08, 0x18, 0x30, 0x30, 0x60, 0x40, 0xC0, 0x80, 0x80, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x81,
0xC1, 0xC1, 0x61, 0x21, 0x31, 0x10, 0x18, 0x0C, 0x0C, 0x06, 0x02, 0x03, 0x01, 0x01, 0x00, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0x60, 0x60, 0x30, 0x10, 0x18, 0x08, 0x0C, 0x06, 0x02, 0x03, 0x01, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Lampadina [] = { // Bitmap segnalazione luce accesa
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x20, 0xE0, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0x20, 0xE0, 0x00, 0xF0, 0x00, 0xE0, 0xA0, 0xB0, 0xB0, 0x98, 0x88, 0xCE, 0xC2, 0xE2,
0x02, 0x03, 0x03, 0x03, 0x02, 0x02, 0x06, 0x0E, 0x18, 0xFB, 0x01, 0x80, 0x80, 0x80, 0x80, 0x80,
0x00, 0x07, 0x00, 0x0F, 0x0C, 0x0F, 0x0E, 0x0B, 0x0F, 0x0E, 0x1A, 0x3A, 0x32, 0xE2, 0xC3, 0x87,
0x80, 0x80, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0x60, 0x38, 0xBF, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0E, 0x07, 0x03, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Lavatrice [] = { // Bitmap segnalazione lavanderia in ciclo
0xF8, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x88, 0x88, 0x88, 0xC8, 0xC8, 0xC8, 0xC8, 0x88,
0x88, 0x88, 0x08, 0x08, 0x08, 0x08, 0x08, 0x18, 0xF8, 0xF8, 0x18, 0x08, 0x08, 0x08, 0x08, 0xF8,
0xFF, 0x00, 0x00, 0x00, 0xF0, 0x3C, 0x0E, 0x03, 0x01, 0x01, 0x00, 0x80, 0xC0, 0xF0, 0xFC, 0xFF,
0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
0xFF, 0x00, 0x00, 0x00, 0x0F, 0x1E, 0x70, 0x60, 0xC0, 0x80, 0x8E, 0xBF, 0xBF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x03, 0x00, 0xFF, 0xFF, 0x00, 0x9C, 0x9C, 0x88, 0x00, 0xFF,
0x1F, 0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
0x11, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x18, 0x1F, 0x1F, 0x18, 0x13, 0x13, 0x11, 0x18, 0x1F
};

const unsigned char Vuoto [] = {	 // Bitmap per cancellazione bitmap precedenti
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Set di caratteri 8x6. Solo quelli utilizzati sono stati tradotti per la visualizzazione verticale
const unsigned char F8x6[][8] =	
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00 },   // 0sp
  { 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f },   // 1# Striscia piena
  { 0x11, 0x29, 0x12, 0x04, 0x08, 0x12, 0x25, 0x22 },   // 2%
  { 0x00, 0x00, 0x00, 0x00, 0x0c, 0x12, 0x12, 0x0c},   // 3Gradi Celsius
  { 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // 4.
  { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },   // 50
  { 0x1e, 0x08, 0x08, 0x08, 0x08, 0x0A, 0x0c, 0x08 },   // 61
  { 0x1f, 0x02, 0x04, 0x08, 0x10, 0x11, 0x11, 0x0e },   // 72
  { 0x0e, 0x11, 0x10, 0x0c, 0x0c, 0x10, 0x11, 0x0e },   // 83
  { 0x1c, 0x08, 0x1f, 0x09, 0x0a, 0x0a, 0x0c, 0x0c },   // 94
  { 0x0e, 0x11, 0x11, 0x10, 0x0f, 0x01, 0x01, 0x1f },   // 105
  { 0x0e, 0x11, 0x11, 0x11, 0x0f, 0x01, 0x11, 0x0e },   // 116
  { 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x11, 0x1f },   // 127
  { 0x0e, 0x11, 0x11, 0x0e, 0x0e, 0x11, 0x11, 0x0e },   // 138
  { 0x0e, 0x11, 0x10, 0x1e, 0x11, 0x11, 0x11, 0x0e },   // 149
  { 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 },   // 15:
  { 0x00, 0x00, 0x1f, 0x00, 0x00, 0x1f,0x00, 0x00 },   // 16=
  { 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x0a, 0x04 },   // 17A
  { 0x0f, 0x11, 0x11, 0x0f, 0x0f, 0x11, 0x11, 0x0f },   // 18B
  { 0x0e, 0x11, 0x01, 0x01, 0x01, 0x01, 0x11, 0x0e },   // 19C
  { 0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e },   // 20I
  { 0x22, 0x12, 0x0a, 0x06, 0x06, 0x0a, 0x12, 0x22 },   // 21K
  { 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 },   // 22L
  { 0x11, 0x19, 0x19, 0x15, 0x15, 0x13, 0x13, 0x11 },   // 23N
  { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f },   // 24T
  { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 },   // 25U
  { 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00 },   // 26_
  { 0x0a, 0x15, 0x15, 0x11, 0x11, 0x00, 0x00, 0x00 },   // 27w
};

// GESTIONE OLED
void LEDPIN_Init(void) // Inizializzazione pin della board OLED    
{
  pinMode(SCL_PIN,OUTPUT);
  pinMode(SDA_PIN,OUTPUT);
  pinMode(RST_PIN,OUTPUT);
  pinMode(DC_PIN,OUTPUT);
}
void LED_WrDat(unsigned char data)   // Scrittura dati su board OLED
{
  unsigned char i = 8;
          digitalWrite(DC_PIN,HIGH);
          digitalWrite(SCL_PIN,LOW);
  while (i--)
  {
    if (data & 0x80)
    {
      digitalWrite(SDA_PIN,HIGH);;;;
    }
    else
    {
      digitalWrite(SDA_PIN,LOW);;;
    }
                digitalWrite(SCL_PIN,HIGH);;;
    asm("nop");;;     
                digitalWrite(SCL_PIN,LOW);
    data <<= 1;    
  }
}
void LED_WrCmd(unsigned char cmd) // Invio comandi a board OLED
{
  unsigned char i = 8;
    digitalWrite(DC_PIN,LOW);;;
    digitalWrite(SCL_PIN,LOW);;;
         while (i--)
  {
    if (cmd & 0x80)
    {
      digitalWrite(SDA_PIN,HIGH);;;
    }
    else
    {
      digitalWrite(SDA_PIN,LOW);;;
    }
    digitalWrite(SCL_PIN,HIGH);;;
                asm("nop");;;           
    digitalWrite(SCL_PIN,LOW);;;
    cmd <<= 1;   
  }   
}
void LED_Set_Pos(unsigned char x, unsigned char y) // Posizionamento cursore su display OLED
{ 
  LED_WrCmd(0xb0+y);
  LED_WrCmd(((x&0xf0)>>4)|0x10);
  LED_WrCmd((x&0x0f)|0x00); 
} 


void LED_Fill(unsigned char bmp_data) // Cancellazione totali pixel attivi su display OLED
{
  unsigned char y,x;
  
  for(y=0;y<8;y++)
  {
    LED_WrCmd(0xb0+y);  
    LED_WrCmd(0x00);     
    LED_WrCmd(0x10);       
    for(x=0;x<128;x++)
      LED_WrDat(bmp_data);
  }
} 
void LED_CLS(void) 
{
  unsigned char y,x;  
  for(y=0;y<8;y++)
  {
    LED_WrCmd(0xb0+y);
    LED_WrCmd(0x00);
    LED_WrCmd(0x10); 
    for(x=0;x<128;x++)
      LED_WrDat(0);
  }
}

void LED_DLY_ms(unsigned int ms) // Ritardo refresh dati su display OLED
{                         
  unsigned int a;
  while(ms)
  {
    a=6675;
    while(a--);
    ms--;
  }
  return;
}

void SetStartColumn(unsigned char d)
{
  LED_WrCmd(0x00+d%16); // Set Lower Column Start Address for Page Addressing Mode
              // Default => 0x00
  LED_WrCmd(0x10+d/16); // Set Higher Column Start Address for Page Addressing Mode
              // Default => 0x10
}

void SetAddressingMode(unsigned char d)
{
  LED_WrCmd(0x20);      // Set Memory Addressing Mode
  LED_WrCmd(d);     // Default => 0x02
              // 0x00 => Horizontal Addressing Mode
              // 0x01 => Vertical Addressing Mode
              // 0x02 => Page Addressing Mode
}

void SetColumnAddress(unsigned char a, unsigned char b)
{
  LED_WrCmd(0x21);      // Set Column Address
  LED_WrCmd(a);     // Default => 0x00 (Column Start Address)
  LED_WrCmd(b);     // Default => 0x7F (Column End Address)
}

void SetPageAddress(unsigned char a, unsigned char b)
{
  LED_WrCmd(0x22);      // Set Page Address
  LED_WrCmd(a);     // Default => 0x00 (Page Start Address)
  LED_WrCmd(b);     // Default => 0x07 (Page End Address)
}

void SetStartLine(unsigned char d)
{
  LED_WrCmd(0x40|d);    // Set Display Start Line
              // Default => 0x40 (0x00)
}

void Set_Charge_Pump(unsigned char d)
{
  LED_WrCmd(0x8D);      // Set Charge Pump
  LED_WrCmd(0x10|d);    // Default => 0x10
              // 0x10 (0x00) => Disable Charge Pump
              // 0x14 (0x04) => Enable Charge Pump
}

void Set_Segment_Remap(unsigned char d)
{
  LED_WrCmd(0xA0|d);    // Set Segment Re-Map
              // Default => 0xA0
              // 0xA0 (0x00) => Column Address 0 Mapped to SEG0
              // 0xA1 (0x01) => Column Address 0 Mapped to SEG127
}

void Set_Entire_Display(unsigned char d)
{
  LED_WrCmd(0xA4|d);    // Set Entire Display On / Off
              // Default => 0xA4
              // 0xA4 (0x00) => Normal Display
              // 0xA5 (0x01) => Entire Display On
}

void Set_Inverse_Display(unsigned char d)
{
  LED_WrCmd(0xA6|d);    // Set Inverse Display On/Off
              // Default => 0xA6
              // 0xA6 (0x00) => Normal Display
              // 0xA7 (0x01) => Inverse Display On
}

void Set_Multiplex_Ratio(unsigned char d)
{
  LED_WrCmd(0xA8);      // Set Multiplex Ratio
  LED_WrCmd(d);     // Default => 0x3F (1/64 Duty)
}

void Set_Display_On_Off(unsigned char d)
{
  LED_WrCmd(0xAE|d);    // Set Display On/Off
              // Default => 0xAE
              // 0xAE (0x00) => Display Off
              // 0xAF (0x01) => Display On
}

void SetStartPage(unsigned char d)
{
  LED_WrCmd(0xB0|d);    // Set Page Start Address for Page Addressing Mode
              // Default => 0xB0 (0x00)
}

void Set_Common_Remap(unsigned char d)
{
  LED_WrCmd(0xC0|d);    // Set COM Output Scan Direction
              // Default => 0xC0
              // 0xC0 (0x00) => Scan from COM0 to 63
              // 0xC8 (0x08) => Scan from COM63 to 0
}

void Set_Display_Offset(unsigned char d)
{
  LED_WrCmd(0xD3);      // Set Display Offset
  LED_WrCmd(d);     // Default => 0x00
}

void Set_Display_Clock(unsigned char d)
{
  LED_WrCmd(0xD5);      // Set Display Clock Divide Ratio / Oscillator Frequency
  LED_WrCmd(d);     // Default => 0x80
              // D[3:0] => Display Clock Divider
              // D[7:4] => Oscillator Frequency
}

void Set_Precharge_Period(unsigned char d)
{
  LED_WrCmd(0xD9);      // Set Pre-Charge Period
  LED_WrCmd(d);     // Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
              // D[3:0] => Phase 1 Period in 1~15 Display Clocks
              // D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void Set_Common_Config(unsigned char d)
{
  LED_WrCmd(0xDA);      // Set COM Pins Hardware Configuration
  LED_WrCmd(0x02|d);    // Default => 0x12 (0x10)
              // Alternative COM Pin Configuration
              // Disable COM Left/Right Re-Map
}

void Set_VCOMH(unsigned char d)
{
  LED_WrCmd(0xDB);      // Set VCOMH Deselect Level
  LED_WrCmd(d);     // Default => 0x20 (0.77*VCC)
}

void Set_NOP(void)
{
  LED_WrCmd(0xE3);      // Command for No Operation
}

void LED_Init(void)        
{
  unsigned char i;
        LEDPIN_Init();
        digitalWrite(SCL_PIN,HIGH);;;
        digitalWrite(RST_PIN,LOW);;;
        LED_DLY_ms(50);
        digitalWrite(RST_PIN,HIGH);
  Set_Display_On_Off(0x00);     // Display Off (0x00/0x01)
  Set_Display_Clock(0x80);      // Set Clock as 100 Frames/Sec
  Set_Multiplex_Ratio(0x3F);    // 1/64 Duty (0x0F~0x3F)
  Set_Display_Offset(0x00);     // Shift Mapping RAM Counter (0x00~0x3F)
  SetStartLine(0x00);       // Set Mapping RAM Display Start Line (0x00~0x3F)
  Set_Charge_Pump(0x04);      // Enable Embedded DC/DC Converter (0x00/0x04)
  SetAddressingMode(0x02);    // Set Page Addressing Mode (0x00/0x01/0x02)
  Set_Segment_Remap(0x01);    // Set SEG/Column Mapping  
  Set_Common_Remap(0x08);   // Set COM/Row Scan Direction 
  Set_Common_Config(0x10);    // Set Sequential Configuration (0x00/0x10)
  Set_Precharge_Period(0xF1);   // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  Set_VCOMH(0x40);        // Set VCOM Deselect Level
  Set_Entire_Display(0x00);     // Disable Entire Display On (0x00/0x01)
  Set_Inverse_Display(0x00);    // Disable Inverse Display On (0x00/0x01)  
  Set_Display_On_Off(0x01);     // Display On (0x00/0x01)
  LED_Fill(0x00);                               //clear all
  LED_Set_Pos(0,0);   
} 
 

void LED_P6x8Char(unsigned char x,unsigned char y,unsigned char ch)
{
   unsigned char c=0,i=0,j=0;     
     
  c =ch-32;
  if(x>122)
  {
    x=0;
    y++;
  }
  LED_Set_Pos(x,y);    
  for(i=0;i<8;i++)
  {     
    LED_WrDat(F8x6[c][i]);  
  }
}

void LED_P6x8Str(unsigned char x,unsigned char y,char ch[])
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {    
    c =ch[j]-32; 
	// Indirizzamento al carattere corretto nella matrice
    if ( c == 3) c = 1;
    if ( c == 5) c = 2;
    if ( c == 6) c = 3;
    if ( c == 14) c = 4;
    if ( c == 16) c = 5;
    if ( c == 17) c = 6;
    if ( c == 18) c = 7;
    if ( c == 19) c = 8;
    if ( c == 20) c = 9;
    if ( c == 21) c = 10;
    if ( c == 22) c = 11;
    if ( c == 23) c = 12;
    if ( c == 24) c = 13;
    if ( c == 25) c = 14;
    if ( c == 26) c = 15;
    if ( c == 29) c = 16;
    if ( c == 33) c = 17;
    if ( c == 34) c = 18;
    if ( c == 35) c = 19;
    if ( c == 41) c = 20;
    if ( c == 43) c = 21;
    if ( c == 44) c = 22;
    if ( c == 46) c = 23;
    if ( c == 52) c = 24;
    if ( c == 53) c = 25;
    if ( c == 63) c = 26;
    if ( c == 87) c = 27;
    if(x>126)
    {
      x=0;
      y++;
    }
    LED_Set_Pos(x,y);    
    for(i=0;i<8;i++)
    {     
      LED_WrDat(F8x6[c][i]);  
    }
    y+=1;
    j++;
  }
}

void LED_PrintBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char bmp[])
{   
  int ii=0;
  unsigned char x,y;
  for(y=y0;y<=y1;y++)
  {
    LED_Set_Pos(x0,y);        
    for(x=x0;x<x1;x++)
    {      
      LED_WrDat(bmp[ii++]);       
    }
  }
}

// Routine per la trasformazione da valore numerico a stringa per visualizzazione su OLED
void Digit() { 
	
	Digit1 = OLEDDigit / 100;
	Digit2 = (OLEDDigit - Digit1 * 100) / 10;
	Digit3 = (OLEDDigit - Digit1 * 100 - Digit2 * 10);
	if (PercPot == 0) {
		StringaOLED[0] = '0' + Digit1;
		StringaOLED[1] = '.';
		StringaOLED[2] = '0' + Digit2;
		StringaOLED[3] = '0' + Digit3;
		StringaOLED[4] = 0;
		StringaOLED[5] = 0;
	} 
	if (PercPot == 1) {
		if (Digit1 == 0) {
			StringaOLED[0] = ' ';//- 16; // Spazio
		}
		else {
			StringaOLED[0] = '0' + Digit1;
		}
		if (Digit2 == 0) {
			StringaOLED[1] = ' '; //- 16; // Spazio
		}
		else {
			StringaOLED[1] = '0' + Digit2;
		}
		StringaOLED[2] = '0' + Digit3;
		StringaOLED[3] = '%';
		StringaOLED[4] = 0;
		StringaOLED[5] = 0;
	}
}
 
 

 void setup()
 {
   LEDPIN_Init();
   LED_Init();
   TA1.current(TATotali, CalTotali);
   TA2.current(TABox, CalBox);
   Refresh = 0;

   pinMode(13, OUTPUT);
   Serial.begin(9600); // start serial for output
					   
   Wire.begin(SLAVE_ADDRESS);

   // define callbacks for i2c communication
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   Serial.println(" I2C Ready!");
   
   
   }

 void receiveData(int byteCount) {

	 while (Wire.available()) {
		 number = Wire.read();
		 Serial.print("data received: ");
		 Serial.println(number);

		 if (number == 1) {

			 if (state == 0) {
				 digitalWrite(13, HIGH); // set the LED on
				 state = 1;
			 }
			 else {
				 digitalWrite(13, LOW); // set the LED off
				 state = 0;
			 }
		 }
	 }
 }

 // callback for sending data
 void sendData() {
	 Wire.write(number);
 }

 void loop()
 {
	Refresh = Refresh + 1;
	ITotali = TA1.calcIrms(1480);
	IBox = TA2.calcIrms(1480);
	KWTotali = (ITotali*Volts) / 1000;
	KWBox = (IBox*Volts) / 1000;
	
    OLEDDigit = (int)(KWTotali * 100);
	Digit ();
	LED_P6x8Str(120,0,"KwT:");
	LED_P6x8Str(120, 4, StringaOLED);
	PercentualePotenza = (100 * OLEDDigit) / 300;
	OLEDDigit = PercentualePotenza;
	PercPot = 1;
	Digit();
	LED_P6x8Str(90, 2, StringaOLED);
	PercPot = 0;
    OLEDDigit = (int)(KWBox * 100);
	Digit();
    LED_P6x8Str(110,0,"KwB:");
	LED_P6x8Str(110, 4, StringaOLED);
	LED_P6x8Str(10, 0, "IT:");
	OLEDDigit = (int)(ITotali * 100);
	Digit();
    LED_P6x8Str(10, 3, StringaOLED);
	LED_P6x8Str(10, 7, "A");
	LED_P6x8Str(0, 0, "IB:");
	OLEDDigit = (int)(IBox * 100);
	Digit();
	LED_P6x8Str(0, 3, StringaOLED);
	LED_P6x8Str(0, 7, "A");
	
	if (KWTotali <= 0.37) LED_P6x8Str(80, 0, "#       ");
	if (KWTotali > 0.37 && KWTotali <= 0.75) LED_P6x8Str(80, 0, "##      ");
	if (KWTotali > 0.75 && KWTotali <= 1.12) LED_P6x8Str(80, 0, "###     ");
	if (KWTotali > 1.12 && KWTotali <= 1.50) LED_P6x8Str(80, 0, "####    ");
	if (KWTotali > 1.50 && KWTotali <= 1.87) LED_P6x8Str(80, 0, "#####   ");
	if (KWTotali > 1.87 && KWTotali <= 2.25) LED_P6x8Str(80, 0, "######  ");
	if (KWTotali > 2.25 && KWTotali <= 2.62) LED_P6x8Str(80, 0, "####### ");
	if (KWTotali > 2.62) LED_P6x8Str(80, 0, "########");

    if (KWTotali >= 2.95) {
      LED_PrintBMP(32,2,64,5,(unsigned char *)Allarme);
      delay (250);
      LED_PrintBMP(32,2,64,5,(unsigned char *)Vuoto);
    }
    if (IBox < 0.11 && KWTotali < 2.95 && Ciclo == 0) {
      LED_PrintBMP(32,2,64,5,(unsigned char *)Vuoto);
      FiltroON = 0;
    }
    if (IBox < 0.15 && KWTotali < 2.95 && Ciclo == 1) {
      FiltroOFF = FiltroOFF + 1;
    }
    if (FiltroOFF > 360) {
      Ciclo = 0;
      FiltroOFF = 0;
    }
	if (IBox > 0.20 && IBox < 0.95 && KWTotali < 2.95 && Ciclo == 0) {
		FiltroON = FiltroON + 1;
		if (FiltroON > 10) {
			LED_PrintBMP(32, 2, 64, 5, (unsigned char *)Lampadina);
			FiltroON = 11;
		}
	}
    if (IBox > 1.00 && KWTotali < 2.95) {
		FiltroON = FiltroON + 1; 
		if (FiltroON > 10) {
			LED_PrintBMP(32, 2, 64, 5, (unsigned char *)Lavatrice);
			Ciclo = 1;
			FiltroON = 11;
		}
    }

	if (Refresh > 3600 && KWTotali < 2.95) { // Cancellazione display ogni 30 minuti circa
		LED_Fill(0x00);
		Refresh = 0;
		delay(2000);
	}
	Serial.print("KW Linea: ");
	Serial.println(KWTotali);
	Serial.print("KW Box-Lavanderia: ");
	Serial.println(KWBox);
	Serial.print("Percentuale Potenza: ");
	Serial.print(PercentualePotenza);
	Serial.println("%");
	Serial.print("I assorbita Linea: ");
	Serial.println(ITotali);
	Serial.print("I assorbita Box-Lavanderia: ");
	Serial.println(IBox);
	Serial.println();
 }
 
