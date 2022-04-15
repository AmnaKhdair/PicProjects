/*
 * File:   ourMain.c
 * Author: ThinkPad
 *
 * Created on December 6, 2021, 12:42 PM
 */

#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <stdio.h>
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE  40536
#define STARTVALUE2  3036
#define WINTER_T 40
#define SUMMER_T 60
#include <xc.h>
unsigned short Mflag=0;
unsigned short modeFlag=0;
unsigned short heatCounter=0;
unsigned short coolCounter=0;
unsigned short countInt=0;
unsigned short count=0;
unsigned short HS=0;
unsigned int RPS_count = 0;
unsigned int AutoMode = 0;
void initInt(void) { 
    INTCON=0;
    INTCON2=0;
    PIR1=0;
    PIE2=0;    
    PIE2bits.TMR3IE = 1;
    T3CON=0x80;
    RCONbits.IPEN=0; 
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
    INTCONbits.INT0IE=1;
    INTCON3bits.INT1IE=1;
    INTCON3bits.INT2IE=1;
    INTCON2bits.INTEDG0=1;
    INTCON2bits.INTEDG1=1;
    INTCON2bits.INTEDG2=1;
    T3CONbits.TMR3ON = 1;
    TMR3H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR3L = (unsigned char) (STARTVALUE & 0x00FF);


   
}
void initT01(void) {
    T0CON = 0;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE2 >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE2 & 0x00FF);
    T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}
void implemntInt0(void){
    INTCONbits.INT0F =0; // Must be cleared by software 
    if(modeFlag==0) modeFlag=1;//COOL
    else if(modeFlag==1) modeFlag=2;//AUTO
    else modeFlag=0;//HEAT
 

}
void implemntInt1(void){
     INTCON3bits.INT1F=0;
    if(modeFlag==0)
    {
    if(heatCounter!=0)
    heatCounter--;
    }
    else if(modeFlag==1)
    {
    if(coolCounter!=0)
    coolCounter--;
    }
    else 
    {
     if (HS!=0) HS--;
    }
    
    
}
void implemntInt2(void){
    INTCON3bits.INT2F=0;
     if(modeFlag==0)
    {
    if(heatCounter!=10)
    heatCounter++;
     }
     else if(modeFlag==1)
    {
    if(coolCounter!=10)
    coolCounter++;
   }
    else 
    {
        if(HS!=5) HS++;
     }
}
void reloadTimer3(void){
    TMR3H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR3L =  (unsigned char)(STARTVALUE & 0x00FF ); 
}
void implemntTR3(void){
   
    PIR2bits.TMR3IF=0;
    if(countInt==10) countInt=0;
    else countInt++; 
    char Buffer[32];
    
    if(countInt<heatCounter)
        PORTCbits.RC5=1;
    else
       PORTCbits.RC5=0;
       
   reloadTimer3();

}
void implemntTR0(void){
    
   PORTDbits.RD1 = !PORTDbits.RD1; //Toggle RD0 every .5 second

    RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); //
    TMR0H = (unsigned char) ((STARTVALUE2 >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE2 & 0x00FF);
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.T0IF = 0;

}
void __interrupt(high_priority) highIsr (void){
   Mflag=1;
    if(INTCONbits.INT0F) implemntInt0();
    if(INTCON3bits.INT1F) implemntInt1();
    if(INTCON3bits.INT2F) implemntInt2();
    if(PIR2bits.TMR3IF) implemntTR3();
    if(INTCONbits.T0IF )implemntTR0();
}
void setupPorts(void) {
   ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
 //   ADCON1 = 0x0F;
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x00; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
  

}
void main(void) {
    
    char Buffer[32];
    float AN[3];    
    int raw_val;
    unsigned char channel;
    float voltage;
    float temp;  
    int  CoolError;
    int T,SP;
    float PWMvalue;
    int RPS;
    int HeatError;
    setupPorts();
    initInt();
    initT01();
    TRISCbits.RC0=1;
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    while (1) {
    for (channel = 0; channel < 3; channel++) 
    {
      
        if(channel==2)
        {   
        voltage= read_adc_voltage((unsigned char) channel); 
        AN[channel] = voltage*100; 
        }
        else{
        raw_val = read_adc_raw_no_lib(channel);
        temp= (raw_val * 100.0) / 1023.0;
        AN[channel]=temp;
        }
    
    }
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "RT: %4.1fC\nSP: %4.1fC", AN[2], AN[0]);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT: %4.1fC", AN[1]);
    lcd_puts(Buffer);
    
    ///////////
 
    if(modeFlag==0)
    {    
    set_pwm1_percent(0.0);
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "MD: Heat  %2d",heatCounter);
    lcd_puts(Buffer);
    
    }
    else if(modeFlag==1)
    {
    heatCounter=0;
    set_pwm1_percent(coolCounter*10);
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "MD: Cool  %2d",coolCounter);
    lcd_puts(Buffer);
    }
    //Auto...
     else if(modeFlag==2)
    {
    int OT= AN[1]; 
    //COOL MODE ...
   
    if (OT > SUMMER_T) 
    { T=AN[2];
    SP=AN[0];
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "MD: AutoCL HS:%d  ",HS);
    lcd_puts(Buffer);
    //SPEED...
    RPS = RPS_count;
    lcd_gotoxy(11, 3);
    sprintf(Buffer, "R=1.1f     ", RPS/7.0); 
    lcd_puts(Buffer); 
    
    AutoMode=1;

    CoolError = T - SP;
    if( CoolError > 0)
    PWMvalue = CoolError*100/10;
    if(PWMvalue>25)
    set_pwm1_percent(PWMvalue*10);
    else set_pwm1_percent(25*10);
    if(T < (SP -HS) )
    { set_pwm1_percent(0.0);
       heatCounter=5;
    }

    }
    //HEAT MODE ...
    else if (OT < WINTER_T)
    {
    T=AN[2];
    SP=AN[0];
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "MD: AutoHT HS:%d  ",HS);
    lcd_puts(Buffer);
    set_pwm1_percent(0.0);   
    HeatError = SP - T;
    if(HeatError > 0)
    {
    heatCounter = HeatError ; 
    if(heatCounter>=10) heatCounter=10;
    if (heatCounter < 5 )heatCounter=5;
    if(SP > 52) heatCounter=10; 
    lcd_gotoxy(11, 3);
    sprintf(Buffer, "H=%3d", heatCounter*10); 
    lcd_puts(Buffer); 
    }
    if(T > (SP +HS))heatCounter=0;
    
    AutoMode=2;
    }
    
    
    
    
    
    
    
    //BOTH MODE ...
    else 
    {
    AutoMode=0;
    HeatError= SP - T;
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "MD: AutoHC      " );
    lcd_puts(Buffer);    
    
    
    //HEAT MODE BOTH...
    if(HeatError > 0){
     T=AN[2];
    SP=AN[0];
    set_pwm1_percent(0.0);   
    HeatError = SP - T;

    if(HeatError > 0)
    {
    heatCounter = HeatError ; 
    if(heatCounter>=10) heatCounter=10;
    if (heatCounter < 5 )heatCounter=5;
    if(SP > 52) heatCounter=10; 
    lcd_gotoxy(11, 3);
    sprintf(Buffer, "H=%3d", heatCounter*10); 
    lcd_puts(Buffer); 
    }
    if(T > (SP +HS))heatCounter=0;

    }
    
    
    
    
    //COOL MODE BOTH...
    else {
    CoolError = (T -SP);
    T=AN[2];
    SP=AN[0];
    //SPEED...
    RPS = RPS_count;
    lcd_gotoxy(11, 3);
    sprintf(Buffer, "R=%1.1f   ", RPS/7.0); 
    lcd_puts(Buffer); 
    
    AutoMode=1;

    CoolError = T - SP;
    if( CoolError > 0)
    PWMvalue = CoolError*100/10;
    if(PWMvalue>25)
    set_pwm1_percent(PWMvalue*10);
    else set_pwm1_percent(25*10);
    if(T < (SP -HS) )
    { set_pwm1_percent(0.0);
       heatCounter=5;
    }

      }

    }
    
 }
         
  Mflag=0;
    }

        return;
    }
   
 


