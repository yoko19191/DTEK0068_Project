/*
 * File:   main.c
 * Author: dtek0068
 *
 * Created on 27 December 2020, 22:30
 */

/**  PIN Assignment for LCD1602
 * D4 - PC4
 * D5 - PC5
 * D6 - PC6
 * D7 - PC7
 * EN - PC0
 * RS - PC1
 */

/**  PIN Assignment for ADC
 * PE0 - LDR
 * GND - LDR
 */

#define F_CPU 3333333

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 

/*-----DEFINE-----*/
#define LCD_DIR PORTC_DIR         /* Define LCD data port direction */
#define LCD_PORT PORTC_OUT        /* Define LCD data port */
#define EN 0                      /* EN pin to PORTC_PIN0 */  
#define RS 1                      /* RS pin to PORTC_PIN1 */ 
/*Threshold number */
/*magic nuber(nope), this adcValue was came from our testing */
/*under certain ambient light and motor*/
#define THRESHOLD 280             

/*---global variable---*/
/*adcValue is shared between main and ISR*/
volatile uint16_t adcValue; 
/*breakNum exists in main only */
uint16_t breakNum;
/*counter is shareed between main and ISR*/
volatile int counter;

/*---prototype---*/
void lcd_command(unsigned char cmnd);
void lcd_init(void);
void lcd_clear(void);
void lcd_print_char(unsigned char data);   
void lcd_print(char *str);
void lcd_print_xy(unsigned char row,unsigned char pos, char *str);
char* int2str(int data);
void rtc_init(void);
void adc_init(void);
uint16_t adc_read(void);
void adc_start(void);
bool adc_is_conersion_done(void);

/*-----ISR-----*/

//ISR(ADC0_RESRDY_vect)
//{
//    /*Read value from ADC Every 0.5 ms(almost real time)*/    
//    adcValue = adc_read();
//}


ISR(RTC_PIT_vect)  
{  
    /*Reset RTC FLAG*/
    RTC.PITINTFLAGS = RTC_PI_bm;
    
    /*counting for 1 sec*/
    counter++;
    
    if(adc_is_conersion_done())
    {
        adcValue = adc_read();
    }
}
/*-----ISR-----*/
int main(void) 
{
    /* Initialization all stuff  here */
    lcd_init();
    rtc_init();
    adc_init();
    
    /*setting.....*/
    adc_start();
    RTC.PITINTCTRL |= RTC_PI_bm; /*Enable RTC*/
    RTC.PITCTRLA |= RTC_PERIOD_CYC16_gc; /*enable RTC clock cycle 16*/
    RTC.PITCTRLA &= ~(RTC_PITEN_bm); /*stop RTC*/
    
    
    set_sleep_mode(SLPCTRL_SMODE_IDLE_gc); 
    
    sei();  /*enable global interrupt*/
    while (1) 
    {   
        /*breakNum count how many time LDR are blocked by propellor in 1s*/
        if(adcValue==THRESHOLD)
        {
            breakNum++;
        }
        
        /*RTC period is 0.5ms, 2048 cycles is 1 second*/
        if(counter==2048)
        {   
            /*reset counter to 0*/
            counter = 0;
            
            /*our propellor gives two break every rotation*/
            /*thus how breakNum/2*60 came from*/
            uint16_t rpm = breakNum/2*60;
            /*Display result*/
            lcd_clear(); 
            lcd_print_xy(0,0,"RPM:");
            lcd_print_xy(0,4,int2str(rpm));
            lcd_print_xy(1,0,"ADC_");
            lcd_print_xy(1,4,int2str(adcValue));        
            /*reset breakNum after every second print*/
            breakNum=0;
        }
        
    }
}//main()




/*-----LCD-----*/
void lcd_command(unsigned char cmnd)
{   
    /* Sending upper nibble */
    LCD_PORT = (LCD_PORT & 0x0F ) | (cmnd & 0xF0);
    /* RS=0 */
    LCD_PORT &= ~(1<<RS);  
    /* Enable pulse*/
    LCD_PORT |= (1<<EN);                            
    _delay_us(1);
     /* Clear bit in EN */
    LCD_PORT &= ~(1<<EN);                          
    _delay_us(200);
    /*Sending lower nibble */
    LCD_PORT = (LCD_PORT & 0x0F) | (cmnd<<4);       
    LCD_PORT |= (1<<EN); 
    _delay_us(1);
    LCD_PORT &= ~(1<<EN);
    _delay_ms(2);
}//void lcd_command(unsigned char);

void lcd_init(void)
{   
    /* Make LCD port direction as o/p */
    LCD_DIR = 0xFF; 
     /*LCD Power on delay always >15ms */
    _delay_ms(20);  
    /*Initializes LCD in 4-bit mode*/
    lcd_command(0x02); 
    /*Configures LCD in 2-line 4-bit mode and 5*8 matrix*/
    lcd_command(0x28);  
    /*Display on, cursor off */
    lcd_command(0x0c);   
    /*Increment cursir (shift cursor to right)*/
    lcd_command(0x06); 
    /*Clear display screen*/
    lcd_command(0x01);   
     /* Clear Display needs 1.64ms */
    _delay_ms(2);             
}//void lcd_init(void);

void lcd_clear(void)
{   
    /* Clear display */
    lcd_command(0x01);         
    _delay_ms(2);
    /* Cursor at home position */
    lcd_command(0x80);         
}//void lcd_clear();

void lcd_print_char(unsigned char data)   
{   
    /* Sending upper nibble */
    LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0); 
    /* RS=1, data reg */
    LCD_PORT |= (1<<RS);      
    LCD_PORT |= (1<<EN);
    _delay_us(1);
    LCD_PORT &= ~(1<<EN);
    _delay_us(200);
    /* Sending lower nibble */
    LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);   
    LCD_PORT |= (1<<EN);
    _delay_us(1);
    LCD_PORT &= ~(1<<EN);
    _delay_ms(2);
}//void lcd_char(unsigned char);

void lcd_print(char *str)
{
    int i;
    /*call lcd_print_char() until reach '\0'*/
    for(i=0;str[i]!=0;i++)    
    {
        lcd_print_char(str[i]);
    }
}//void lcd_string(char*);

void lcd_print_xy(unsigned char row,unsigned char pos, char *str)
{
    if( (row==0) && (pos<16))
    {
        /*Command of first row and required position <16*/
        lcd_command( (pos & 0x0F) | (0x80) ); 
    }
    else if( (row==1) && (pos<16))
    {
        /*Command of first row and required position <16*/
        lcd_command( (pos & 0x0F) | (0xC0) ); 
    }
    lcd_print(str);
}//void lcd_string_xy(unsigned char,unsigned char,char*);

char* int2str(int data)
{   
    static char buffer[6];
    int num = data;
    itoa(num,buffer,10);
    return buffer;
}//char* int2str(int);
/*-----LCD-----*/

/*-----RTC-----*/
void rtc_init(void)
{
    uint8_t temp;
    //Init Oscillator in RTC
    temp = CLKCTRL.XOSC32KCTRLA; 
    //Disable oscillator 
    temp &= ~CLKCTRL_ENABLE_bm; 
    //register protection
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp); 
    
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
        ; /* Wait until XOSC32KS becomes 0 */
    }
    
    /* SEL = 0 (Use External Crystal): */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp &= ~CLKCTRL_SEL_bm;
    /* Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    
    /* Enable oscillator: */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm;
    /* Writing to protected register */
    ccp_write_io((void*)&CLKCTRL.XOSC32KCTRLA, temp);
    
    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ; /* Wait for all register to be synchronized */
    }

    /* 32.768kHz External Crystal Oscillator (XOSC32K) */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;

    /* Run in debug: enabled */
    RTC.DBGCTRL = RTC_DBGRUN_bm;
    /* Periodic Interrupt: enabled */
    RTC.PITINTCTRL = RTC_PI_bm; 
    /* RTC Clock Cycles 16 (0.5ms) */
    RTC.PITCTRLA = RTC_PERIOD_CYC16_gc 
                 | RTC_PITEN_bm; /* Enable: enabled */
}//void rtc_init(void);
/*-----RTC-----*/
/*-----ADC-----*/
 void adc_init(void)
{
    /* Disable digital input buffer */ 
    PORTE.PIN0CTRL &= ~PORT_ISC_gm;
    /*PE0 is where LDR is connected to*/
    PORTE.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    
    /* Disable pull-up resistor */
    PORTE.PIN0CTRL &= ~PORT_PULLUPEN_bm;
    
    /*Select reference voltage as 2.5V*/
    VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
 
    /* CLK_PER divided by 4 */
    ADC0.CTRLC = ADC_PRESC_DIV16_gc      
               |ADC_REFSEL_INTREF_gc;  
    /* ADC Enable: enabled 10-bit mode */
    ADC0.CTRLA = ADC_ENABLE_bm          
               | ADC_RESSEL_10BIT_gc;   
    /* Select ADC channel */
    ADC0.MUXPOS  = ADC_MUXPOS_AIN8_gc;
    /* Enable FreeRun mode */
    ADC0.CTRLA |= ADC_FREERUN_bm;
}//void adc_init();

 uint16_t adc_read(void)
{
    /* Clear the interrupt flag by writing 1: */
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    return ADC0.RES;
}//uint16_t adc_read();
 
 void adc_start(void)
{
    /* Start conversion */
    ADC0.COMMAND = ADC_STCONV_bm;
}//adc_start;
 
bool adc_is_conersion_done(void)
{
    return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}//adc_is_conersion_done
/*-----ADC-----*/