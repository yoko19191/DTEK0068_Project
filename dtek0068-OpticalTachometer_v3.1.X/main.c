/*
 * File:   main.c
 * Author: Guanghang Chen(guchen@utu.fi)
 * Created on 31 December 2020, 17:52
 * Description: 
 * Measured the RPM of small DC motor with visible light and LDR
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

/**  PIN Assignment for TCB3
 * PB5 - L9110's forward PWM
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

/*---global variable---*/
/*adcValue is shared between main and ISR*/
volatile uint16_t adcValue; 
/*breakNum is shared between main and ISR */
volatile uint16_t breakNum;
/*count is shared between main and ISR*/
volatile uint16_t count;
/*flag is used for setting duty of DC motor*/
/*which shared by main and ISR*/
volatile uint8_t flag = 0;

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
void tcb3_init (void);
void port_init(void);

/*-----ISR-----*/

ISR(RTC_PIT_vect)  
{  
    /*Reset RTC FLAG*/
    RTC.PITINTFLAGS = RTC_PI_bm;
    
    /*counting every 0.5ms*/
    if( adc_is_conersion_done() )
    {   
        /*read value from on board ADC*/
        adcValue = adc_read();
        count++;
    }
  
}//ISR(RTC_PIT_vect)  


ISR(PORTF_PORT_vect)
{
    /*Clearing an interrupt flag*/
    PORTF.INTFLAGS = PORTF.INTFLAGS;
    
    /*Flag increase every press button*/
    flag++;
    
}//ISR(PORTF_PORT_vect)

/*-----ISR-----*/


int main(void) 
{
    /* Initialization all stuff  here */
    lcd_init();
    rtc_init();
    adc_init();
    port_init();
    tcb3_init();
    
    /*two variables for optimizing threshold*/
    uint16_t max=0;
    uint16_t min=1023;
    /*Threshold can be modified*/
    /*and exist in main only*/
    /*initial value came from testing*/
    uint16_t threshold = 295;
    
    /*setting.....*/
    adc_start();
    RTC.PITINTCTRL |= RTC_PI_bm; /*Enable RTC*/
    RTC.PITCTRLA |= RTC_PERIOD_CYC16_gc; /*enable RTC clock cycle 16*/
    RTC.PITCTRLA &= ~(RTC_PITEN_bm); /*stop RTC*/
    
    
    /*Setting Sleep mode to IDLE*/
    set_sleep_mode(SLPCTRL_SMODE_IDLE_gc); 
    
    sei();  /*enable global interrupt*/
    while (1) 
    {   
        /*Go to sleep after Every ISR turn*/
        sleep_mode();
        
        
        /*Setting duty for TCB3*/
        /*Using for-else instead of switch-case due to efficient*/
        if(flag==1)
        {
            TCB3.CCMPH = 0x00;
            lcd_print_xy(0,10,int2str(flag));
        }
        else if(flag==2)
        {
            TCB3.CCMPH = 0xC0;
            lcd_print_xy(0,10,int2str(flag));
        }
        else if(flag==3)
        {
            TCB3.CCMPH = 0xFF;
            lcd_print_xy(0,10,int2str(flag));
            //reset flag
            flag = 0;
        }
        
        /*breakNum add one if adcValue reach exactly threshold*/
        if(adcValue == threshold)
        {
            breakNum++;
        }
        
        /*Finding max and min in 2048 samples(1s)*/
        if(max<adcValue)
        {
            max = adcValue;
        }
            
        if(min>adcValue)
        {
            min = adcValue;  
        }
      
        
        /*If RTC is running and 1 sec up*/
        if((RTC.PITCTRLA & RTC_PI_bm) && (count==2048))
        {
            /*Reset count to zero*/
            count = 0;
            
            /*this propellor gives two break every rotation*/
            /*thus how breakNum/2*60 came from*/
            uint16_t rpm = breakNum/2*60;
            
            /*Display result*/
            lcd_clear(); 
            lcd_print_xy(0,0,"RPM:");
            lcd_print_xy(0,4,int2str(rpm));
            /*Code for testing*/
            lcd_print_xy(1,0,"ADC:");
            lcd_print_xy(1,4,int2str(adcValue));  
            lcd_print_xy(1,10,int2str(threshold) );
            
            
            /*If propellor is not rotating*/
            if( (max-min)<10)
            {
                rpm = 0;
            }
            else
            {   
                /*optimized threshold every second*/
                threshold = (min+max)/2;  
            }
            
            /*Reset max and min */
            max = 0; 
            min = 1023;
            
            /*Reset breakNum to zero*/
            breakNum = 0;
        }//automatically adjust threshold
     
        
        
        
        
    }//while(1)
    
}//main()




void port_init(void)
{
    /*Setup button and reset its status*/
    PORTF.DIRCLR = PIN6_bm;
    PORTF.PIN6CTRL = PORT_ISC_FALLING_gc;
    /*Set PB5 as output for TCB3 and initialize it high*/
    PORTB_DIRSET |= PIN5_bm;
    
}

void tcb3_init (void)
{
    /* Load CCMP register with the period and duty cycle of the PWM */
    TCB3.CCMP = 0x00FF;

    /* Enable TCB3 and Divide CLK_PER by 2(6.5kHz)*/
    TCB3.CTRLA |= TCB_ENABLE_bm;
    TCB3.CTRLA |= TCB_CLKSEL_CLKDIV2_gc;
    
    /* Enable Pin Output and configure TCB in 8-bit PWM mode */
    TCB3.CTRLB |= TCB_CCMPEN_bm;
    TCB3.CTRLB |= TCB_CNTMODE_PWM8_gc;
}


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
}//adc_is_conersion_done;
/*-----ADC---1-M-*/
