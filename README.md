# ATmega4809_Project_Optical Tachometer

### AVR-based Optical Tachometer

**Functionality** 

- Measured the RPM of small DC motor with visible light and LDR
- Update RPM once a second on LCD1602
- Adjustable motor RPM control with PWN + L9110

**Module in used**

- ADC_RESSEL_10BIT_gc
- RTC_PERIOD_CYC16_gc
- TCB_CNTMODE_PWM8_gc

**PIN Assignment**(latest)

- LCD1602
  - **PC4** - D4
  - **PC5** - D5
  - **PC6** - D6
  - **PC7** - D7
  - **PC0** - EN
  - **PC1** - RS
- ADC + LDR
  - **PE0** - LDR
- TCB+L9110
  - **PB5** - FORWARD PWM

**Others**

- IDE: MPLAB X 

### UPDATE

#### version 3.1

(2020-12-31)

- new feature: implement adjustable DC motor with L9110.
- **Issue:** LCD1602 data transmission  is VERY likely corrupt due to over voltage consumed from L9110 chip. 

#### version 3.0

(2020-12-30)

- fix: threshold can self-adaptive every second(anti-ambient_light)
- OneClickOptimize mode now has been removed.

#### version2.0 

(2020-12-27)

- new feature: add OneClickOptimize mode, able optimize threshold. 
