#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/

uint8_t PWM_PIN[8] = {9,10,5,6};



/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    uint16_t Temp2;
    Temp2 = motor[2] - 1000;
    #if (NUMBER_MOTOR > 0)
        // Timer 1 A & B [1000:2000] => [8000:16000]
        #ifdef EXT_MOTOR_RANGE
          OCR1A = ((motor[1]<<4) - 16000) + 128;
        #elif defined(EXT_MOTOR_64KHZ)
          OCR1A = (motor[1] - 1000) >> 2; // max = 255
        #elif defined(EXT_MOTOR_32KHZ)
          OCR1A = (motor[1] - 1000) >> 1; // max = 511
        #elif defined(EXT_MOTOR_16KHZ)
          OCR1A = motor[1] - 1000;        //  pin 9
        #elif defined(EXT_MOTOR_8KHZ)
          OCR1A = (motor[1]-1000) << 1;   //  pin 9
        #else
          OCR1A = motor[1]<<3; //  pin 9
        #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifdef EXT_MOTOR_RANGE 
        OCR1B = ((motor[3]<<4) - 16000) + 128;
      #elif defined(EXT_MOTOR_64KHZ)
        OCR1B = (motor[3] - 1000) >> 2;
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1B = (motor[3] - 1000) >> 1;
      #elif defined(EXT_MOTOR_16KHZ)
        OCR1B = motor[3] - 1000;        //  pin 10
      #elif defined(EXT_MOTOR_8KHZ)
        OCR1B = (motor[3]-1000) << 1;   //  pin 10
      #else
        OCR1B = motor[3]<<3; //  pin 10
      #endif
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
        #ifdef EXT_MOTOR_RANGE 
          OCR3A = ((motor[0]<<4) - 16000) + 128;
        #elif defined(EXT_MOTOR_64KHZ)
          OCR3A = (motor[0] - 1000) >> 2;
        #elif defined(EXT_MOTOR_32KHZ)
          OCR3A = (motor[0] - 1000) >> 1;
        #elif defined(EXT_MOTOR_16KHZ)
          OCR3A = motor[0] - 1000;        //  pin 5
        #elif defined(EXT_MOTOR_8KHZ)
          OCR3A = (motor[0]-1000) << 1;   //  pin 5
        #else
          OCR3A = motor[0]<<3; //  pin 5
        #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifdef EXT_MOTOR_RANGE 
        TC4H = (((motor[2]-1000)<<1)+16)>>8; OCR4D = ((((motor[2]-1000)<<1)+16)&0xFF); //  pin 6
      #elif defined(EXT_MOTOR_64KHZ)
        Temp2 = Temp2 >> 2;
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_32KHZ)
        Temp2 = Temp2 >> 1;
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_16KHZ)
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_8KHZ)
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #else
        TC4H = motor[2]>>8; OCR4D = (motor[2]&0xFF); //  pin 6
      #endif
    #endif    

  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if defined(EXT_MOTOR_64KHZ) || defined(EXT_MOTOR_32KHZ) || defined(EXT_MOTOR_16KHZ) || defined(EXT_MOTOR_8KHZ)
      TCCR1A = (1<<WGM11);
      TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
      TCCR3A = (1<<WGM31);
      TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);
      #if defined(EXT_MOTOR_64KHZ)
        ICR1   = 0x00FF; // TOP to 255;
        ICR3   = 0x00FF; // TOP to 255;
        TC4H = 0x00;
        OCR4C = 0xFF; // phase and frequency correct mode & top to 255
        TCCR4B = (1<<CS40);             // prescaler to 1
      #elif defined(EXT_MOTOR_32KHZ)
        ICR1   = 0x01FF; // TOP to 511;
        ICR3   = 0x01FF; // TOP to 511;
        TC4H = 0x01;
        OCR4C = 0xFF; // phase and frequency correct mode & top to 511
        TCCR4B = (1<<CS40);             // prescaler to 1
      #elif defined(EXT_MOTOR_16KHZ)
        ICR1   = 0x03FF; // TOP to 1023;
        ICR3   = 0x03FF; // TOP to 1023;
        TC4H = 0x03;
        OCR4C = 0xFF; // phase and frequency correct mode & top to 1023
        TCCR4B = (1<<CS40);             // prescaler to 1
      #elif defined(EXT_MOTOR_8KHZ)
        ICR1   = 0x07FF; // TOP to 2046;
        ICR3   = 0x07FF; // TOP to 2046;
        TC4H = 0x3;
        OCR4C = 0xFF; // phase and frequency correct mode
        TCCR4B = (1<<CS41);             // prescaler to 2
      #endif
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
      TCCR4D = 0;
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
    #else
      #if (NUMBER_MOTOR > 0) && ( !defined(A32U4_4_HW_PWM_SERVOS) )
        TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
        TCCR1A &= ~(1<<WGM10);
        TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
        TCCR1B |= (1<<WGM13) | (1<<CS10); 
        ICR1   |= 0x3FFF; // TOP to 16383;     
        TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
      #endif
      #if (NUMBER_MOTOR > 1)
        TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
      #endif
      #if (NUMBER_MOTOR > 2)
        #if !defined(HWPWM6) // timer 4A
          TCCR4E |= (1<<ENHC4); // enhanced pwm mode
          TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
          TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
          TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
        #else // timer 3A
          TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
          TCCR3A &= ~(1<<WGM30);
          TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
          TCCR3B |= (1<<WGM33) | (1<<CS30); 
          ICR3   |= 0x3FFF; // TOP to 16383;     
          TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
        #endif 
      #endif
      #if (NUMBER_MOTOR > 3) || ( (NUMBER_MOTOR > 0) && defined(A32U4_4_HW_PWM_SERVOS) )
        #if defined(HWPWM6) 
          TCCR4E |= (1<<ENHC4); // enhanced pwm mode
          TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
          TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
        #endif
        TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
      #endif
      #if (NUMBER_MOTOR > 4)
        #if defined(HWPWM6) 
          TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
          TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A 
        #else
          initializeSoftPWM();
        #endif
      #endif
      #if (NUMBER_MOTOR > 6)
        #if defined(HWPWM6) 
          initializeSoftPWM();
        #endif
      #endif
    #endif
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
}



/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

/**************************************************************************************/
/************             Motor software PWM generation              ******************/
/**************************************************************************************/
// SW PWM is only used if there are not enough HW PWM pins (for exampe hexa on a promini)

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

  /****************    Pre define the used ISR's and Timerchannels     ******************/
  #if !defined(PROMICRO)
    #define SOFT_PWM_ISR1 TIMER0_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER0_COMPA_vect
    #define SOFT_PWM_CHANNEL1 OCR0B
    #define SOFT_PWM_CHANNEL2 OCR0A 
  #elif !defined(HWPWM6)
    #define SOFT_PWM_ISR1 TIMER3_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER3_COMPC_vect
    #define SOFT_PWM_CHANNEL1 OCR3B
    #define SOFT_PWM_CHANNEL2 OCR3C 
  #else
    #define SOFT_PWM_ISR2 TIMER0_COMPB_vect  
    #define SOFT_PWM_CHANNEL2 OCR0B 
  #endif
  
  /****************         Initialize Timers and PWM Channels         ******************/
  void initializeSoftPWM(void) {
    #if !defined(PROMICRO)
      TCCR0A = 0; // normal counting mode
      #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
      #endif
      #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
        TIMSK0 |= (1<<OCIE0A);
      #endif
    #else
      #if !defined(HWPWM6)
        TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
        TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
        TCCR3B |= (1<<CS30);   
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
          TIMSK3 |= (1<<OCIE3C);
        #endif   
      #else
        TCCR0A = 0; // normal counting mode
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
      #endif
    #endif
  }
  
  /****************               Motor SW PWM ISR's                 ******************/
  // hexa with old but sometimes better SW PWM method
  // for setups without servos
  #if (NUMBER_MOTOR == 6) && (!defined(SERVO) && !defined(HWPWM6))
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN5_highState>0) SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    }
    ISR(SOFT_PWM_ISR2) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN6_highState>0) SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 0;   
      }
    }
  #else
    #if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
      // HEXA with just OCR0B 
      ISR(SOFT_PWM_ISR1) { 
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_1_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_2_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_2_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_1_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
          state = 0;   
        }
      } 
    #endif
    //the same with digital PIN A2 & 12 OCR0A counter for OCTO
    #if (NUMBER_MOTOR > 6)
      ISR(SOFT_PWM_ISR2) {
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_3_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_4_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_4_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_3_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
          state = 0;   
        }
      }
    #endif
  #endif
#endif

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #if defined(DYNBALANCE)
    return;
  #endif
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + axisPID[YAW]*Z

  /****************                   main Mix Table                ******************/
  #if defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif


  /****************                compensate the Motors values                ******************/
  #ifdef VOLTAGEDROP_COMPENSATION
    {
      #if (VBATNOMINAL == 84) // 2S
        #define GOV_R_NUM 24
        static int8_t g[] = { 0,4,8,12,17,21,25,30,34,39,44,49,54,59,65,70,76,81,87,93,99,106,112,119,126 };
      #elif (VBATNOMINAL == 126) // 3S
        #define GOV_R_NUM 36
        static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
      #elif (VBATNOMINAL == 252) // 6S
        #define GOV_R_NUM 72
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,17,18,19,21,22,24,25,27,28,30,31,33,34,36,38,39,41,
            42,44,46,47,49,51,52,54,56,58,59,61,63,65,66,68,70,72,74,76,78,79,81,83,85,87,89,91,93,95,97,99,
            101,104,106,108,110,112,114,117,119,121,123,126        };
      #elif (VBATNOMINAL == 255) // 6S HV
        #define GOV_R_NUM 73
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,16,18,19,21,22,24,25,26,28,29,31,33,34,36,37,39,40,
             42,44,45,47,48,50,52,53,55,57,59,60,62,64,66,67,69,71,73,75,76,78,80,82,84,86,88,90,92,94,96,98,
             100,102,104,106,108,111,113,115,117,119,122,124,126        };
      #elif (VBATNOMINAL == 129) // 3S HV
        #define GOV_R_NUM 37
        static int8_t g[] = { 0,3,5,8,11,13,16,19,22,25,28,31,34,37,40,43,46,49,53,56,59,63,66,70,74,77,81,85,
             89,93,96,101,105,109,113,117,122,126         };
      #elif (VBATNOMINAL == 168) // 4S
        #define GOV_R_NUM 48
        static int8_t g[] = { 0,2,4,6,8,10,12,14,17,19,21,23,25,28,30,32,34,37,39,42,44,47,49,52,54,57,59,62,
             65,67,70,73,76,78,81,84,87,90,93,96,99,103,106,109,112,116,119,122,126    };
      #else
        #error "VOLTAGEDROP_COMPENSATION requires correction values which fit VBATNOMINAL; not yet defined for your value of VBATNOMINAL"
      #endif
      uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, GOV_R_NUM);
      for (i = 0; i < NUMBER_MOTOR; i++) {
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
      }
    }
  #endif



  
  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #ifndef MOTOR_STOP
        motor[i] = conf.minthrottle;
      #else
        motor[i] = MINCOMMAND;
      #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
  {
    static uint32_t lastRead = currentTime;
    uint16_t amp;
    uint32_t ampsum, ampus; // pseudo ampere * microseconds
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
        ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
        #if (LOG_VALUES >= 3)
          pMeter[i]+= ampus; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
          ampsum += ampus; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
      #endif
    }
    lastRead = currentTime;
  }
  #endif
}
