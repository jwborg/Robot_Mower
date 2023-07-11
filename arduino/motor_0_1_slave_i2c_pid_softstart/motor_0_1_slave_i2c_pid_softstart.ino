// 
// Drives two DC motors using PWM duty control by Arduino. Each motor is equiped with an IR speed 
// sensor. Approximate 40Hz at full speed. A PID controller is used to control the number
// of rotations of the motors, either to keep in sync or to control to difference in rotations.
// The driver is controlled by I2C interface from a Master (RPI). The following commands in 
// are supported:
//
// d,[steps],[step_difference],[duty]]          Drive command, negative steps indicate reverse.
// z,[steps],0,[duty]]                          Zero Turn, negative steps indicate reverse.
// t,[Kp],[Ki],[Kd]                             Tuning paramaters for PID
// c                                            Command status
// s                                            Show status
// r                                            Show all registers
// 
// The following replies are returned to the master upon receipt
// $ok                                              Command received
// $bad                                             Invalid command
//
// Hardware lines
// READY Output pin will be high after completion of the Drive command
// ESD input pin is polled, if HIGH both motors will stop instantanously.
//
// This module can be tested using the serial monitor, however the serial.println affect the timing, and 
// should never be used in interrupt service routines. 
//
// 18/8/2022 - Rev 1.1
//             MTR A and B PWN Pin assignments changed to work with PCB board rev 1.0
//             PCB rev 1.0 has MTR A DIR connected to D6 and MTR A PWM to D4, but D4 is not PWM capable
//             The MTR A cable DIR and PWM were reversed to avoid changes on PCB board rev 1.0: 
//             MTR A Cable reversed DIR and PM pins: 
//                MTR A DIR D6 -> D4
//                MTR A PWM D4 -> D6
//             Software changes:  
//                int motor0_pwm_pin = D5 -> D6;
//                int motor1_pwm_pin = D6 -> D5;
//  18/8/2022 - Rev 1.1 
//            ESD active state changed to HIGH
//  29/9/2022 - Rev 1.2
//            Changed READY pin to D10 to match PCN
//            Used built-in LED on D13 to indicate ESD status. ESD actived = LED on.
//
//   1/1/2023 Rev 1.3 
//            Increased the soft start minimum duty from 30% to 70%. In grass the resistance is too great
//            to start driving, hence when zero turning the desired angle doe not get reached, especially for 
//            small turns.
//
#include <math.h>
#include <string.h>
#include <Wire.h>
#include "PID.h"        // read from project directory
#include "PID.c"

// revision
#define REVISION "v1.3 (PWM signal fix for PCB rev 1.0)"

// PID Controller parameters
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.0f
#define PID_TAU 0.01f
#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define SAMPLE_TIME_S 0.01f

// create an instance of the PID controller
PIDController pid = { PID_KP, PID_KI, -PID_KD,
                      PID_TAU,
                      PID_LIM_MIN, PID_LIM_MAX,
                      SAMPLE_TIME_S };

// Arduino is slave
#define SLAVE_ADDRESS 0x08 

// status codes
#define OK 0
#define ESD 1
#define ERROR 2

// motor 0
float duty_0;                   // duty 0-100%
int pwm_signal_0;               // pwm signal 0-4095
volatile int motor_count_0;     // motor0 rotation count
//int motor0_pwm_pin = 5;         // pwm pin capable of 960Hz
int motor0_pwm_pin = 6;         // pwm pin capable of 960Hz
int motor0_dir_pin = 4;         // direction of motor0 pin
int motor0_int_pin = 2;         // rotation counter pin
int motor0_dir;                 // direction of motor0

// motor 1
float duty_1;                   // duty 0-100%
int pwm_signal_1;               // pwm signal 0-4095
volatile int motor_count_1;     // motor0 rotation count
// int motor1_pwm_pin = 6;         // pwm pin capable of 960Hz
int motor1_pwm_pin = 5;         // pwm pin capable of 960Hz
int motor1_dir_pin = 7;         // direction of motor1 pin
int motor1_int_pin = 3;         // rotation counter pin
int motor1_dir;                 // direction of motor1

// debounce related
unsigned long now_time;
unsigned long intr_time_0, intr_time_1;
unsigned long last_intr_time_0, last_intr_time_1;
unsigned long debounce_time = 5;  // debounce time in ms (blue-red 8ms between pulses, red-green 20)



// emergency shutdown
int esd_pin = 8;                // by polling pin
int esd_led = 13;               // When LED is on, ESD was triggered

// ready 
int ready_pin = 10;             // to tell the PI, I'm ready 


// registers layout
struct s_reg {
  //                    Description             No_Bytes  Pi          R/W   by command
  char  cmd;         // command character       1         char        R/W   
  int   no_steps;    // number of steps         2         short int   W     Drive
  int   steer_sp;    // steer setpoint in steps 2         short int   W
  int   duty;        // motor duty 0-100%       2         short int   W
  float Kp;          // K proportional          4         float       R/W   Tune and Show
  float Ki;          // K integral              4         float       R/W
  float Kd;          // K derivative            4         float       R/W
  int   m0_count;    // motor 0 count           2         short int   R     Show
  int   m1_count;    // motor 1 count           2         short int   R
};

// struct 'rgstr' and byte array 'reg' overlap and use the same memory; therefore
// the data can be accessed by the declared variables in struct s_reg and the
// byte data in 'reg'. This works because the Arduino and the Raspberry are both little endian,
// both use IEEE 75432 floating points and the integers used by the Pi are selected to 
// be using the same number of bytes
union u_reg {
  s_reg   rgstr;               // the struct
  byte    reg[sizeof(s_reg)];  // byte data to send and receive  
};

// create an instance of u_reg
u_reg ur;
int reg_index = 0;                // index into byte registers

// offset into byte registers (always the first byte received from master for both read and write)
int offset;







// ISR called upon receipt of I2C data from the master. This can be:
// 1. A register offset plus a block of data from write_i2c_block_data
// 2. A register offset from read_i2c_block_data. This will be followed by 
//    a sendData request initiated by the master.
void receiveData(int byteCount){

  int byte_rcvd;
  int rcvd_byte_cnt;

  //Serial.println( (String) "byteCount = " + byteCount);     // only for debugging; don't do in ISR
  
  rcvd_byte_cnt  = 0;       // no bytes received yet
  reg_index = 0;
  while(Wire.available()) {

      // read a byte from the receive buffer
      byte_rcvd = Wire.read();

      // Master's write_i2c_block_data and read_i2c_block_data: the first character is the offset into
      // the registers. read_i2c_block_data will then raise OnRequest event.
      if (rcvd_byte_cnt == 0) {
          offset = byte_rcvd;
          //Serial.println( (String) "Got offset: " + offset);    // only for debugging; don't do in ISR
      }

      // write_i2c_data only: any following bytes is data, so write into the correct registers
      if (rcvd_byte_cnt > 0) {
          ur.reg[reg_index + offset] = byte_rcvd;
          reg_index++;
      } 
      
      // track number of bytes received
      rcvd_byte_cnt++;
  }
}




// ISR called when the master is requesting data. The master will have sent the offset first using requestData
void sendData(){
    // the offset was received by a receive event 
    // wire.write apparently only sents the number of bytes requested, size at the maximum expected 
    Wire.write(&ur.reg[offset], sizeof(s_reg));          
    
    //Serial.println((String) "data sent from offset " + offset);    // only for debugging; don't do in ISR
}





// interrupt service routine for motor 0 rotation counter
void isr_motor0()          
{  
  // debounce time is set to just less than the priod to stop counting spurious signals
  intr_time_0 = millis();
  if (intr_time_0 - last_intr_time_0 > debounce_time)
    {            
    motor_count_0++;
    last_intr_time_0 = intr_time_0;
    }
}

// interrupt service routine for motor 0 rotation counter
void isr_motor1()          
{  
  intr_time_1 = millis();
  if (intr_time_1 - last_intr_time_1 > debounce_time)
    {            
    motor_count_1++;
    last_intr_time_1 = intr_time_1;
    }
}








// function used to drive the motors
int move(int no_steps, float steer_sp, int turn_on_spot, float req_duty_sp) {

  //float dt = 0.5;      // the cycle time of this algorithm in seconds
  unsigned long time;
  
  
  float count_diff_sp = 0.0;
  float delta_count; 
  float duty_adj;
  float max_duty = 100.0;
  float min_duty = 50.0;                  // at 30% does not overcome resistance to start in grass
  float ramp_rate = 0.4;                  // adjust accordingly, serial println have large impact
  int ramp_steps = 0;                     // the number of steps used for ramping up 
  int ramping_up = 1;                     // initially ramping_up = True
  float duty_sp;                          // the current duty set point
  int esd_state;
  int pwm_direction_0, pwm_direction_1;
  int status = OK;
  
  
  
  // tell the PI we are working
  digitalWrite(ready_pin, LOW);
  
  // reset the motor counters
  motor_count_0 = 0;
  motor_count_1 = 0;

  // new drive command, clear the PID history
  PIDController_Init(&pid);
  
  // determine the rotation direction of the motors. 
  if (! turn_on_spot) {
      // both motors in the same direction; straight or at angle
      if (no_steps  > 0) {
            motor0_dir = 0;            // both motors forward
            motor1_dir = 0;
      } else {
            motor0_dir = 1;            // both motors backward
            motor1_dir = 1;
      }
  } else
      // motors in reverse direction; turning on the spot
      if (no_steps  > 0) {
            motor0_dir = 0;            // clockwise
            motor1_dir = 1;
      } else {
            motor0_dir = 1;            // counter clockwise
            motor1_dir = 0;
      }
  // set the motor rotation direction
  digitalWrite(motor0_dir_pin, motor0_dir);
  digitalWrite(motor1_dir_pin, motor1_dir);
   
  // now that the direction of the motors has been set, the steps are always positive    
  no_steps = abs(no_steps);
  
  // start with equal minimum duty for both motors as we are ramping up
  duty_sp = min_duty;
  
  // until the required number of steps has been reached
  while ( (motor_count_0 < no_steps) && (motor_count_1 < no_steps) ) {
    
    // check if Emergency Shutdown switch was activated, break from while loop
    esd_state = digitalRead(esd_pin);
    if (esd_state == HIGH) {

      status = ESD;
      Serial.println((String) "Emergency Shutdown operated");
      
      // and switch the ESD LED on
      digitalWrite(esd_led, esd_state);    
      break;
    }

    // increase the ramp rate until req_duty_sp is reached or the halfway point is reached
    if (ramping_up & (duty_sp < req_duty_sp)) {

      // do not increase if we are over the halfway mark
      if (motor_count_0 < (no_steps/2)){
          //Serial.println((String) "increase ramp rate");
          duty_sp = duty_sp + ramp_rate;     // ramp-up until we are at req_duty_sp
          duty_sp = min(duty_sp, req_duty_sp);   // limit to req_duty_sp

          // keep track how many steps are required to ramp-up; a similar amount will be required to ramp down
          ramp_steps = motor_count_0;
        } else {
          //Serial.println((String) "ramp-up switched off 1 " + ramp_steps);
          ramping_up = 0;
      }

      // if the requested duty_sp is reached
      if (duty_sp >= req_duty_sp) {
        ramping_up = 0;
        //Serial.println((String) "ramp-up switched off 2 " + ramp_steps);
      }
      
      
    }

    // when ramping down needs to start 
    if (motor_count_0 > (no_steps - ramp_steps)) {
      //Serial.println((String) "decrease ramp rate");
      duty_sp = duty_sp - ramp_rate;     // ramp-down until we are at min_duty
      duty_sp = max(duty_sp, min_duty);   // limit to min_duty
    }

    //Serial.println((String) "duty_sp = " + duty_sp + " m0 " + motor_count_0 + " m1 " + motor_count_1);
    
    // calculate the difference in rotation between the 2 motors
    delta_count = motor_count_0 - motor_count_1;

    // save the motor counts for updating the master
    ur.rgstr.m0_count = motor_count_0;
    ur.rgstr.m1_count = motor_count_1;
                    
    // control to minimize the motor distance difference
    PIDController_Update(&pid, steer_sp, delta_count);
    duty_adj = pid.out;
    
    // adjust the duty setpoint depending on the duty limits in case the duty of
    // either motor needs to be adjusted if progress of both motors is imbalanced
    if (duty_sp + abs(duty_adj) > max_duty)
    {
        duty_sp = max_duty - abs(duty_adj);
    } else {
        if (duty_sp - abs(duty_adj) < min_duty)
        {
          duty_sp = min_duty + abs(duty_adj);
        }
    }        
    duty_0 = duty_sp + duty_adj;
    duty_1 = duty_sp - duty_adj;   

    // limit the calculated duties
    duty_0 = max( min(duty_0, max_duty), min_duty);
    duty_1 = max( min(duty_1, max_duty), min_duty);
  
    // map it to the range of the duty 0-100%
    pwm_signal_0 = map(duty_0, 0, 100, 0, 255);
    pwm_signal_1 = map(duty_1, 0, 100, 0, 255);

    // power the motors 
    analogWrite(motor0_pwm_pin, pwm_signal_0);
    analogWrite(motor1_pwm_pin, pwm_signal_1);
  
    // debug message
    Serial.println((String) "Duty = " + duty_0 + " m0 = " + motor_count_0 + " " + duty_1 + " m1 = " + motor_count_1);
  
    // next cycle in <dt> second
    delay((int) (SAMPLE_TIME_S * 1000.0));
    
  }

  // switch both motors off, don't reset the counters yet, they are still spinning
  analogWrite(motor0_pwm_pin, 0);
  analogWrite(motor1_pwm_pin, 0);

  // report the final count
  ur.rgstr.m0_count = motor_count_0;
  ur.rgstr.m1_count = motor_count_1;
  Serial.println((String) "m0 = " + motor_count_0 + " m1 = " + motor_count_1);
  Serial.println((String) "$ok");
  
  // signal the Arduino is ready again
  digitalWrite(ready_pin, HIGH);
  return(status);
}



void setup() {
    
  // setup the serial console for debugging messages
  Serial.begin(9600);
  Serial.println((String) "Double motor - I2C Slave - Ramp " + REVISION);

  // setup I2C communication with RPI Master
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // set up the motor control pins
  pinMode(motor0_pwm_pin, OUTPUT);
  pinMode(motor0_dir_pin, OUTPUT);
  pinMode(motor1_pwm_pin, OUTPUT);
  pinMode(motor1_dir_pin, OUTPUT);

  // initialize the debounce time
  last_intr_time_0 = millis();
  last_intr_time_1 = millis();

  // setup pin 2 as interrupt for motor 0 rotation counter
  attachInterrupt(digitalPinToInterrupt(motor0_int_pin),isr_motor0,FALLING); 
  
  // setup pin 3 as interrupt for motor 1 rotation counter
  attachInterrupt(digitalPinToInterrupt(motor1_int_pin),isr_motor1,FALLING); 

  // set up the registers
  ur.rgstr.Kp = pid.Kp;
  ur.rgstr.Ki = pid.Ki;
  ur.rgstr.Kd = pid.Kd;

  // setup pin for Emergency shutdown for polling (Uno has only 2 interrupts)
  pinMode(esd_pin, INPUT_PULLUP);

  // setup pin for LED to show ESD state, this will tell if the ESD was activated
  pinMode(esd_led, OUTPUT);

  // setup pin for ready signal, this will tell the Pi the Arduino is ready
  pinMode(ready_pin, OUTPUT);
  digitalWrite(ready_pin, HIGH);    // Arduino is ready

}



void loop() {

    int status;
    int no_steps; 
    float duty_sp;
    float steer_sp;
    int esd_state; 

    // show the correct ESD state
    esd_state = digitalRead(esd_pin);
    digitalWrite(esd_led, esd_state);    

    // only commands that write to the slave need coding. All commands that read from the slave,
    // read directly from the registers, which are updated by the motor driver.
                                                                   
    //  drive command
    if(ur.rgstr.cmd == 'd'){
 
        no_steps = ur.rgstr.no_steps;
        steer_sp = ur.rgstr.steer_sp;
        duty_sp  = ur.rgstr.duty;
 
        Serial.println((String) "drive " + no_steps + " " + steer_sp + " " + duty_sp);
        status = move(no_steps, steer_sp, 0, duty_sp);
        Serial.println((String) "drive status = " + status);

        // clear the command to indicate completed
        ur.rgstr.cmd = 0x0;
    }
    
    // zero turn command
    else if (ur.rgstr.cmd == 'z'){
                                                            
        no_steps = ur.rgstr.no_steps;
        steer_sp = 0;
        duty_sp  = ur.rgstr.duty;
        float steer_sp = 0.0;
        Serial.println((String) "zero " + no_steps + " " + steer_sp + " " + duty_sp);
        status = move(no_steps, steer_sp, 1, duty_sp);
        Serial.println((String) "zero turn status = " + status);
        
        // clear the command to indicate completed
        ur.rgstr.cmd = 0x0;
    }
    // tune command
    else if(ur.rgstr.cmd == 't'){
                                                           
        
        pid.Kp = ur.rgstr.Kp;
        pid.Ki = ur.rgstr.Ki;
        pid.Kd = ur.rgstr.Kd;
        Serial.println((String) "Kp = " + pid.Kp + " Ki = " + pid.Ki + " Kd = " + pid.Kd);

        // clear the command to indicate completed
        ur.rgstr.cmd = 0x0;

    } 
    // put in a small delay
    delay(200);
    
}
