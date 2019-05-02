/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "Motor.h"


ros::NodeHandle nh;
Timer t,t1;


//Hardware Intialization
DigitalOut myled(LED1);
Motor top(p26, p24, p25); // pwm, fwd, rev
Motor bottom(p21, p23, p22); // pwm, fwd, rev

bool done = 0;
char donepickup[5] = "done";


std_msgs::String str_msg;


//Timer base drop 
void drop() {
    Timer t;
    t.start();
    
    while (t.read()<4) { 
        top.speed(1);
        //pc.printf("%f\n",t.read());    
    }
    t.stop();
    top.speed(0); 
}    


//Timer base pick
void pick() {
    Timer t1;
    t1.start();
    
    while (t1.read()<4) { 
        top.speed(-1);
        //pc.printf("%f\n",t.read());    
    }
    t.stop();
    top.speed(0); 
}  





//mesagge call back function that interrupts when int "1" is received 
void messageCb(const std_msgs::Int32& msg){
    if (msg.data == 1)
    {
    //myled = !myled;   // blink the led
    drop(); 
    wait(2);
    //pick();
    done = 1;
    }
    
    else {
        done = 0; 
    }
}

 //establish ros node chatter and callback
ros::Publisher chatter("/state_machine/confirmation", &str_msg);
ros::Subscriber<std_msgs::Int32> sub("/state_machine/forklift", &messageCb);

int main() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);

    while (1) {
        str_msg.data = donepickup;
        if (done == 1)
        {
            chatter.publish( &str_msg );
            wait(3);
            pick(); 
            done = 0;
        }
        nh.spinOnce();
        wait_ms(1000);
    }
}