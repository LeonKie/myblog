---
title: "InverterdPendelum"
date: 2020-08-31T23:38:56+02:00
draft: false
featuredImagePreview: "IMG_20200830_182322.jpg"
categories: [arduino, feedback control theory]
tags: [electronics, dc motor, L298n, lamp]
math: 
 enable: true
---

{{< youtube YQyPyLgG7x4 >}}

## Introduction

The inverted pendulum is one of the most popular demonstrations in system control, has been studied since the 1960[^1] and prime example at unis and other reseach institutes. However the inverted pendulum, which is nothing more than a pendulum that has the center of mass above the pivot point, yet has not particular use case. To change this once in for all, I build a **inverted pendulum lamp**, which can be used as a normal stylsh desk lanmp. *(The main incentive however wasn't so heroic. Honestly! It sounded just like a nice project idea!)* 

{{< admonition type=note title="Note" >}}

This blog post will give you a step by step **recreation guide** and highlights the the design choices I made as well as introduce you to the implementation of the **LQR Control Algorithm** I used to balance the Lamp. I will also mention the resources that i used to speed up the programming part at the the end of the post!

{{< /admonition>}}
## Building

### Components

{{< admonition type=info title="List of used parts" >}}

 - [📟 Arduino UNO](https://www.ebay.de/itm/Arduino-UNO-komp-Board-ATmega328-CH340-Hochwertig/283648092280?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 - [⚡ L298N motor driver](https://www.ebay.de/itm/L298N-Motortreiber-Endstufe-Schrittmotor-Driver-Arduino-Modul-Board-Raspberry-P/252909515166?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 -  [⚙️ Geared DC-motor 39.6 mm, 15:1, 12 V DC](https://www.reichelt.de/getriebemotor-39-6-mm-15-1-12-v-dc-gm39-6-15-12v-p159640.html)
 - [🔌 Capsule Slip Ring AC 240V](https://www.ebay.de/itm/12-5mm-300Rpm-6-Wires-CIRCUITS-2A-Capsule-Slip-Ring-Schleifring-240V-Dirigent-/232587637569)
 - [📏 Wdd35d4-5k Contious Angle Sensor](https://www.ebay.de/itm/WDD35D4-5K-Conductive-Plastic-Angular-Displacement-Sensor-Angle-Sensor-New/263908784245?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649) 
 - [🔌 1 Cannel Relay 5V/230V](https://www.ebay.de/itm/1-Kanal-Relais-5V-230V-Raspberry-Pi-Modul-Channel-Relay-für-Arduino-QITA/274108493382?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 - [📏 Potentiometer 50k (linear)](https://www.amazon.de/gp/product/B07PC436ZD/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)
 - [💡 E27 Fassung](https://www.amazon.de/DiCUNO-Vintage-Lampenfassung-Schraube-Gl%C3%BChbirne/dp/B07HRD6KGQ/ref=redir_mobile_desktop?ie=UTF8&aaxitk=ZDRvDTdnV76iI1MniJFRIQ&hsa_cr_id=1883828810402&ref_=sbx_be_s_sparkle_mcd_asin_1)


{{</admonition>}}


### Mechanical Setup
To accomplish this symbiosis between this still highly complex balancing problem and the potential usage as a design element, I first had to desing a balancing cart 🚗 with a pivoting E27 socket for the lamp 💡. This brings the extra challenge of feeding a, in this case 230V, cable throw various mountings and controlling it using a ordinary relay.

{{<figure src= "IMG_20200830_182322.jpg">}}

{{<admonition type=tip title="Design-choices">}}

### ⚙️ DV-Motor (first axel)
  - the L298N Motor Driver Module works up to 12V , without removing the jumper wire, therefore a 12V DV-Motor is a good choice for the setup
  - a DC-Motor with 300-440 RPM or with a 15:1 gear ratio and a high enought torque, 10Ncm in my case (but that much might not be necessary) was sufficient for my setup
  - ❗ be carefull with the maximum current! The allowed Amps for the L298N Motor Driver Module is 2A (more information [here](https://components101.com/modules/l293n-motor-driver-module#:~:text=Driver%20Chip%3A%20Double%20H%20Bridge,Logic%20Voltage%3A%205V))

### 📏 Contious Angle Sensor (second axel)
  - is used to monitor the position of the cart 🚗 relative to its starting position
  - the angle sensor works like a normal potentiometer but allows for more that 360° turning angle.
  - not the most reliable solution as it produced high noise in the zero orientation <br> => **Quick-fix:** use a capacitor as a low-pass filter between the ground and output voltage. 
  
### 📟 Arduino 
  - the L298 has a 5V regulator and can be used as the power supply (see [connection diagram](#connecting-motor-driver-board))
  - ❗ dont mount any high current consumer like the lamp 💡 or the motor ⚙ to the 5V output of the board it is not made for such high currents.

### 📏 Potentiometer 50k (linear)
  - used to detect the angel of the pendulum
  - additionally used as a mount for the pendulum

### 🔌 Capsule Slip Ring AC 240V
  - the lamp 💡 is powerd by a isolated electric circuit with 240V AC
  - the slip ring feeds the power thought the rotational pivot point
  - also used as mount for the pendelum

### 🔌 Relay
  - isolates the 12V DC and the 240V AC 
  - is triggerd by the arduino

{{</admonition>}}


### Connecting Motor Driver Board

![Stepper Motor with L298N and Arduino Tutorial (4 Examples)](https://www.makerguides.com/wp-content/uploads/2019/05/l298n-motor-driver-with-stepper-motor-and-arduino-wiring-diagram-schematic-pinout.jpg)






### Connecting Potentiometer


{{<figure src= "https://qph.fs.quoracdn.net/main-qimg-3b294f2e74073525723aacf5e7587b2b.webp">}}



### Integration of Relay[^3]

{{<figure src= "https://diyi0t.com/wp-content/uploads/2020/02/Relais-Module-Arduino-Uno-World_Steckplatine-1536x989.png" title= "Simple introduction to connect the relay to the arduino ">}}






## Control Algorithm

In the following section I will give a brief introduction to the theory of the LQR Controller and provide you with my recommended resources which help you determine the coefficients of the feedback regulator!

### Mathematical representation

{{<figure src= "http://ctms.engin.umich.edu/CTMS/Content/InvertedPendulum/System/Modeling/figures/pendulum.png">}}


The simplified system consists of a inverted pendulum which is connected to a motorized card. Without further inspection we can   recognize that we are looking at a unstable system. If the cart is not moved it is nearly impossible to balance the pendulum in the upright position. The objective is it now to design a feedback controller which accelerates the carte enough so that the angle $ \phi $ of the pendulum is close to 180°. 

However we first have to define the nonlinear model:

{{<figure src= "https://ctms.engin.umich.edu/CTMS/Content/InvertedPendulum/System/Modeling/html/InvertedPendulum_SystemModeling_eq14127640478467589374.png" >}}

{{<figure src= "https://ctms.engin.umich.edu/CTMS/Content/InvertedPendulum/System/Modeling/html/InvertedPendulum_SystemModeling_eq13622041353118727580.png" >}}


{{< admonition type=example title="List of Variables" open= false >}}

- *M*			 	mass of the cart
- *m*				mass of pendulum
- *b*				coefficient of friction for cart
- *l*				length to pendulum center of mass
- *I*				mass moment of inertia of the pendulum
- *F*				force applied to the cart
- *x*				cart position coordinate
- *$\theta$* pendulum angle from vertical (down)
- *u*       (input) here equivalent to the Force *F*

{{< /admonition>}}




### LQR Controller




### Determine grains
The script to determine the gains for the LQR Controller can be found [here](https://github.com/zjor/inverted-pendulum/blob/master/dc-motor/pendulum_lqr_control.py)!

### DC-Motor system identification
The LQR Controller takes as a input the Force *F* of the DC-Motor. Therefore the next step is to find the undelying DC-Motor model and estimate the corresponding system parameters. Then the arduino is able to change the speed of the engine using the PWM signal.

We can model a standard DC-Motor using the following equation:
$$
\frac{d w}{d t}\left(J+m r^{2}\right)+w\left(B+\frac{k^{2}}{R}\right)=\frac{k U}{R}
$$

Simplifying this further:

{{<figure src= "https://hackster.imgix.net/uploads/attachments/1020507/codecogseqn_vWvU4P629o.png?auto=compress%2Cformat&w=680&h=510&fit=max" >}}

To determine the parameters a,b,c we can record several velocity curves with different input voltages. Using the following plot and theses scripts  [[dc_motor_lqr_control.py](https://github.com/zjor/inverted-pendulum/blob/master/dc-motor/dc_motor_lqr_control.py),[estimate_params.py](https://github.com/zjor/inverted-pendulum/blob/master/dc-motor/estimate_params.py)] we can now find the exact model parameters!

{{<figure src= "Engine-estimation.png" title="">}}

*Credits to zjor for creating the scripts[^4]*


## Conclustion



## Code

The code is based on a existing arduino project[^4]

```arduino
#define A 9   
#define B 1.0 
#define C 2.1 

#define Kth 61.20847945 
#define Kw  10.42165907
#define Kx  16.2303454
#define Kv  13.43669139 

#define Ki  0
#define PI 3.1415926535897932384626433832795


//define pins
int speed = 9;
int front = 6;
int back  = 7;
int lamp = 2;


//define constans
float wheelCirc = 141.37;
float THETA_THRESHOLD = PI / 8;

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;


//init States
float x=0;
float d_x;
float phi;
float d_phi;
float phi_sum=0;

//usefull variables
float pos_last1;
float pos_last2;
float pos_last3;
float pos_last4;
float last_phi;

float control, u, dt , v_soll;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(speed, OUTPUT);
  pinMode(front, OUTPUT);
  pinMode(back, OUTPUT);
  pinMode(lamp, OUTPUT);
  digitalWrite(lamp,HIGH);
  digitalWrite(front, LOW);
  digitalWrite(back, LOW);
  
  pos_last1= mapf(analogRead(A1),0,1023,0,wheelCirc)/1000;
  pos_last2=pos_last1;
  pos_last3=pos_last1;
  pos_last4=pos_last1;
  last_phi= mapf(analogRead(A0)-512+4 , -512, 512, -PI ,PI );
  lastTimeMicros = 0L;
  
  //Debugging

  //Serial.print("x");Serial.print("\t");
  //Serial.print("dx");Serial.println("\t");
  //Serial.print("Kv*d_x");Serial.print("\t");
  //Serial.print("Kth*phi");Serial.print("\t");
  //Serial.print("Kw*d_phi");Serial.print("\t");
  //Serial.print("control");Serial.print("\t");
  //Serial.print("u");Serial.println("\t");
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void driveMotor(float pwm_out){
  analogWrite(speed, fabs(pwm_out));
    if (pwm_out > 0) {
      digitalWrite(front, HIGH);
      digitalWrite(back, LOW);
    }else if (pwm_out < 0) {
      digitalWrite(front, LOW);
      digitalWrite(back, HIGH);
    }else{
      digitalWrite(front, LOW);
      digitalWrite(back, LOW);
      
    }
  
}


boolean isControllable(float phi) {
  return fabs(phi) < THETA_THRESHOLD;
}

float saturate(float v, float maxValue) {
  if (fabs(v) > maxValue) {
    return (v > 0) ? maxValue : -maxValue;
  } else {
    return v;
  }
}


float derivative(float x_new, float x_old, float dt){
  
  float d_temp= x_new - x_old;
  float d_pos = 0;
  
  if (fabs(d_temp) < fabs(d_temp+wheelCirc/1000) and (fabs(d_temp) < fabs(d_temp-wheelCirc/1000))){
    d_pos= d_temp;
  }else if (fabs(d_temp+wheelCirc/1000) < fabs(d_temp-wheelCirc/1000)){
    d_pos= (d_temp+wheelCirc/1000);
  }else {
    d_pos= (d_temp-wheelCirc/1000);
  }
  
  if (fabs(d_pos/dt)>1.1){
    d_pos= -d_x*dt;
  } 
  
  return - d_pos/dt;
}

void updateStates(float dt){
  

  //read sensor input (Angle of wheel)
  float sensor_x = analogRead(A1);
  float pos_new =mapf(sensor_x,0,1023,0,wheelCirc)/1000;

  //smoothing the signal input 
  float pos_avg= (pos_last1+pos_last2+pos_last3+pos_last4)/4; 
  float d_x_2 = derivative(pos_new, pos_last2,2*dt);
  float d_x_3 = derivative(pos_new, pos_last3,3*dt);
  d_x = (d_x_2+d_x_3)/2;
  
  //update state variables
  x = x + dt*d_x;
  pos_last4=pos_last3;
  pos_last3=pos_last2;
  pos_last2=pos_last1;
  pos_last1=pos_new;
  
  //read sensor phi
  float sensor_phi = analogRead(A0) - 512;
  phi = mapf(sensor_phi+8 , -512, 512, -PI, PI ); // map sensor input to correct mean an variance 
  

  d_phi=(phi-last_phi)/dt;
  last_phi= phi;
  
  phi_sum=phi_sum+dt*phi;
  if (isControllable(phi)){
    phi_sum=0;
  }
  
  //Debugging

  //Serial.print(sensor_x, 4);Serial.print("\t");
  //Serial.print(sensor_x, 4);Serial.print("\t");
  //Serial.print(pos_new, 4);Serial.print("\t");
  //Serial.print(d_x*1000, 4);Serial.println("\t");
  //Serial.print(d_x_2*1000, 4);Serial.print("\t");
  //Serial.print(d_x_3*1000, 4);Serial.println("\t");
  //Serial.print(d_x_4*1000, 4);Serial.println("\t");
  //Serial.print(phi, 4);Serial.print("\t");
  //Serial.print(d_phi, 4);Serial.print("\t");

}

// the loop routine runs over and over again forever:
void loop() {
  now = micros();
  dt = (1.0*(now - lastTimeMicros))/ 1000000;
  updateStates(dt);
  
  
  
  
  if (isControllable(phi)) {
    digitalWrite(lamp, LOW);
    
    control = (Kx * x + Kv * d_x + Kth * phi + Kw * d_phi);
    u = (control + A * d_x + copysignf(C, d_x)) / B;
    u = 255.0 * u / 12;
    
    v_soll = saturate(u, 254);
    driveMotor(v_soll);
  } else {
    v_soll= 0;
    driveMotor(v_soll);
    digitalWrite(lamp, HIGH);
  }
  



  lastTimeMicros = now;
  delay(5);        // delay in between reads for stability
}

```



{{< figure src= "controlled-cart.gif" >}}





## Links

[^1]: http://web.mit.edu/klund/www/papers/Roberge1960.pdf
[^2]: https://components101.com/modules/l293n-motor-driver-module#:~:text=Driver%20Chip%3A%20Double%20H%20Bridge,Logic%20Voltage%3A%205V
[^3]: https://diyi0t.com/relay-tutorial-for-arduino-and-esp8266/
[^4]: https://create.arduino.cc/projecthub/zjor/inverted-pendulum-on-a-cart-199d6f