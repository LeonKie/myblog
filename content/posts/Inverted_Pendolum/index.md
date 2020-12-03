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

The inverted pendulum is one of the most popular demonstrations in system control. It has been researched since the 1960 [^1]and therefore a wide range of control systems have already been designed.

This projects is a symbiosis between this still highly complex balancing problem and a potential usage as a design element. Combing a balancing cart 🚗 with a desk lamp 💡 brings the extra challenge of feeding a, in this case 230V, cable throw various mountings and controlling it using a ordinary relay.

{{< admonition type=note title="Note" >}}

This blog post will give you a step by step **recreation guide** and highlights the the design choices I made as well as introduce you to the implementation of the **LQR Control Algorithm** I used to balance the Lamp.

{{< /admonition>}}

## Building

### Components

{{< admonition type=info title="List of used parts" >}}

 - [📟 Arduino UNO](https://www.ebay.de/itm/Arduino-UNO-komp-Board-ATmega328-CH340-Hochwertig/283648092280?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 - [⚡ L298N motor driver](https://www.ebay.de/itm/L298N-Motortreiber-Endstufe-Schrittmotor-Driver-Arduino-Modul-Board-Raspberry-P/252909515166?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 -  [⚙️ Geared DC-motor 39.6 mm, 15:1, 12 V DC](https://www.reichelt.de/getriebemotor-39-6-mm-15-1-12-v-dc-gm39-6-15-12v-p159640.html)
 - [🔌 Capsule Slip Ring AC 240V](https://www.ebay.de/itm/300Rpm-Capsule-Slip-Ring-6-Circuits-Wires-12-5mm-2A-AC-240V-Test-Equipment/283569124340?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 - [📏 Wdd35d4-5k Contious Angle Sensor](https://www.ebay.de/itm/WDD35D4-5K-Conductive-Plastic-Angular-Displacement-Sensor-Angle-Sensor-New/263908784245?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649) 
 - [🔌 1 Cannel Relay 5V/230V](https://www.ebay.de/itm/1-Kanal-Relais-5V-230V-Raspberry-Pi-Modul-Channel-Relay-für-Arduino-QITA/274108493382?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649)
 - [📏 Potentiometer 50k (linear)](https://www.amazon.de/gp/product/B07PC436ZD/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)


{{</admonition>}}

### Connecting Motor Driver Board

![Stepper Motor with L298N and Arduino Tutorial (4 Examples)](https://www.makerguides.com/wp-content/uploads/2019/05/l298n-motor-driver-with-stepper-motor-and-arduino-wiring-diagram-schematic-pinout.jpg)





### Connecting Potentiometer





### Integration of Relay 







## Control Algorithm

In the following section I will give a brief introduction to the theory of the LQR Controller and provide you with my recommended resources with help you determine the coefficients of the feedback regulator

### Mathematical representation

{{<figure src= "http://ctms.engin.umich.edu/CTMS/Content/InvertedPendulum/System/Modeling/figures/pendulum.png">}}


The simplified system consists of a inverted pendulum which is connected to a motorized card. Without further inspection we can   recognize that we are looking at a unstable system. If the cart isn't moved it is nearly impossible to balance the pendulum in the upright position. The objective is it now to design a feedback controller which accelerates the carte enough so that the angle $ \phi $ of the pendulum is close to 180°. 

However we first have to define the nonlinear model:



$$\begin{bmatrix}   a & b \\\\   c & d \end{bmatrix}$$

$$\begin{bmatrix} \dot{x} \\\\ \ddot{x} \\\\ \dot{\phi} \\\\ \ddot{\phi} \end{bmatrix} =\begin{bmatrix} 0 & 1 & 0 & 0 \\\\ 0 & \frac{-\left(I+m l^{2}\right) b}{I(M+m)+M m l^{2}} & \frac{m^{2} g l^{2}}{I(M+m)+M m l^{2}} & 0 \\\\ 0 & 0 & 0 & 1 \\\\ 0 & \frac{-m l b}{I(M+m)+M m l^{2}} & \frac{m g l(M+m)}{I(M+m)+M m l^{2}} & 0 \end{bmatrix}$$

\begin{bmatrix}{c}
x \\\\
\dot{x} \\\\
\phi \\\\
\dot{\phi}
\end{bmatrix}$$



{{< admonition type=example title="List of Variables" open= false >}}

- *M*				mass of the cart
- *m*				mass of pendulum
- *b*				coefficient of friction for cart
- *l*				length to pendulum center of mass
- *I*				mass moment of inertia of the pendulum
- *F*				force applied to the cart
- *x*				cart position coordinate
- *$$\theta$$*				pendulum angle from vertical (down)

{{< /admonition>}}





### LQR Controller



### Determine grains

### Determine system state variables

### Code



Regelungssysteme gibt es zu genüge in heutigen Applikationen der Robotik. Das Auto verfügt zum Beispiel um zigtausende solcher geregelten Systeme. Früher wurde die durch die Aktuation des Gaspedal noch mechanisch 1:1 der Winkel der [Drosselklappe](https://de.wikipedia.org/wiki/Drosselklappe_(Motor)) verändert um einen höheren Lustmassenstrom zu gewährleisten.

 Heutzutage in deiner Welt mit Automatikgetriebe und Spurhalteassistent müssen durch die den Input des Gaspedal aber weitaus komplexere Entscheidungen getroffen werden. Beim schnellem Drücken des Gaspedals[^2] muss wird vorzugsweise eher einen Gang mehr runtergeschaltet als bei langesamen Durchdrücken um maximales Drehmoment zu gewährleisten. Alle diese Regelungssystem sind meisten unsichtbar und je unerkennbare für die Benutzer desto besser wurde das Regierungssystem design. Das liegt daran dass die meisten Systeme Stabile sind. Wenn aber eine [Regelstrecke](https://de.wikipedia.org/wiki/Regelstrecke) instabil ist kommt das Potenzial heutiger Regler erst zum Vorschein.



{{< admonition type=tip title="This is a tip" >}}
- Das Ziel dieses Projekts ist es gut verstandene **Regelungssystem** zu verbinden mit neuen Designmöglichkeiten.

- Das inverierte Pendel soll ersetzt werden mir einer moderen Lampe 💡

  

  {{</admonition>}}



{{< figure src= "controlled-cart.gif" >}}







[^1]: http://web.mit.edu/klund/www/papers/Roberge1960.pdf
[^2]: 