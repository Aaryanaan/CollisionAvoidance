# CollisionAvoidance
# Scissor Lift Collision Detection Module

### Overview  
This project, developed in collaboration with **Skanska’s Safety Innovation Team**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.  
The system uses **ultrasonic sensors** to provide 360° proximity coverage, delivering adaptive alerts to operators and improving situational awareness in dynamic construction environments.  

---

### Key Features  
- **2 Degrees of Detection:** two ultrasonic sensor readings for reliable distance measurement under varying noise and lighting conditions for 60 degrees vertical and 270 degrees horizontal per module.
- **Embedded Software:** Real-time Arduino firmware for continuous sensor polling, adaptive audio alerts, and signal filtering.  
- **Robust Design:** Built for durability, scalability, and cost efficiency — **45% cheaper** than initial prototype iterations.  
- **Data-Driven Development:** Design decisions guided by **Pugh matrices**, **scoring analyses**, and **peer-reviewed research**.  
- **Client Collaboration:** Co-developed with **3 other interns** through Duke's EGR101 course; presented deliverables to Skanska and other professional investors as part of a **client-facing engagement**.  

---

### Technical Stack  
- **Hardware:** VL53L0X ToF sensors, HC-SR04 ultrasonic sensors, Arduino Mega 2560, piezo buzzer, custom PCB  
- **Software:** Arduino IDE (C/C++), VSCode, PyGame & Conda (Visualization), Onshape & KiCad (mechanical & electrical design)  
- **Documentation:** Pugh screening matrices, design scoring tables, and technical memos written for client review  

---

### Results  
- Achieved **360° proximity coverage** and **40% reduction in blind-spot risk** in pilot testing.  
- Prevents potential damages worth **$50K–$100K per incident** with a per-unit build cost of **~$500**.  
- Demonstrated scalable, safety-first innovation with a **>10,000% ROI** on equipment protection.  

---

### Team  
- **Project Leads:** Aaryan Nanekar, Veer Bhatia, Pierson Jones, Jack Oakman  
- **Client Partner:** Skanska USA – Safety Innovation Team  

---

### Repository Structure
```
hardware-designs
├── circuit_schematic.kicad_pcb
├── enclosure_model.STL
firmware
├── main.ino
docs
├── pugh_matrix.pdf
├── TechnicalMemo1.pdf
└── TechnicalMemo2.pdf
```


---

### License  
This project is currently **private and under client collaboration**. Portions may be shared publicly for academic or demonstration purposes only.

---

### Acknowledgements  
Special thanks to **Skanska** and **Duke University’s EGR101 Course** for technical guidance, mentorship, and access to prototyping facilities.
