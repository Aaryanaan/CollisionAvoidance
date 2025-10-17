# CollisionAvoidance
# Scissor Lift Collision Detection Module

### Overview  
This project, developed in collaboration with **Skanska’s Safety Innovation Team**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.  
The system combines **Time-of-Flight (ToF)** and **ultrasonic sensors** to provide 360° proximity coverage, delivering adaptive alerts to operators and improving situational awareness in dynamic construction environments.  

---

### Key Features  
- **Multi-Sensor Fusion:** Combines ToF and ultrasonic readings for reliable distance measurement under varying noise and lighting conditions.  
- **Embedded Software:** Real-time Arduino firmware for continuous sensor polling, adaptive audio alerts, and signal filtering.  
- **Robust Design:** Built for durability, scalability, and cost efficiency — **45% cheaper** than initial prototype iterations.  
- **Data-Driven Development:** Design decisions guided by **Pugh matrices**, **scoring analyses**, and **peer-reviewed research**.  
- **Client Collaboration:** Co-developed with **3 other interns** through Duke’s Embedded Systems & CAD Design Program; presented deliverables to Skanska as part of a **client-facing engagement**.  

---

### Technical Stack  
- **Hardware:** VL53L0X ToF sensors, HC-SR04 ultrasonic sensors, Arduino Mega 2560, piezo buzzer, custom PCB  
- **Software:** Arduino IDE (C/C++), MATLAB (signal analysis), SolidWorks / KiCad (mechanical & electrical design)  
- **Documentation:** Pugh screening matrices, design scoring tables, and technical memos written for client review  

---

### Results  
- Achieved **360° proximity coverage** and **40% reduction in blind-spot risk** in pilot testing.  
- Prevents potential damages worth **$50K–$100K per incident** with a per-unit build cost of **~$500**.  
- Demonstrated scalable, safety-first innovation with a **>10,000% ROI** on equipment protection.  

---

### Team  
- **Project Lead:** Aaryan Nanekar  
- **Collaborators:** Veer Bhatia, Pierson Jones, Jack Oakman  
- **Client Partner:** Skanska USA – Safety Innovation Team  

---

### Repository Structure 
/hardware-designs
┣ circuit_schematic.kicad_pcb
┣ enclosure_model.sldprt
/firmware
┣ main.ino
/docs
┣ pugh_matrix.pdf
┣ technical_memo_v2.pdf
