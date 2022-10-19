# PIC16 Digital Thermometer
Digital Thermometer using PIC microcontroller (PIC16F886) and MPLAB X IDE v5.40 - XC8-CC v2.30.

16x2 LCD Display (CGRAM Custom Character Handling), Pushbutton (Input with the Internal Pull-Up Resistor Enabled), LM50 Temperature Sensor, and Oversampled ADC Readings.

Celsius/Fahrenheit Measures (Mode Selected by a Pushbutton).

How to simulate it using Proteus 8:
- Build the Main Project (F11 - MPLAB X IDE);
- Open Digital_Thermometer.pdsprj (Proteus v8.9 SP2);
- Right-click U1 (PIC16F886) and click the 'Edit Properties' option (Ctrl + E);
- Browsing 'Program file:' select the generated hex file (user_folder/MPLABXProjects/project_name.X/dist/default/production/file_name.X.production.hex);
- Click the 'OK' button to close the 'Edit Component' window and click the play button to run the simulation;
- During simulation click the MODE pushbutton to change between Celsius/Fahrenheit.

A. Inácio Morais - anderson.morais@protonmail.com - (35) 99161-9878
