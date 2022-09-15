/* This software shall enable the measurement of NTC-thermistor and 
pressure-values for the "gas-mouse" contraption by Mario Gaß.
Code written by Bernhard Klampfl */

#include <PID_v1.h>

//// User settings parameters ////

double Setpoint_block = 40;    // Temperature of block for the Preheating stage (°C)
double Setpoint_box  = 40;     // Temperature of box for the Preheating stage (°C)

// safety feature to limit the maximum PWM output

int max_block_PWM = 30; 
int max_box_PWM = 0;      
   
// Calibration parameters for 0-4 VDC 
// y = k*x + d
// y = real voltage
// x = (wrongly) measured voltage

float k = 0.907535;         
float d = 0.731423;         

// Setup Raspberry pico I/O parameters

int NTC_0 = 26;                     // GP26 - block
int NTC_1 = 27;                     // GP27 - box
int p_0 = 28;                       // GP28 - pressure
int PIN_BLOCK = 0;                  // GP0 
int PIN_BOX = 1;                    // GP1 - not used
int PIN_IN = 2;                     // GP2
int PIN_VAC = 3;                    // GP3
int PIN_PROP = 4;                   // GP4
int VAC_BUTTON = 5;                 // GP5 - vacuum button pin
int LED_FLASH = 25;                 // GP25 - indicator LED

// Data Output Variables
const bool DataRead_Excel = false;  // Variable for controlling the data Output format.
									// True:  MS Excel Data Streamer
									// False: Arduino IDE Serial Monitor
const bool Monitor_Output = true;   // Should the Output PWM-Signal byte be monitored in

// PID Variables and PID Instances
double Input0;
double Input1;
double Out0, Out1;
double Kp0 = 1.0, Ki0 = 1.0, Kd0 = 1.0;     // block PID Coefficients
double Kp1 = 1.0, Ki1 = 1.0, Kd1 = 1.0;         // box System PID Coefficients
int Output0, Output1;

PID myPID0(&Input0, &Out0, &Setpoint_block, Kp0, Ki0, Kd0, DIRECT);
PID myPID1(&Input1, &Out1, &Setpoint_box, Kp1, Ki1, Kd1, DIRECT);

// Utility Time Variables
float Total_Time_min = 0;                         // Time since activation. Millis() in minutes (min)
float Time_min = 0;                           //          -||-           (min)

// other global variables
float voltage_cal = 0.0;        // voltage value after calibration calculation in V
double T_0, T_1;                 // calculated NTC-temperatures in °C
float p_mbar = 0.0;             // calculated pressure values in mbar
bool IN = false;
bool VAC = false;
bool PROP = false;
int sample_number = 200;


void setup()
{
    Serial.begin(9600);
    pinMode(PIN_IN, OUTPUT);
    pinMode(PIN_VAC, OUTPUT);
    pinMode(PIN_PROP, OUTPUT);
    pinMode(VAC_BUTTON, INPUT_PULLUP);
    pinMode(LED_FLASH, OUTPUT);
    
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
}

void loop()
{   
    digitalWrite(LED_FLASH, HIGH);
    v_probing(p_0, sample_number);
    digitalWrite(LED_FLASH, LOW);
    RunPID();

    
    if (digitalRead(VAC_BUTTON) == LOW) 
    {
    digitalWrite(PIN_VAC, HIGH);
    VAC = true;
    digitalWrite(PIN_IN, LOW);
    IN = false;
    digitalWrite(PIN_PROP, LOW);
    PROP = false;
    }
        
    else if (T_0 < 43 && T_0 > 37)
    {
        digitalWrite(PIN_VAC, LOW);
        VAC = false;
                                                                                                          
        if (p_mbar <= 2000)
        {   
            sample_number = 100;
            digitalWrite(PIN_IN, HIGH);
            IN = true;
            digitalWrite(PIN_PROP, LOW);
            PROP = false;
        }

        else if (p_mbar > 2500)
        {   
            sample_number = 100;
            analogWrite(PIN_PROP, 188);
            PROP = true;
            digitalWrite(PIN_IN, LOW);
            IN = false;
        }

        else if (p_mbar > 2300)
        {   
            sample_number = 1000;
            analogWrite(PIN_PROP, 187);
            PROP = true;
            digitalWrite(PIN_IN, LOW);
            IN = false;
        }

       
        else if (p_mbar <= 2300 && p_mbar >= 2000)
        {   
            sample_number = 1000;
            digitalWrite(PIN_PROP, LOW);
            PROP = false;
            digitalWrite(PIN_IN, LOW);
            IN = false;
        }
    }
    else
    {
        sample_number = 100;
        digitalWrite(PIN_VAC, LOW);
        VAC = false;
        digitalWrite(PIN_PROP, LOW);
        PROP = false;
        digitalWrite(PIN_IN, LOW);
        IN = false; 
    }
    output();
 
    delay(5);
}


float ReadTemp(int address)               // Function for getting Temperatures from the NTC
{
   
    //Settings for NTC-Resistor
    int sample_number = 25;                     // number of samples taken
    int counter = 0;
    int bitwertNTC = 0;
    long widerstand1 = 100000;                   //Widerstand des NTC-Widerstands in Ohm
    int bWert = 3950;                           // B- value of the NTC
    double widerstandNTC = 0;
    double kelvintemp = 273.15;                // 0°Celsius in Kelvin
    double Tn = kelvintemp + 25;                 //Temperature in Kelvin
    double TKelvin = 0;                     //calculated Temperature in °C
    double TempC = 0;  

    //////NTC-Calculation_START/////////

    // read analog value 
    while (counter < sample_number) 
    {
        bitwertNTC += analogRead(address); 
        counter++;
        delay(5);
    }
    bitwertNTC = bitwertNTC/sample_number;    
    // calculate resistance of the NTC
    widerstandNTC = widerstand1*(((double)bitwertNTC/1024)/(1-((double)bitwertNTC/1024))); // eigtl Bitwert*Vin/1024 - kürzt sich aber im Bruch weg
    // calculate Temperature in Kelvin                                  
    TKelvin = 1/((1/Tn)+((double)1/bWert)*log((double)widerstandNTC/widerstand1));
    // Temperature in °C                                        
    TempC=TKelvin-kelvintemp;                   
    counter = 0;
    //////NTC-Calculation_END/////////

    return TempC;

} // End of ReadTemp()

void RunPID()
{
    T_0 = ReadTemp(NTC_0);
    T_1 = ReadTemp(NTC_1);

    Total_Time_min = float(millis()) / 60 / 1000;  // Time since Arduino turned on or last reset

    //Calculate Output for the Inlet Transfer Heating System
    Input0 = T_0;                    // Input for myPID0 is TempC0.
    myPID0.Compute();                    // Calculate Output0.
    Output0 = round(Out0);
    Output0 = Output0*max_block_PWM/255;
    analogWrite(PIN_BLOCK, Output0);   // Send PWM-signal to heater-MOSFETs.

    //Calculate Output for the Heating System of the Cooling Tower
    Input1 = T_1;                      // Input for myPID1 is TempC1.
    myPID1.Compute();                    // Calculate Output1.
    Output1 = round(Out1);
    if(Output1 >= max_box_PWM)
    {
        Output1 = max_box_PWM;
    }
    analogWrite(PIN_BOX, Output1);   // Send PWM-signal to heater-MOSFETs.

    
    
} // End of RunPID()


float v_probing(int address, int sample_number)
{
             // number of samples taken
    int sum = 0;                    // sum of samples taken
    int counter = 0;                // current sample number
    float voltage = 0.0;            // calculated voltage
    float p_psi = 0.0;
    float p_bar = 0.0;

    //take several samples
    while (counter < sample_number) {
        sum += analogRead(address);
        counter++;
        delay(1);
    }
    // calculate the average voltage
    // this is a modified version with calibrated output for Raspberry Pico
    voltage = ((float)sum / (float)sample_number * 3.1) / 1024.0;
    voltage = voltage * 11.132;
    voltage_cal = (voltage-d)/k;
    p_bar = ((voltage_cal-0.33)*5.99675)+0.4;                // (voltage_cal-0.3378287)*90.644885;
                                                        //  p_bar = p_psi*0.0689458+0.00166362;
    p_mbar = p_bar*1000;                               // p_mbar = p_bar*1000;
    counter = 0;
    sum = 0;

    return p_mbar;
    
}

void output()
{
    if (DataRead_Excel == true) 
    {       // Output Via MS Excel Datastreamer
        Serial.print(Total_Time_min, 3);    Serial.print(",");
        Serial.print(Time_min, 3);  		Serial.print(",");
        Serial.print(T_0);       	    	Serial.print(",");
        Serial.print(T_1);       	    	Serial.print(",");
        Serial.print(p_mbar);               Serial.print(",");
        Serial.print(voltage_cal);          Serial.print(",");
    

    if (Monitor_Output == true) 
    {      // Monitor Output PWM-Signals is activated
        Serial.print(Output0);      Serial.print(",");
        Serial.print(Output1);      Serial.println();
        
    }
    else 
    {                             // Monitor Output PWM-Signals is deactivated
        Serial.println("");              // New Line
    }

    }   // End if Excel-Readout Branch

    else 
    {                               // Output Via Arduino IDE Serial Monitor
        Serial.print(Total_Time_min, 3);    Serial.print("  ");
        Serial.print(Time_min, 3);  		Serial.print("  ");
        Serial.print(T_0);    		  	    Serial.print("    ");
        Serial.print(T_1);    			    Serial.print("    ");
        Serial.print(p_mbar);               Serial.print("    ");
        Serial.print(voltage_cal);          Serial.print("    ");
        Serial.print(IN);                   Serial.print("    ");
        Serial.print(VAC);                  Serial.print("    ");
        Serial.print(PROP);                 Serial.print("    |");
    

    if (Monitor_Output == true) 
    {      // Monitor Output PWM-Signals is activated
        Serial.print(Output0);  			Serial.print(" ");
        Serial.print(Output1);  			Serial.println();
        
    }
    else 
    {                             // Monitor Output PWM-Signals is deactivated
        Serial.println("");              // New Line
    }
    }   // End if Serial Monitor-Readout Branch


    delay(5);
}
