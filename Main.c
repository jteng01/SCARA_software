
void setup() {
    Serial.begin(115200);

    TCCR1A = 0;			//Reset entire TCCR1A register
    TCCR1B = 0;			//Reset entire TCCR1B register
    TCCR1A |= B00000100;		//Set CS12 to 1 so we get Prescalar = 256
    TCNT1 = 0;			//Reset Timer 1 value to 0
    
    

    
}


void loop() {
    
    //initialize variables
    
    //run the code indefinetly
    while(1){
        
    }
}
