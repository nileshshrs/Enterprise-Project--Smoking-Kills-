// Pin for relay module set as output
int myRelay = 7
                                                                                      +;
volatile byte relayState = LOW;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Hello! Connected");

    pinMode(myRelay, OUTPUT);
    digitalWrite(myRelay, HIGH);
    relayState = HIGH;
    Serial.println("RELAY ON");
}

void loop() {
    // Check if data is available in the serial buffer
    if (Serial.available() > 0) {
        char command = Serial.read(); // Read the command from serial
        
        if (command == '0') {
            // Turn relay ON
            pullRelayHIGH();
        } 
        else if (command == '1') {
            // Turn relay OFF
            pullRelayLOW();
        }
        else {
            Serial.println("Invalid Command. Use '1' to turn ON and '0' to turn OFF.");
        }
    }
    
    delay(100); // Small delay to avoid overwhelming the serial buffer
}

void pullRelayHIGH() {
    if(relayState != HIGH){
        digitalWrite(myRelay, HIGH);
        relayState = HIGH;
        Serial.println("RELAY OFF");
    }
}

void pullRelayLOW() {
    if(relayState != LOW){
        digitalWrite(myRelay, LOW);
        relayState = LOW;
        Serial.println("RELAY ON");
    }
}