#include <Servo.h>

Servo left_wheel;   // create Servo object to control a servo
Servo right_wheel;  // create Servo object to control a servo

const int bufferSize = 64;     // Maximum size of the buffer
char inputBuffer[bufferSize];  // Buffer to hold incoming data
int bufferIndex = 0;           // Current index in the buffer

// Define PWM pins
const int pwmPin1 = 10;
const int pwmPin2 = 11;
int null_signal = 1500;
int high_signal =1685;
int low_signal = 1310;
int min_level = -100;
int max_level = 100;
int null_level = 0;


void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);
  Serial.println("Enter command like: MOVE 1200 2300");
  left_wheel.attach(pwmPin1, 1000, 2000);   // attaches the servo on pin 9 to the Servo object
  right_wheel.attach(pwmPin2, 1000, 2000);  // attaches the servo on pin 10 to the Servo object
  // Reset to null
  right_wheel.writeMicroseconds(1500);
  left_wheel.writeMicroseconds(1500);
}

void loop() {
  // Check if there is data available in the serial buffer
  while (Serial.available() > 0) {
    // Read the next byte of incoming data
    char receivedChar = Serial.read();

    // Check if the received character is the newline character
    if (receivedChar == '\n') {
      // Null-terminate the string
      inputBuffer[bufferIndex] = '\0';

      // Parse the command and parameters
      parseCommand(inputBuffer);

      // Reset the buffer index for the next input
      bufferIndex = 0;
    } else {
      // Check if there's still room in the buffer
      if (bufferIndex < bufferSize - 1) {
        // Store the received character in the buffer
        inputBuffer[bufferIndex++] = receivedChar;
      } else {
        // Buffer overflow, you might want to handle this situation
        Serial.println("Buffer overflow!");
        bufferIndex = 0;  // Reset buffer index
      }
    }
  }
}

void parseCommand(char* command) {
  // Split the command into tokens
  char* token = strtok(command, " ");

  // Check if the first token is "MOVE"
  if (token != NULL && strcmp(token, "MOVE") == 0) {
    // Read the next token, which should be the first PWM value
    token = strtok(NULL, " ");
    int move_intensity1 = 0;
    int move_intensity2 = 0;

    if (token != NULL) {
      move_intensity1 = fixHighLow(atoi(token));  // Convert the first PWM value to an integer
    }

    // Read the next token, which should be the second PWM value
    token = strtok(NULL, " ");
    if (token != NULL) {
      move_intensity2 = fixHighLow(atoi(token));  // Convert the second PWM value to an integer
    }
    if (move_intensity1 == null_level) {
      right_wheel.writeMicroseconds(null_signal);
    } else {
      right_wheel.writeMicroseconds(map(move_intensity1, min_level, max_level, low_signal, high_signal));
    }
    if (move_intensity1 == null_level) {
      left_wheel.writeMicroseconds(null_signal);
    } else {
      left_wheel.writeMicroseconds(map(move_intensity2, min_level, max_level, low_signal, high_signal));
    }
    Serial.print("Setting Wheel values: ");
    Serial.print(move_intensity1);
    Serial.print(" and ");
    Serial.println(move_intensity2);
  } else {
    Serial.println("Invalid command. Use 'MOVE value1 value2'");
  }
}

int fixHighLow(int move_intensity) {
  if (move_intensity > max_level) {
    move_intensity = max_level;
  }
  if (move_intensity < min_level) {
    move_intensity = min_level;
  }
  return move_intensity;
}