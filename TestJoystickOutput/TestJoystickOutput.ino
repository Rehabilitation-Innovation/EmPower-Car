const int joystickPinX = A1;      // Analog input pin X (joystick) - Left/Right
const int joystickPinY = A0;      // Analog input pin Y (joystick) - Fwd/Rev
int joystickRawX, joystickRawY;
int outputX, outputY;
int SpeedScale = 100;

// Calibration values (adjust these based on your joystick readings)
const int minX = 93;
const int maxX = 961;
const int minY = 93;
const int maxY = 961;

void setup() {
    Serial.begin(9600);
    
    // Configure pins with pull-up resistors
    pinMode(joystickPinX, INPUT_PULLUP);
    pinMode(joystickPinY, INPUT_PULLUP);
    
    // Optional: Print calibration message
    Serial.println("Joystick test starting...");
    delay(100);  // Allow pull-ups to stabilize
}

void loop() {
    // Read joystick values
    joystickRawX = analogRead(joystickPinX);
    joystickRawY = analogRead(joystickPinY);
    
    // Print values for debugging
    Serial.print("X: ");
    Serial.print(joystickRawX);
    Serial.print("\tY: ");
    Serial.println(joystickRawY);
    
    delay(100);  // Small delay to make serial output readable
}