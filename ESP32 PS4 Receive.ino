#include <PS4Controller.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08  // I2C Address
int8_t ps4data[3];          // Array to store PS4 controller data

void setup() {
  Wire.begin(SLAVE_ADDRESS);       // Set up ESP32 as I2C slave
  PS4.begin();                     // Initialize PS4 controller
  Serial.begin(115200);
  Wire.onRequest(requestEvent);    // Callback for I2C data request
}

void loop() {
  if (PS4.isConnected()) {
    // Read PS4 controller data
    int8_t Vx = PS4.LStickX();  // Left Stick X
    int8_t Vy = PS4.LStickY();  // Left Stick Y

    // Deadzone adjustments
    if (abs(Vx) <= 10) Vx = 0;
    if (abs(Vy) <= 10) Vy = 0;

    // Fill data array
    ps4data[0] = Vx;
    ps4data[1] = Vy;
    ps4data[2] = map(PS4.R2Value() - PS4.L2Value(), -255, 255, -127, 127);

    // Print data for debugging
    Serial.println((String)ps4data[0] + " " + ps4data[1] + " " + ps4data[2]);
  }

  delay(100);  // Small delay for stability
}
 
// Event called when master requests data
void requestEvent() {
  Wire.write((uint8_t*)ps4data, sizeof(ps4data));  // Send PS4 data array
}
