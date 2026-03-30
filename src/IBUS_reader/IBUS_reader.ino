/*the purpose of this code is to read the IBUS dat coming from the iA10B FlySky RC receiver
  it utilizes ann arduino nano, reading data over tx pin
  Instructions:
  power receiver 5v
  power on transmitter  
  Upload code to arduino
  connect ibus to tx pin ONLY after previous step
  read values on terminal

  notes: 
  -this code is used to precisely trim the ends of the pwm signal of individual channels, as well as adjusting the neutral spot accuratelly
  -make sure to connect the gnd of the arduino to the receiver/power supply, otherwise nothing will read
*/
#include <IBusBM.h>

IBusBM ibus;

void setup() {
  // Initialize Serial for debugging and IBUS
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to initialize

  Serial.println("Starting IBUS setup...");
  delay(1000); // Optional delay for debugging

  // Start IBUS communication using the hardware Serial port
  ibus.begin(Serial);

  Serial.println("IBUS setup complete :)");
}

void loop() {
  // Read and print IBUS channel data
  Serial.print("CH1: ");
  Serial.print(ibus.readChannel(0));
  Serial.print("\tCH2: ");
  Serial.print(ibus.readChannel(1));
  Serial.print("\tCH3: ");
  Serial.print(ibus.readChannel(2));
  Serial.print("\tCH4: ");
  Serial.print(ibus.readChannel(3));
  Serial.print("\tCH5: ");
  Serial.print(ibus.readChannel(4));
  Serial.print("\tCH6: ");
  Serial.print(ibus.readChannel(5));
  Serial.print("\tCH9: ");
  Serial.println(ibus.readChannel(9));

  delay(100);  // Small delay for readability
}
