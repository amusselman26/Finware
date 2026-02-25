void setup() {
  // Initialize the built-in LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Check your board's schematic; the LED might be active-low
  // If the LED is connected to Vcc, LOW turns it ON, HIGH turns it OFF

  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (or off, depending on wiring)
  delay(1000);                      // Wait for a second
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off (or on)
  delay(1000);                      // Wait for a second
}
