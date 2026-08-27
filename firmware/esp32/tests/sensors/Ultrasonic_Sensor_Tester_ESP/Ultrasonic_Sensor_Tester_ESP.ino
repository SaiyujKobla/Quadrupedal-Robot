const int trigPin = 5;
const int echoPin = 34;

void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);

  delay(1000);
  Serial.println("Ultrasonic diagnostic test");
}

void loop() {
  Serial.print("Echo before trigger: ");
  Serial.println(digitalRead(echoPin));

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 40000);

  Serial.print("Pulse duration: ");
  Serial.print(duration);
  Serial.println(" us");

  if (duration == 0) {
    Serial.println("ERROR: Echo never went HIGH");
  } else {
    float distanceCm = duration * 0.0343f / 2.0f;

    Serial.print("Distance: ");
    Serial.print(distanceCm, 1);
    Serial.println(" cm");
  }

  Serial.println();
  delay(500);
}