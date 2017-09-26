int i;
byte colors[7];
void setup() {
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() > 0) {
    Serial.readBytes(colors, 7);

    for(i = 0; i < 7; i++)
      colors[i] = 255 - colors[i];
//      Serial.print(colors[i]);
//      Serial.print(", ");
//    }
//    Serial.println();
  }
  analogWrite(11, colors[0]);
  analogWrite(10, colors[1]);
  analogWrite(9, colors[2]);
  analogWrite(6, colors[3]);
  analogWrite(5, colors[4]);
  analogWrite(3, colors[5]);
  // put your main code here, to run repeatedly:

}

