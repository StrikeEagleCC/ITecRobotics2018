void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int xmin = 500;
  static int xmax = 500;
  static int ymin = 500;
  static int ymax = 500;

  int x = analogRead(A1);
  int y = analogRead(A2);

  if (x < xmin) xmin = x;
  if (x > xmax) xmax = x;
  if (y < ymin) ymin = y;
  if (y > ymax) ymax = y;

  Serial.print(xmin);
  Serial.print("\t");
  Serial.print(xmax);
  Serial.print("\t");
  Serial.print(ymin);
  Serial.print("\t");
  Serial.print(ymax);
  Serial.print("\t\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);
}
