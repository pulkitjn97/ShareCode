byte inByte[15];
float fx = 0.0, fy = 0.0, fz = 0.0;

union {
  float uFloat;
  uint8_t uInt8[4];
}thisUnion_ax, thisUnion_ay, thisUnion_az;

//#define max_size 3

//thisunion all_data[max_size];
   
//thisunion  

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
    uint8_t getserial = Serial1.read();
    if (getserial == 'S')
    {
      //Read ax values
      Serial.readBytes(inByte,4);      
      for (int i = 0; i < 4; i++)
      {
        inByte[i] = Serial1.read();
        //Serial.print(inByte[i],BIN);
        //Serial.print("  ");
      }

      //Read ay values
      Serial.readBytes(inByte,4);      
      for (int i = 4; i < 8; i++)
      {
        inByte[i] = Serial1.read();
       // Serial.print(inByte[i],BIN);
       // Serial.print("  ");
      }

      //Read az values
      Serial.readBytes(inByte,4);      
      for (int i = 8; i < 12; i++)
      {
        inByte[i] = Serial1.read();
        //Serial.print(inByte[i],BIN);
        //Serial.print("  ");
      }
      Serial.println("");
      
      for (int i = 0; i < 4; i++) {
        thisUnion_ax.uInt8[i] = inByte[i];
        thisUnion_ay.uInt8[i] = inByte[i+4];
        thisUnion_az.uInt8[i] = inByte[i+8];
      }
      fx = thisUnion_ax.uFloat;
      fy = thisUnion_ay.uFloat;
      fz = thisUnion_az.uFloat;

      Serial.print("Received Data, Ax= ");
      Serial.print(fx);
      Serial.print(" Ay= ");
      Serial.print(fy);
      Serial.print(" Az= ");
      Serial.println(fz);

      Serial1.flush();

    }
}
