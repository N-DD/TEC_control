int parseCommand (char* inCommand, float* inValue){

  ///READING SERIAL PORT
int readLength = Serial.available();

String inBuffer;

if(readLength){
 inBuffer = Serial.readStringUntil('\n');
 //Serial.print("Received: "); Serial.println(inBuffer);
 }

//Parsing command
*inCommand = inBuffer[0];

if(inBuffer.length()>2){
//parse value  
  char floatbuf[32];
  inBuffer.remove(0,2);
  inBuffer.toCharArray(floatbuf, sizeof(floatbuf));
  *inValue = atof(floatbuf);

}else{
  *inValue = 0;
}

return readLength;
//Serial.print("Command: "); Serial.println(InCommand->command);
//Serial.print("Value: "); Serial.println(InCommand->value);

//    
//  
//
//  
// }
//    
//  //read the incoming byte
//  
//  
//  
//  //delayy next read out

}

