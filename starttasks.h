#ifndef _start_tasks_
#define _start_tasks_

namespace StartTasks{
  static boolean SPI_ON=false;
  bool startSPI(){
     if(SPI_ON){return true;}
     pinMode(46, OUTPUT);digitalWrite(46, HIGH);//Disable Flash
     pinMode(53, OUTPUT);digitalWrite(53, HIGH);//Disable SD
     SPI.begin();
     SPI.setDataMode(SPI_MODE3);
     SPI.setClockDivider(SPI_CLOCK_DIV8);
     SPI_ON=true;
     return SPI_ON;
  }

  static boolean I2C_ON=false;
  bool startI2C(){
      if(I2C_ON){return true;}
      Wire.begin();
      I2C_ON=true;
      return I2C_ON;
  }
}

#endif
