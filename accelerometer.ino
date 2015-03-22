// Code from 
// codeyoung.blogspot.com/2009/11/adxl345-accelerometer-breakout-board.html
// datasheet:
// www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf

#include <Wire.h>
void writeTo(int device, byte address, byte val) {
    Wire.beginTransmission(device);  // start transmission to device 
    Wire.write(address);             // send register address
    Wire.write(val);                 // send value to write
    Wire.endTransmission();          // end transmission
}

void readFrom(int device, byte address, int num, byte buff[]) {
    Wire.beginTransmission(device);  // start transmission to device 
    Wire.write(address);             // sends address to read from
    Wire.endTransmission();          // end transmission
  
    Wire.beginTransmission(device);  // start transmission to device (initiate again)
    Wire.requestFrom(device, num);   // request 6 bytes from device

    int i = 0;
    while(Wire.available())          // device may send less than requested (abnormal)
    {
        buff[i] = Wire.read();       // receive a byte
        i++;
    }
    Wire.endTransmission();          // end transmission
}


#define ACCELADDR (0x53)    //ADXL345 device address
#define ACCELBYTES (6)      //num of bytes we are going to read each time (two bytes for each axis)

byte buff[ACCELBYTES] ;    //6 bytes buffer for saving data read from the device

void setupAccelerometer()
{
    Wire.begin();                 // join i2c bus (address optional for master)

    //Turning on the ADXL345
    // 0x2D is the Power Control register of the ADXL345.
    writeTo(ACCELADDR, 0x2D, 0);  // We first reset the power control register,
    writeTo(ACCELADDR, 0x2D, 16); // then put the sensor in standby mode,
    writeTo(ACCELADDR, 0x2D, 8);  // and last we are putting it in to measure mode.
    // We're doing the writes one after another because that's what the
    // datasheet recommends. We could simply do the last write also. If you 
    // don't turn on the fourth bit on (writeTo(DEVICE, 0x2D, 8)) the sensor 
    // will be in sleep mode, and will give zero readings!!
}


void getAccelVector(int xyz[])
{
    // Let's start from the beginning. The regAddress variable represents
    // the register we want to start reading from (registers 0x32 - 0x37 
    // are acceleration data registers, page 18 in the datasheet).
    // Next we're performing a readFrom function call, we pass our device's
    // address, the register address to start reading from , the number of
    // bytes to read and the buffer in which to save the data.
    // Now comes the tricky part - convert the data received from 2 bytes in
    // to one int. Each axis' pair of bytes are manipulated, in order to
    // create an int from two bytes using C bitwise operators (see here and
    // here for more info on bitwise operators in C).
    // The delay in the end is not strictly necessary, but it gave me better
    // performance that way.

    int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
    int x, y, z;

    readFrom(ACCELADDR, regAddress, ACCELBYTES, buff); //read the acceleration data from the ADXL345

    //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
    //thus we are converting both bytes in to one int
    x = (((int)buff[1]) << 8) | buff[0];   
    y = (((int)buff[3])<< 8) | buff[2];
    z = (((int)buff[5]) << 8) | buff[4];

//     char str[512];   //string buffer to transform data before sending it to the serial port
//     //we send the x y z values as a string to the serial port
//     sprintf(str, "%d %d %d", x, y, z);  
//     Serial.print(str);
//     Serial.write(10);

    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;

    //It appears that delay is needed in order not to clog the port
    delay(15);
}


float vecLength(int vec[], int dim)
{
    uint8_t i;
    float a = 0;
    for(i=0; i<dim; i++)
    {
        a += (float) vec[i] * (float) vec[i]; // Cast to float to prevent integer overflow.
    }
    return sqrt(a);
}


float getAccelMag()
{
    int xyz[3];
    getAccelVector(xyz);
    return vecLength(xyz, 3);
}



float get2DAccelMag(int repeats)
{ 
    float a;
    if( repeats <= 0 )
    {
        a = get2DAccelMag();
    }
    else
    {
        uint8_t i;
        a = 0.0;
        for(i=0; i<repeats; i++)
        {
            a += get2DAccelMag();
            delay(0.1);
        }
        a /= repeats;
    }
    return a;
}



float get2DAccelMag()
{
     // Magnitude of projection onto xy plane. I.e., ignores gravity.
    int xyz[3];
    getAccelVector(xyz);
    return vecLength(xyz, 2);
}
