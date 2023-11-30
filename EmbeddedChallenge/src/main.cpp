#include <mbed.h>
#include "LCD_DISCO_F429ZI.h"
#include "math.h"

#define CALIBRATION 0  // 0: Normal mode; 1: Calculates the zero level values for gyro
#define PLOT 0         // 0: Normal mode; 1: Print values for plot

// from the datasheet L3GD20
#define RADIUS_X 2     
#define RADIUS_Y 2
#define RADIUS_Z 0.55

#define GYRO_MOSI PF_9   // Master Out Slave In for SPI
#define GYRO_MISO PF_8   // Master In Slave Out for SPI
#define GYRO_SCK  PF_7   // Clock for SPI
#define GYRO_CS   PC_1   // Chip select for SPI

#define REG_1_CONFIG 0x3f   // datarate = 100 Hz, Cutoff = 25, Enable = 1, Xaxis = 1, Yaxis = 1, Zaxis = 1 
#define REG_2_CONFIG 0x00   // Highpass filter mode: Normal, Highpass cutoff: 8Hz
#define REG_4_CONFIG 0x10   // Endianess = DataLSB, Fullscale = 500, SelfTest = Normal, SPI = 4 wiremode
#define REG_5_CONFIG 0x10   // Boot: Normal mode, FIFO_EN: disabled, HPen: HPF enabled, No interrupt

#define REG_1_ADDRESS  0x20
#define REG_2_ADDRESS  0x21
#define REG_4_ADDRESS  0x23
#define REG_5_ADDRESS  0x24
#define ID_REG_ADDRESS 0x0F
#define X_REG_ADDRESS  0x28
#define Y_REG_ADDRESS  0x2A
#define Z_REG_ADDRESS  0x2C

#define READ_CMD         0x80
#define MULTIPLEBYTE_CMD 0x40
#define DUMMY_BYTE       0x00

#define FS_500_SENSITIVITY 0.0175 // sensitivity from the datasheet

#define MAX_GYRO 500
#define MIN_GYRO -500

float samples[40][3];           // sampled at the rate of 0.5s for 20s; Values in terms of angular velocity(dps)
float linearVelStorage[40][3];

int8_t dataCollected = 0;
int8_t dataSent = 0;

// ZERO-level, after calibration of gyroscope values
int16_t zero_X = 13;
int16_t zero_Y = 22;
int16_t zero_Z = 3; 

#if CALIBRATION
int16_t X_cal[100];
int16_t Y_cal[100];
int16_t Z_cal[100];
int cal_count = 0;
#endif

double globDist; //distance covered in 20 seconds

SPI spi(GYRO_MOSI, GYRO_MISO, GYRO_SCK);
DigitalOut GyroCs(GYRO_CS);
LCD_DISCO_F429ZI lcd;
static BufferedSerial serial_port(USBTX, USBRX);

// Gyro related functions
int Gyro_Init();
void Gyro_Get_XYZ(float xyz[]);


int main() {

  serial_port.set_baud(9600);
  GyroCs = 1;
  spi.format(8,3);
  spi.frequency(1000000);

  int Gyro_ID;

  float GyroXYZ[3]; //buffer for storing 3 bytes X, Y, Z
  float linearVelocity[3];

  int sampleIndex = 0;
  
  Gyro_ID = Gyro_Init();
  printf("Gyro_ID: %d\n", Gyro_ID);

  // LCD related initialization
  //////////////////////////////////////////////////////////////////////////
  BSP_LCD_SetFont(&Font20);
  lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Gyro values", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Velocity", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"Time ellapsed", CENTER_MODE);
  wait_us(1000000);

  BSP_LCD_SetFont(&Font16);
  lcd.DisplayStringAt(0, LINE(16), (uint8_t *)"0 s", CENTER_MODE);

  char x_msg[25]={0};
  char y_msg[25]={0};
  char z_msg[25]={0};

  char x_vel[25]={0};
  char y_vel[25]={0};
  char z_vel[25]={0};

  char distance_display[25]={0};
  char time_display[10]={0};
  /////////////////////////////////////////////////////////////////////////


  while(1) {
    
    Gyro_Get_XYZ(GyroXYZ);
    
    // Avoid occational erroneous data
    if(GyroXYZ[0] > MAX_GYRO || GyroXYZ[0] < MIN_GYRO) {
      GyroXYZ[0] = 0;
    }
    if(GyroXYZ[1] > MAX_GYRO || GyroXYZ[1] < MIN_GYRO) {
      GyroXYZ[1] = 0;
    }
    if(GyroXYZ[2] > MAX_GYRO || GyroXYZ[2] < MIN_GYRO) {
      GyroXYZ[2] = 0;
    }

    // linear velosity for X, Y, Z (0.5 seconds)
    if(sampleIndex == 0) {
      linearVelocity[0] = 0;
      linearVelocity[1] = 0;
      linearVelocity[2] = 0;
    } else {
      // linear velociy = angular velocity * radius
      linearVelocity[0] = (samples[sampleIndex-1][0]-GyroXYZ[0]) * (RADIUS_X * 0.001);
      linearVelocity[1] = (samples[sampleIndex-1][1]-GyroXYZ[1]) * (RADIUS_Y * 0.001);
      linearVelocity[2] = (samples[sampleIndex-1][2]-GyroXYZ[2]) * (RADIUS_Z * 0.001);
    }
        
    if(sampleIndex < 40) {
      // Accumulate data till 20s
      samples[sampleIndex][0] = GyroXYZ[0];
      samples[sampleIndex][1] = GyroXYZ[1];
      samples[sampleIndex][2] = GyroXYZ[2];

      linearVelStorage[sampleIndex][0] = linearVelocity[0];
      linearVelStorage[sampleIndex][1] = linearVelocity[1];
      linearVelStorage[sampleIndex][2] = linearVelocity[2];
    }

    if (sampleIndex == 40) {
      for(int i = 1; i < 40; i++) {
        float x_dist, y_dist, z_dist;
        // distance = linear velocity * time taken
        x_dist = linearVelStorage[i][0] * 0.5;
        y_dist = linearVelStorage[i][1] * 0.5;
        z_dist = linearVelStorage[i][2] * 0.5;

        //calculate distance travelled in 0.5s interval
        double dist = sqrt((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist));

        globDist += dist;

        // LCD Display
        lcd.Clear(LCD_COLOR_WHITE);
        BSP_LCD_SetFont(&Font24);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Distance", CENTER_MODE);
        BSP_LCD_SetFont(&Font24);        
        sprintf(distance_display, "%5.2f m", globDist);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *)distance_display, CENTER_MODE);
      }

      //calibration section
      globDist -= 0.035; //offset
      if(globDist < 0) {
        globDist = 0;
      }
      globDist /= 0.165; //scaling
      printf("globDist = %f\n", globDist);

      dataCollected = 1;
      sampleIndex++;
    }

    if (dataCollected == 1 && dataSent == 0) {
      // Send the data accumulated for 20s through UART
      char message[30] = {0};
      for(int i = 0; i < 40; i++) {
        sprintf(message, "%2d sample:\nx value: %4.5f\ny value: %4.5f\nz value: %4.5f\n\n\0", i, samples[i][0], samples[i][1], samples[i][2]);
        int j = 0;
        while(message[j] != '\0')
        {
        	serial_port.write(&message[j], 1);
          j++;
        }
        dataSent = 1;
      }

#if PLOT
      printf("Plot data for Angular velocity\n");
      for(int j = 0; j < 3; j++) {
        printf("%f, ", samples[0][j]);
        for(int i = 1; i < 39; i++) {
          printf("%f, ", samples[i][j]);
        }
        printf("%f\n", samples[39][j]);
      }

      printf("Plot data for Linear velocity\n");
      for(int j = 0; j < 3; j++) {
        printf("%f, ", linearVelStorage[0][j]);
        for(int i = 1; i < 39; i++) {
          printf("%f, ", linearVelStorage[i][j]);
        }
        printf("%f\n", linearVelStorage[39][j]);
      }
#endif
    }

    // LCD Display
    /////////////////////////////////////////////////////////////////////////
    if(sampleIndex < 40) {
      sprintf(x_msg, "X Value: %5.2f", GyroXYZ[0]);
      sprintf(y_msg, "Y Value: %5.2f", GyroXYZ[1]);
      sprintf(z_msg, "Z Value: %5.2f", GyroXYZ[2]);

      sprintf(x_vel, "X Vel: %5.2f", linearVelocity[0]);
      sprintf(y_vel, "Y Vel: %5.2f", linearVelocity[1]);
      sprintf(z_vel, "Z Vel: %5.2f", linearVelocity[2]);

      sprintf(time_display, "%5.2f s", sampleIndex * 0.5);
      
      lcd.ClearStringLine(3);
      lcd.ClearStringLine(4);
      lcd.ClearStringLine(5);

      lcd.DisplayStringAt(0, LINE(3), (uint8_t*)x_msg, LEFT_MODE);
      lcd.DisplayStringAt(0, LINE(4), (uint8_t*)y_msg, LEFT_MODE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t*)z_msg, LEFT_MODE);

      lcd.ClearStringLine(9);
      lcd.ClearStringLine(10);
      lcd.ClearStringLine(11);

      lcd.DisplayStringAt(0, LINE(9), (uint8_t*)x_vel, LEFT_MODE);
      lcd.DisplayStringAt(0, LINE(10), (uint8_t*)y_vel, LEFT_MODE);
      lcd.DisplayStringAt(0, LINE(11), (uint8_t*)z_vel, LEFT_MODE);
      
      lcd.ClearStringLine(16);
      lcd.DisplayStringAt(0, LINE(16), (uint8_t *)time_display, CENTER_MODE);
    }
    /////////////////////////////////////////////////////////////////////////

    sampleIndex++;

    wait_us(500000);

  }
}

int Gyro_Init() {
  int ID;

  GyroCs = 0;
  spi.write(ID_REG_ADDRESS | READ_CMD);
  ID = spi.write(DUMMY_BYTE);
  wait_us(3);
  GyroCs = 1;

  GyroCs = 0;
  spi.write(REG_1_ADDRESS); // REG_1 address
  spi.write(REG_1_CONFIG);
  wait_us(3);
  GyroCs = 1;

  GyroCs = 0;
  spi.write(REG_2_ADDRESS); // REG_2 address
  spi.write(REG_2_CONFIG);
  wait_us(3);
  GyroCs = 1;

  GyroCs = 0;
  spi.write(REG_4_ADDRESS); // REG_4 address
  spi.write(REG_4_CONFIG);
  wait_us(3);
  GyroCs = 1;

  GyroCs = 0;
  spi.write(REG_5_ADDRESS); // REG_5 address
  spi.write(REG_5_CONFIG);
  wait_us(3);
  GyroCs = 1;
  
  return ID;
}

void Gyro_Get_XYZ(float xyz[]) {
  int low = 0;
  int high = 0;
  int16_t x, y, z;

  // Read data for Xaxis
  GyroCs = 0;
  spi.write(X_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD);
  wait_us(3);
  low = spi.write(DUMMY_BYTE);
  high = spi.write(DUMMY_BYTE);
  wait_us(3);
  GyroCs = 1;
  x = (high << 8) | low;

  // Read data for Yaxis
  GyroCs = 0;
  spi.write(Y_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD);
  wait_us(3);
  low = spi.write(DUMMY_BYTE);
  high = spi.write(DUMMY_BYTE);
  wait_us(3);
  GyroCs = 1;
  y = (high << 8) | low;

  // Read data for Zaxis
  GyroCs = 0;
  spi.write(Z_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD);
  wait_us(3);
  low = spi.write(DUMMY_BYTE);
  high = spi.write(DUMMY_BYTE);
  wait_us(3);
  GyroCs = 1;
  z = (high << 8) | low;

  xyz[0] = (x - zero_X) * FS_500_SENSITIVITY;
  xyz[1] = (y - zero_Y) * FS_500_SENSITIVITY;
  xyz[2] = (z - zero_Z) * FS_500_SENSITIVITY;

#if CALIBRATION
  if(cal_count < 100) {
    X_cal[cal_count] = x;
    Y_cal[cal_count] = y;
    Z_cal[cal_count] = z;
    cal_count++;
  }
  else if(cal_count == 100) {
    printf("x;y;z;\n");
    for(int i = 0; i < 100; i++) {
      zero_X += X_cal[i] / 100;
      zero_Y += Y_cal[i] / 100;
      zero_Z += Z_cal[i] / 100;
      printf("%d;%d;%d;\n", X_cal[i], Y_cal[i], Z_cal[i]);
    }
    cal_count++;
  }
#endif
}
