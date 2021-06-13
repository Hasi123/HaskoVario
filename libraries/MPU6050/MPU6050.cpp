#include <Arduino.h>
#include "MPU6050.h"
#include <I2CHelper.h>
#include "inv_dmp_uncompress.h"
#include <EEPROM.h>
#include <toneAC.h>
#include "variance.h"

variance var;
MPU6050 mpu;

//constructor
MPU6050::MPU6050(unsigned char Addr) {
  mpuAddr = Addr;
}

//check if mpu is not being moved
//based on variance of gyro values
bool MPU6050::isResting(unsigned short threshold) {
  const unsigned short dataPoints = 250;
  short reading[3];
  
  var.reset();
  I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS); //clear int status
  for (unsigned short i = 0; i < dataPoints; i++) {
    while (!I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS)); //wait for new reading
    for (unsigned char ii = 0; ii < 3; ii++) { //get all axis
      reading[ii] = I2C::readWord(mpuAddr, MPU6050_RA_GYRO_XOUT_H + (2 * ii));
    }
	var.update(reading);
  }

  return var.getSum() < threshold;
}

//reads word "loops" times and averages the result
short MPU6050::readWordAveraged(unsigned char regAddr, unsigned short loops) {
  long sum = 0;
  I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS); //clear int status
  for (unsigned short i = 0; i < loops; i++) {
    while (!bitRead(I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS), MPU6050_INTERRUPT_DATA_RDY_BIT)); //wait for new reading
    sum += I2C::readWord(mpuAddr, regAddr);
  }
  return (short)(sum / loops);
}

//calibrate mpu if initially up side down
bool MPU6050::calibrate(void) {
	
  //check if moving
  var.reset();
  for (unsigned char i = 0; i < 50; i++) { //over half second
    while (!I2C::newData); //wait for new reading
    I2C::newData = false;
    var.update(gyroData);
  }
  bool moving = var.getSum() > 200;

  if ((accelData[2] > -7000) || moving) { //also up side down?
    return false;
  }
  
  detachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN));
  delay(10); //wait for last I2C interrupt to end

  //setup for calibration
  I2C::writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, bit(MPU6050_PWR1_DEVICE_RESET_BIT)); //reset
  delay(100);
  I2C::writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); //ClockSource to X gyro
  I2C::writeByte(mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); //DLPF
  I2C::writeByte(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_1000 << 3); //Gyro range
  I2C::writeByte(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16 << 3); //Accel range
  //calibration units are in 1000 gyro and 16 accel range
  delay(50);

  short origAccOffs[3];
  unsigned char origGain[3]; //[7:4] accel gain, [3:0] gyro gain

  //get factory calibration data
  origAccOffs[0] = I2C::readWord(mpuAddr, MPU6050_RA_XA_OFFS_H);
  origAccOffs[1] = I2C::readWord(mpuAddr, MPU6050_RA_YA_OFFS_H);
  origAccOffs[2] = I2C::readWord(mpuAddr, MPU6050_RA_ZA_OFFS_H);
  origGain[0] = I2C::readByte(mpuAddr, MPU6050_RA_X_FINE_GAIN);
  origGain[1] = I2C::readByte(mpuAddr, MPU6050_RA_Y_FINE_GAIN);
  origGain[2] = I2C::readByte(mpuAddr, MPU6050_RA_Z_FINE_GAIN);
  //...gyro offsets not needed, since 0

  //////////////////
  //gyro calibration
  //////////////////
  const unsigned short loops = 200;
  short reading[3];
  long sum[3];
  do {
    sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
    var.reset();

    I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS); //clear int status
    for (unsigned short i = 0; i < loops; i++) {
      while (!I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS)); //wait for new reading
      for (unsigned char ii = 0; ii < 3; ii++) { //get all axis
        reading[ii] = I2C::readWord(mpuAddr, MPU6050_RA_GYRO_XOUT_H + (2 * ii));
        sum[ii] += reading[ii];
      }
      var.update(reading);
    }
  } while (var.getSum() > 1500); //only save if variance low

  gyroData[0] = -sum[0]/loops;
  gyroData[1] = -sum[1]/loops;
  gyroData[2] = -sum[2]/loops;

  ////////////////////
  //Accel calibration
  ///////////////////
  //set accel fine gain to 0 for easier calculation
  I2C::writeByte(mpuAddr, MPU6050_RA_X_FINE_GAIN, 0);
  I2C::writeByte(mpuAddr, MPU6050_RA_Y_FINE_GAIN, 0);
  I2C::writeByte(mpuAddr, MPU6050_RA_Z_FINE_GAIN, 0);
  delay(50); //to apply offset values

  //if not moving get averaged (max) data of active axis (TODO maybe: if new data save the higher value, or better var?)
  short accelMax[6];
  unsigned char calibState = 0;
  unsigned char accIndex;
  while (true) {
    if (isResting()) {
      accelData[0] = readWordAveraged(MPU6050_RA_ACCEL_XOUT_H, 50);
      accelData[1] = readWordAveraged(MPU6050_RA_ACCEL_YOUT_H, 50);
      accelData[2] = readWordAveraged(MPU6050_RA_ACCEL_ZOUT_H, 50);

      for (unsigned char i = 0; i < 3; i++) {
        accIndex = i * 2;
        if (!bitRead(calibState, accIndex) && accelData[i] > 1500) { //not set and positive
          accelMax[accIndex] = accelData[i];
          bitSet(calibState, accIndex);
          toneAC(700, 10, 300, true);
        }
        accIndex++;
        if (!bitRead(calibState, accIndex) && accelData[i] < -1500) { //not set and negative
          accelMax[accIndex] = accelData[i];
          bitSet(calibState, accIndex);
          toneAC(700, 10, 300, true);
        }
      }
    }

    //if all axes have data point -> calculate values -> write to registers and print
    if (calibState == 0b111111) {
      short newAccOffs[3];
      unsigned char newGain[3];

      for (unsigned char i = 0; i < 3; i++) {
        //Offsets
        newAccOffs[i] = (accelMax[i * 2] + accelMax[i * 2 + 1]) / 2;

        //Scaling
        short diff = 2048 - (accelMax[i * 2] - newAccOffs[i]); //need to calculate offset compansated values
        if (diff >= 0) //round positive values
          diff += 14;
        else //round negative values
          diff -= 14;
        diff /= 29; //strange value found by experimentation
		
        newGain[i] = (diff << 4) | (origGain[i] & 0b00001111); //combine new acc gain with factory gyro gain

        //final offstes
        newAccOffs[i] = origAccOffs[i] - newAccOffs[i];
      }

      //save values into EEPROM
      EEPROM.put(0, gyroData);
      EEPROM.put(6, newAccOffs);
      EEPROM.put(12, newGain);

      return true;
    } //calculate calibration values
  } //while(1)
}

//load calibration data into the registers
void MPU6050::load_calibration(short *gyro_offs, short *accel_offs, unsigned char *fine_gain) {
  I2C::writeBytes(mpuAddr, MPU6050_RA_X_FINE_GAIN, 3, fine_gain);
  I2C::writeWords(mpuAddr, MPU6050_RA_XA_OFFS_H, 3, accel_offs);
  I2C::writeWords(mpuAddr, MPU6050_RA_XG_OFFS_USRH, 3, gyro_offs);
}

//Write to the DMP memory
char MPU6050::mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {

  if (!I2C::writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!I2C::writeBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Read from the DMP memory
char MPU6050::mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {

  if (!I2C::writeWord(mpuAddr, MPU6050_RA_BANK_SEL, mem_addr))
    return -1;
  if (!I2C::readBytes(mpuAddr, MPU6050_RA_MEM_R_W, length, data))
    return -1;
  return 0;
}

//Load and verify DMP image
char MPU6050::load_dmp() {  //using compressed DMP firmware
  //#define VERIFY_DMP //should we verify loading the DMP?

  unsigned short ii, this_write;
  unsigned char progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#ifdef VERIFY_DMP
  unsigned char cur[MPU6050_DMP_MEMORY_CHUNK_SIZE];
#endif

  /* start loading */
  inv_dmp_uncompress_reset();

  for (ii = 0; ii < UNCOMPRESSED_DMP_CODE_SIZE; ii += this_write) {
    this_write = min(MPU6050_DMP_MEMORY_CHUNK_SIZE, UNCOMPRESSED_DMP_CODE_SIZE - ii);

    /* decompress chunk */
    for (unsigned short progIndex = 0; progIndex < this_write; progIndex++)
      progBuffer[progIndex] = inv_dmp_uncompress();

    //write
    if (mpu_write_mem(ii, this_write, progBuffer))
      return -1;
#ifdef VERIFY_DMP //check
    if (mpu_read_mem(ii, this_write, cur))
      return -1;
    if (memcmp(progBuffer, cur, this_write))
      return -2;
#endif
  }

  /* Set program start address. */
  if (!I2C::writeWord(mpuAddr, MPU6050_RA_DMP_CFG_1, MPU6050_DMP_START_ADDRESS))
    return -1;

  return 0;
}

//Init the sensor, load calibration data and dmp
void MPU6050::init(void) {
  I2C::writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, bit(MPU6050_PWR1_DEVICE_RESET_BIT)); //reset
  delay(100);
  I2C::writeByte(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO); //wake up and set clock to gyro X (recomended by datasheet)
  I2C::writeByte(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000 << 3); //Gyro range
  I2C::writeByte(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS << 3); //Accel range
  I2C::writeByte(mpuAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); //DLPF
  I2C::writeByte(mpuAddr, MPU6050_RA_SMPLRT_DIV, 1000 / MPU6050_SAMPLE_RATE - 1);  //sample rate divider
#ifdef MPU6050_INTERRUPT_PIN
  //writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, 0); //disable interrupts, already 0 by default
  I2C::writeByte(mpuAddr, MPU6050_RA_INT_PIN_CFG, bit(MPU6050_INTCFG_LATCH_INT_EN_BIT) | bit(MPU6050_INTCFG_INT_RD_CLEAR_BIT)); //setup interrupt pin
#endif

  //load calibration data from EEPROM
  EEPROM.get(0, calibData);
  //check if values are plausible on gyro X
  if (calibData.gyrOffs[0] != -32768 && calibData.gyrOffs[0] != 32767)
    load_calibration(calibData.gyrOffs, calibData.accOffs, calibData.fineGain);

  load_dmp();

  //writeByte(mpuAddr, MPU6050_RA_FIFO_EN, 0); //disable FIFO, already 0 by default
  I2C::writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_RESET_BIT) | bit(MPU6050_USERCTRL_FIFO_RESET_BIT)); //reset FIFO and DMP
  delay(50);
  I2C::writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_EN_BIT) | bit(MPU6050_USERCTRL_FIFO_EN_BIT)); //enable FIFO and DMP
#ifdef MPU6050_INTERRUPT_PIN
  I2C::writeByte(mpuAddr, MPU6050_RA_INT_ENABLE, bit(MPU6050_INTERRUPT_DMP_INT_BIT)); //enable hardware DMP interrupt
#endif

}

void MPU6050::parseFIFO(unsigned char *fifo_data){
      //parse data
    quatData[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
                  ((long)fifo_data[2] << 8) | fifo_data[3];
    quatData[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
                  ((long)fifo_data[6] << 8) | fifo_data[7];
    quatData[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
                  ((long)fifo_data[10] << 8) | fifo_data[11];
    quatData[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
                  ((long)fifo_data[14] << 8) | fifo_data[15];
    accelData[0] = ((short)fifo_data[16] << 8) | fifo_data[17];
    accelData[1] = ((short)fifo_data[18] << 8) | fifo_data[19];
    accelData[2] = ((short)fifo_data[20] << 8) | fifo_data[21];
    gyroData[0] = ((short)fifo_data[22] << 8) | fifo_data[23];
    gyroData[1] = ((short)fifo_data[24] << 8) | fifo_data[25];
    gyroData[2] = ((short)fifo_data[26] << 8) | fifo_data[27];
}

void MPU6050::resetFIFO(void) {
  I2C::writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_FIFO_RESET_BIT)); //reset FIFO
  delay(50);
  I2C::writeByte(mpuAddr, MPU6050_RA_USER_CTRL, bit(MPU6050_USERCTRL_DMP_EN_BIT) | bit(MPU6050_USERCTRL_FIFO_EN_BIT)); //enable FIFO and DMP
}

//read 1 FIFO packet and parse data
//should be called after interrupt or data_ready state
char MPU6050::getFIFO(void) {
  unsigned char fifoData[MPU6050_FIFO_LENGTH];
  unsigned short fifo_count = I2C::readWord(mpuAddr, MPU6050_RA_FIFO_COUNTH);

  if (fifo_count != MPU6050_FIFO_LENGTH) { //reset FIFO if more data than 1 packet
    resetFIFO();
    return -1;
  }
  else {
    I2C::readBytes(mpuAddr, MPU6050_RA_FIFO_R_W, MPU6050_FIFO_LENGTH, fifoData); //get FIFO data
    parseFIFO(fifoData);

    return 0;
  }
}

bool MPU6050::newDmp() {
#ifdef MPU6050_INTERRUPT_PIN
  return (PIND & (1 << MPU6050_INTERRUPT_PIN));
#else
  return bitRead(I2C::readByte(mpuAddr, MPU6050_RA_INT_STATUS), MPU6050_INTERRUPT_DMP_INT_BIT);
#endif
}

/* compute vertical vector and vertical accel from IMU data */
double MPU6050::getVertaccel(void) {

  /* G to ms convertion */
#define VERTACCEL_G_TO_MS 9.80665

  /* quat scale */
#define LIGHT_INVENSENSE_QUAT_SCALE_SHIFT 30
#define LIGHT_INVENSENSE_QUAT_SCALE ((double)(1LL << LIGHT_INVENSENSE_QUAT_SCALE_SHIFT))

  /* accel scale */
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 14
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 13
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 12
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 11
#endif
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))

  /***************************/
  /* normalize and calibrate */
  /***************************/
  double accel[3], quat[4], vertVector[3];

  for (unsigned char i = 0; i < 3; i++)
    accel[i] = ((double)accelData[i]) / LIGHT_INVENSENSE_ACCEL_SCALE;

  for (unsigned char i = 0; i < 4; i++)
    quat[i] = ((double)quatData[i]) / LIGHT_INVENSENSE_QUAT_SCALE;


  /******************************/
  /* real and vert acceleration */
  /******************************/

  /* compute vertical direction from quaternions */
  vertVector[0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  vertVector[1] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
  vertVector[2] = 2 * (quat[0] * quat[0] + quat[3] * quat[3]) - 1;

  /* compute real acceleration (without gravity) */
  double ra[3];
  for (unsigned char i = 0; i < 3; i++)
    ra[i] = accel[i] - vertVector[i];

  /* compute vertical acceleration */
  double vertAccel = (vertVector[0] * ra[0] + vertVector[1] * ra[1] + vertVector[2] * ra[2]) * VERTACCEL_G_TO_MS;
  return vertAccel;
}
