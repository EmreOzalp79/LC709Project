#include "lc709.h"

uint8_t lc709_crc8(uint8_t *data, int len) {
			
		const uint8_t POLYNOMIAL=0x07 ;
		uint8_t crc=0x00;
	
		for (int j = len; j; --j) {
			crc ^= *data++;
			for (int i = 8; i; --i) {
				crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
				}
			}
		return crc;
		}
void I2C_Start (void)
{
/**** STEPS FOLLOWED  ************
1. Send the START condition 
2. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/	

	I2Cx->CR1 |= (1<<10);  // Enable the ACK
	I2Cx->CR1 |= (1<<8);  // Generate START
	while (!(I2Cx->SR1 & (1<<0)));  // Wait fror SB bit to set
}
void I2C_Write (uint8_t data)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Send the DATA to the DR Register
3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2Cx->SR1 & (1<<7)));  // wait for TXE bit to set
	I2Cx->DR = data;
	while (!(I2Cx->SR1 & (1<<2)));  // wait for BTF bit to set
}
void I2C_Address (uint8_t Address)
{
/**** STEPS FOLLOWED  ************
1. Send the Slave Address to the DR Register
2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
3. clear the ADDR by reading the SR1 and SR2
*/	
	I2Cx->DR = Address;  //  send the address
	while (!(I2Cx->SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2Cx->SR1 | I2Cx->SR2;  // read SR1 and SR2 to clear the ADDR bit
}
void I2C_Stop (void)
{
	I2Cx->CR1 |= (1<<9);  // Stop I2C
}
void I2C_WriteMulti (uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2Cx->SR1 & (1<<7)));  // wait for TXE bit to set 
	while (size)
	{
		while (!(I2Cx->SR1 & (1<<7)));  // wait for TXE bit to set 
		I2Cx->DR = (uint32_t )*data++;  // send data
		size--;
	}
	
	while (!(I2Cx->SR1 & (1<<2)));  // wait for BTF to set
}
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR
2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR 
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
	   after reading the second last data byte (after the second last RxNE event)
*/		
	
	int remaining = size;
	
/**** STEP 1 ****/	
	if (size == 1)
	{
		/**** STEP 1-a ****/	
		I2Cx->DR = Address;  //  send the address
		while (!(I2Cx->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 1-b ****/	
		I2Cx->CR1 &= ~(1<<10);  // clear the ACK bit 
		uint8_t temp = I2Cx->SR1 | I2Cx->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2Cx->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/	
		while (!(I2Cx->SR1 & (1<<6)));  // wait for RxNE to set
		
		/**** STEP 1-d ****/	
		buffer[size-remaining] = I2Cx->DR;  // Read the data from the DATA REGISTER
		
	}

/**** STEP 2 ****/		
	else 
	{
		/**** STEP 2-a ****/
		I2Cx->DR = Address;  //  send the address
		while (!(I2Cx->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 2-b ****/
		uint8_t temp = I2Cx->SR1 | I2Cx->SR2;  // read SR1 and SR2 to clear the ADDR bit
		
		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2Cx->SR1 & (1<<6)));  // wait for RxNE to set
			
			/**** STEP 2-d ****/
			buffer[size-remaining] = I2Cx->DR;  // copy the data into the buffer			
			
			/**** STEP 2-e ****/
			I2Cx->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}
		
		// Read the SECOND LAST BYTE
		while (!(I2Cx->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2Cx->DR;
		
		/**** STEP 2-f ****/
		I2Cx->CR1 &= ~(1<<10);  // clear the ACK bit 
		
		/**** STEP 2-g ****/
		I2Cx->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		while (!(I2Cx->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2Cx->DR;  // copy the data into the buffer
	}	
	
}
void writeWord (uint8_t Reg, uint16_t data)
{   uint8_t send[5];
	uint8_t writeBuff[8];
  send[0] = 0x16; // write byte
  send[1] = Reg;                       // command / register
  send[2] = data & 0xFF;
  send[3] = data >> 8;
  send[4] = lc709_crc8(send, 4);
  writeBuff[0]=send[2];
	writeBuff[1]=send[3];
	writeBuff[2]=send[4];

	I2C_Start ();
	I2C_Address (0x16);
	I2C_Write (Reg);
  I2C_WriteMulti (writeBuff,3);
	I2C_Stop ();
}
void readWord (uint8_t Reg, uint16_t *data)
{ 
	uint8_t buffer[3];

	I2C_Start ();
	I2C_Address (0x16);
	I2C_Write (Reg);
	I2C_Start ();  // repeated start
	I2C_Read (0x17, buffer, 3);
	I2C_Stop (); 
	*data = buffer[1];
  *data <<= 8;
  *data |= buffer[0];
  

}
uint16_t getICversion(void) {
  uint16_t vers = 0;
  readWord(LC709204F_IC_VERSION, &vers);
  return vers;
}

/*!
 *    @brief  Initialize the RSOC algorithm
 *    @return True on I2C command success
 */
void initRSOC(void) {
  writeWord(LC709204F_INITIAL_RSOC, 0xAA55);
}

/*!
 *    @brief  Get battery voltage
 *    @return Floating point value read in Volts
 */
float cellVoltage(void) {
  uint16_t voltage = 0;
  readWord(LC709204F_CELL_VOLTAGE , &voltage);
	float value= voltage;
  return value/1000 ;
}

/*!
 *    @brief  Get battery state in percent (0-100%)
 *    @return Floating point value from 0 to 100.0
 */
float cellPercent(void) {
  uint16_t percent = 0;
  readWord(LC709204F_ITE, &percent);
  return percent/10.0;
}

/*!
 *    @brief  Get battery thermistor temperature
 *    @return Floating point value from -20 to 60 *C
 */
float getCellTemperature(void) {
  uint16_t temp = 0;
  readWord(LC709204F_CELL_TEMP, &temp);
  return temp / 10.0;
}

float timeToEmpty (void) {
  uint16_t empty= 0;
  readWord(LC709204F_TIME_TO_EMPTY, &empty);
  return empty;
}

float timeToFull(void) {
  uint16_t timeFull= 0;
  readWord(LC709204F_TIME_TO_FULL, &timeFull);
  return timeFull;
}


/*!
 *    @brief  Set the temperature mode (external or internal)
 *    @param t The desired mode: LC709203F_TEMPERATURE_I2C or
 * LC709203F_TEMPERATURE_THERMISTOR
 *    @return True on successful I2C write
 */
void setTemperatureMode(lc709204_tempmode_t t) {
  writeWord(LC709204F_STATUS_BIT, (uint16_t)t);
}

/*!
 *    @brief  Set the approximate pack size, helps RSOC calculation
 *    @param apa The lc709203_adjustment_t enumerated approximate cell size
 *    @return True on successful I2C write
 */
void setPackSize(lc709204_adjustment_t apa) {
  writeWord(LC709204F_APA, (uint16_t)apa);
}

/*!
 *    @brief  Set battery APA value, per LC709203F datasheet
 *    @param apa_value 8-bit APA value to use for the attached battery
 *    @return True on successful I2C write
 */
void setPackAPA(uint16_t apa_value) {
  writeWord(LC709204F_APA , apa_value);
}

/*!
 *    @brief  Set the alarm pin to respond to an RSOC percentage level
 *    @param percent The threshold value, set to 0 to disable alarm
 *    @return True on successful I2C write
 */
void setAlarmRSOC(uint8_t percent) {
  writeWord(LC709204F_ALARM_LOW_RSOC, percent);
}

/*!
 *    @brief  Set the alarm pin to respond to a battery voltage level
 *    @param voltage The threshold value, set to 0 to disable alarm
 *    @return True on successful I2C write
 */
void setAlarmVoltage(float voltage) {
  writeWord(LC709204F_ALARM_LOW_CELL_VLT, voltage * 1000);
}

/*!
 *    @brief  Set the power mode, LC709203F_POWER_OPERATE or
 * LC709203F_POWER_SLEEP
 *    @param t The power mode desired
 *    @return True on successful I2C write
 */
void setPowerMode(lc709204_powermode_t t) {
  writeWord(LC709204F_IC_POWERMODE, (uint16_t)t);
}

/*!
 *    @brief  Get the thermistor B value (e.g. 3950)
 *    @return The uint16_t B value
 */
uint16_t getThermistorB(void) {
  uint16_t val = 0;
  readWord(LC709204F_TSENSE2_THERMB, &val);
  return val;
}

/*!
 *    @brief  Set the thermistor B value (e.g. 3950)
 *    @param b The value to set it to
 *    @return True on successful I2C write
 */
void setThermistorB(uint16_t b) {
  writeWord(LC709204F_TSENSE2_THERMB, b);
}

/*!
 *    @brief  Get the battery profile parameter
 *    @return The uint16_t profile value (0 or 1)
 */
uint16_t getBattProfile(void) {
  uint16_t val = 0;
  readWord(LC709204F_CHANGE_PARAM, &val);
  return val;
}

/*!
 *    @brief  Set the battery profile parameter
 *    @param b The value to set it to (0 or 1)
 *    @return True on successful I2C write
 */
void setBattProfile(uint16_t b) {
	writeWord(LC709204F_CHANGE_PARAM, b);
}

/*


*/
void currentDirection(lc709204_current_mode_t t) {
  writeWord(LC709204F_CURRENT_DIR, (uint16_t)t);
}
void emptyCellVoltage(uint16_t a) {
  writeWord(LC709204F_EMPTY_CELL_VOLTAGE, a);
}
uint16_t getStateOfHealth(void) {
  uint16_t health = 0;
  readWord(LC709204F_STATE_OF_HEALTH, &health);
  return health;
}
