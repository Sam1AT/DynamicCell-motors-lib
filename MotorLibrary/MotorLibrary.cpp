#include "Arduino.h"
#include "MotorLibrary.h"

AX_RX_XL::AX_RX_XL(unsigned char ID)
{
    _ID = ID;
}

unsigned short AX_RX_XL::update_crc_XL(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

void AX_RX_XL::goto_wheel_mode_XL()
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;     // id

  buffer[5] = 6;   // packet lengh L
  buffer[6] = 0;     // packet lengh H

  buffer[7] = 3;    // Instruction

  buffer[8] = 11;   // address L
  buffer[9] = 0;    // address H

  buffer[10] = 1;   // 1: wheel mode    2: joint mode

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);     // CRC_L
  buffer[12] = highByte(crc);   // CRC_H

  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }

  delay(100);
}


void AX_RX_XL::goto_joint_mode_XL()
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;     // id

  buffer[5] = 6;   // packet lengh L
  buffer[6] = 0;     // packet lengh H

  buffer[7] = 3;    // Instruction

  buffer[8] = 11;   // address L
  buffer[9] = 0;    // address H

  buffer[10] = 2;  // 1: wheel mode    2: joint mode

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);     // CRC_L
  buffer[12] = highByte(crc);   // CRC_H

  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }

  delay(100);
}


void AX_RX_XL::joint_XL(unsigned int position, unsigned int speed, unsigned int wait)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00;  // RESERVED
  buffer[4] = _ID;       // id

  buffer[5] = 9;       // packet lengh L
  buffer[6] = 0;       // packet lengh H

  buffer[7] = 3;       // Instraction  write

  buffer[8] = 30;     // address L
  buffer[9] = 0;      // address H

  buffer[10] = lowByte(position);     // parameter
  buffer[11] = highByte(position);    // parameter

  buffer[12] = lowByte(speed);       // parameter
  buffer[13] = highByte(speed);     // parameter

  crc = update_crc(0, buffer, 14);
  buffer[14] = lowByte(crc);     // CRC_L
  buffer[15] = highByte(crc);    // CRC_H


  for (i = 0; i < 16; i++)
  {
    Serial1.write(buffer[i]);
  }

  delay(wait);
}


void AX_RX_XL::wheel_XL(unsigned int speed, unsigned char cw_ccw)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;   // id

  buffer[5] = 7;     // packet lengh L
  buffer[6] = 0;    // packet lengh H

  buffer[7] = 3;    // Instraction  write

  buffer[8] = 32;   // address L speed
  buffer[9] = 0;    // address H speed

  if (cw_ccw == 1)
    bitWrite(speed, 10, 0);    // cw
  else
    bitWrite(speed, 10, 1);  // ccw

  buffer[10] = lowByte(speed);    // L parameter
  buffer[11] = highByte(speed);   // H parameter

  crc = update_crc(0, buffer, 12);
  buffer[12] = lowByte(crc);   // CRC_L
  buffer[13] = highByte(crc);  // CRC_H


  for (i = 0; i < 14; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
}


void AX_RX_XL::led_XL(unsigned char id, char r, char g, char b)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00;  // RESERVED
  buffer[4] = _ID;    // id

  buffer[5] = 6;     // packet lengh L
  buffer[6] = 0;     // packet lengh H

  buffer[7] = 3;     // Instraction

  buffer[8] = 25;    // address L
  buffer[9] = 0;     // address H

  buffer[10] = ((b * 4) + (g * 2) + r); // parameter

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);   // CRC_L
  buffer[12] = highByte(crc);  // CRC_H

  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }

  delay(100);

}


void AX_RX_XL::angle_limit_XL(unsigned int cw_limit, unsigned int ccw_limit)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;   // id
  buffer[5] = 5 + 2; // packet lengh L  : packet lengh after packet lengh
  buffer[6] = 0;     // packet lengh H
  buffer[7] = 3;     // Instraction  write
  buffer[8] = 6;     // address L cw_limit
  buffer[9] = 0;     // address H cw_limit
  buffer[10] = lowByte(cw_limit);
  buffer[11] = highByte(cw_limit);

  crc = update_crc(0, buffer, 12);
  buffer[12] = lowByte(crc);  // CRC_L
  buffer[13] = highByte(crc);  // CRC_H


  for (i = 0; i < 14; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = id;   // id
  buffer[5] = 5 + 2; // packet lengh L  : packet lengh after packet lengh
  buffer[6] = 0;     // packet lengh H
  buffer[7] = 3;     // Instraction  write
  buffer[8] = 8;     // address L ccw_limit
  buffer[9] = 0;     // address H ccw_limit
  buffer[10] = lowByte(ccw_limit);
  buffer[11] = highByte(ccw_limit);

  crc = update_crc(0, buffer, 12);
  buffer[12] = lowByte(crc);  // CRC_L
  buffer[13] = highByte(crc);  // CRC_H

  for (i = 0; i < 14; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);

}



void AX_RX_XL::speed_XL(unsigned int speed)
{
  if (speed > 1023)
    speed = 1023;

  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;   // id

  buffer[5] = 7;   // packet lengh L
  buffer[6] = 0;   // packet lengh H

  buffer[7] = 3;   // Instraction  write

  buffer[8] = 32;  // address L speed
  buffer[9] = 0;   // address H speed

  buffer[10] = lowByte(speed);    // L parameter
  buffer[11] = highByte(speed);   // H parameter

  crc = update_crc(0, buffer, 12);
  buffer[12] = lowByte(crc);  // CRC_L
  buffer[13] = highByte(crc);  // CRC_H


  for (i = 0; i < 14; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
}


void AX_RX_XL::torque_XL(unsigned char State)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00;   // RESERVED
  buffer[4] = _ID;        // id

  buffer[5] = 6;   // packet lengh L
  buffer[6] = 0;   // packet lengh H

  buffer[7] = 3;   // Instraction  write

  buffer[8] = 24;  // address L torque
  buffer[9] = 0;    // address H torque

  buffer[10] = State;

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);  // CRC_L
  buffer[12] = highByte(crc);  // CRC_H

  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
}


void AX_RX_XL::change_id_XL(unsigned char new_id)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;   // id
  buffer[5] = 4 + 2; // packet lengh L  : packet lengh after packet lengh
  buffer[6] = 0;     // packet lengh H
  buffer[7] = 3;     // Instraction  write
  buffer[8] = 3;     // address L id
  buffer[9] = 0;     // address H id
  buffer[10] = new_id;

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);  // CRC_L
  buffer[12] = highByte(crc);  // CRC_H


  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
}


void AX_RX_XL::change_baudrate_XL(unsigned char baudrate_code)
{
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = 0xFD;
  buffer[3] = 0x00; // RESERVED
  buffer[4] = _ID;   // id

  buffer[5] = 6; // packet lengh L
  buffer[6] = 0;     // packet lengh H

  buffer[7] = 3;    // Instraction  write

  buffer[8] = 4;    // address L baudrate
  buffer[9] = 0;    // address H baudrate

  buffer[10] = baudrate_code;

  crc = update_crc(0, buffer, 11);
  buffer[11] = lowByte(crc);  // CRC_L
  buffer[12] = highByte(crc);  // CRC_H


  for (i = 0; i < 13; i++)
  {
    Serial1.write(buffer[i]);
  }
  delay(100);
}









void AX_RX_XL::wheel(char ccw_cw, int speedd)
{
  if (ccw_cw == 1) // cw
  {
    bitWrite(speedd, 10, 1); // set  bit_10
  }
  else // ccw
  {
    bitWrite(speedd, 10, 0); // clear bit_10
  }

  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 3); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 32; // speed address
  packet[6] = lowByte(speedd);
  packet[7] = highByte(speedd);

  ////////////////

  // checksum
  packet[8] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]);

  for (int i = 0; i < 9; i++)
  {
    Serial3.write(packet[i]);
  }

  //delay(5);
}

//************************************

void AX_RX_XL::change_baudrate(char baudrate_code)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 4;  // address baudrate
  packet[6] = baudrate_code;

  ///////////////

  // checksum
  packet[7] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);

  for (int i = 0; i < 8; i++)
  {
    Serial3.write(packet[i]);
  }
}

//************************************

void AX_RX_XL::change__ID(char new_id)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 3;  // address id
  packet[6] = new_id;

  ////////////////

  // checksum
  packet[7] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);

  for (int i = 0; i < 8; i++)
  {
    Serial3.write(packet[i]);
  }
}

//************************************

void AX_RX_XL::return_level(char level)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 16;  // address return_level
  packet[6] = level;

  ////////////////

  // checksum
  packet[7] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);

  for (int i = 0; i < 8; i++)
  {
    Serial3.write(packet[i]);
  }

  delay(10);
}

//***************************************************

void AX_RX_XL::torque(char State)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 2); // Length
  packet[4] = 3;

  ///////////////////////

  packet[5] = 24;  // address torque
  packet[6] = State;

  //////////////////

  // checksum
  packet[7] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);

  /////////////////////

  for (int i = 0; i < 8; i++)
  {
    Serial3.write(packet[i]);
  }

  delay(100);
}

//************************************

void AX_RX_XL::led(char State)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 25;  // address led
  packet[6] = State;

  ////////////////

  // checksum
  packet[7] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);

  for (int i = 0; i < 8; i++)
  {
    Serial3.write(packet[i]);
  }
}

//************************************

void  AX_RX_XL::joint(unsigned int Position, unsigned int Speed, unsigned int Time)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (2 + 5); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 30; // P_GOAL_POSITION_L address
  packet[6] = lowByte(Position);
  packet[7] = highByte(Position);
  packet[8] = lowByte(Speed);
  packet[9] = highByte(Speed);

  ////////////////

  // checksum
  packet[10] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8] + packet[9]);

  for (int i = 0; i < 11; i++)
  {
    Serial3.write(packet[i]);
  }
  delay(10);

  delay(Time);
}

//**************************************

void AX_RX_XL::speed(unsigned int Speed)
{
  if (Speed > 1023)
  {
    Speed = 1023;
  }

  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (3 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 32; // speed address
  packet[6] = lowByte(Speed);
  packet[7] = highByte(Speed);

  ////////////////

  // checksum
  packet[8] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]);

  for (int i = 0; i < 9; i++)
  {
    Serial3.write(packet[i]);
  }
  delay(10);
}

//************************************

void AX_RX_XL::angle_limit(unsigned int ccw_limit, unsigned int cw_limit)
{
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (3 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 0x06; // CW_Angle_Limit address
  packet[6] = lowByte(cw_limit);
  packet[7] = highByte(cw_limit);

  ////////////////

  // checksum
  packet[8] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]);

  for (int i = 0; i < 9; i++)
  {
    Serial3.write(packet[i]);
  }

  ////////////////

  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = _ID;
  packet[3] = (3 + 2); // Length
  packet[4] = 3;

  ////////////////

  packet[5] = 0x08; // CCW_Angle_Limit address
  packet[6] = lowByte(ccw_limit);
  packet[7] = highByte(ccw_limit);

  ////////////////

  // checksum
  packet[8] = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]);

  for (int i = 0; i < 9; i++)
  {
    Serial3.write(packet[i]);
  }
}

//************************************

void AX_RX_XL::goto_joint_mode()
{
  AX_RX_XLf.angle_limit(id, 1023, 0);
}

//************************************

void AX_RX_XL::goto_wheel_mode()
{
  AX_RX_XL.angle_limit(_ID, 0, 0);
}