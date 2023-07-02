/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#include "AP_Proximity_config.h"

#include "AP_Proximity_Backend_Serial.h"


#define LDROBOT_TIMEOUT_MS              500000    // Driver will report "unhealthy" if valid sensor readings not received within this many ms
#define LDROBOT_INIT_TIMEOUT_MS         1000   // Timeout this many ms after init
#define LDROBOT_MAX_RANGE_M             25.0f   // max range of the sensor in meters
#define LDROBOT_MIN_RANGE_M             0.1f   // min range of the sensor in meters


#define  LDROBOT_PACKET_HEADER  0x54
#define  LDROBOT_PACKET_VER_LEN  0x2C
#define  LDROBOT_PACKET_TOTAL_LEN 0X2F
#define  LDROBOT_POINT_PER_PACK 12
#define  LDROBOT_PACKET_DATA_OFFSET 1 //packet data after ver_len
#define  LDROBOT_PACKET_CRC_OFFSET 46 //crc8

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;


typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[LDROBOT_POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;
class AP_Proximity_LDLidar_STL27L : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update the state of the sensor
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return LDROBOT_MAX_RANGE_M; }
    float distance_min() const override { return LDROBOT_MIN_RANGE_M; }

private:

     // read bytes from the sensor
    void read_sensor_data();

    // parse one byte from the sensor. Return false on error.
    bool parse_byte(uint8_t data);

    // parse payload, to pick out distances, and feed them to the correct faces
    void parse_payload();

    // reset all variables and flags
    void reset();

    // expected bytes from the sensor
    enum PacketList {
        Header = 0,
        VerLen,
        Data
    } _parse_state;

    union
    {
      LiDARFrameTypeDef _frame;
      uint8_t data[LDROBOT_PACKET_TOTAL_LEN];
    }_packet;

    uint8_t _bytes_count; //counter of payload bytes received
    bool _initialized;
    uint32_t _last_init_ms;                 // system time of last sensor init
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor

    AP_Proximity_Temp_Boundary _temp_boundary; // temporary boundary to store incoming payload

};
