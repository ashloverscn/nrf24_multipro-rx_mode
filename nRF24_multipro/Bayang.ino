/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#define BAYANG_BIND_COUNT       1000
#define BAYANG_PACKET_PERIOD    2000
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5

static uint8_t Bayang_rf_chan = 0;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];

enum{
    // flags going to packet[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
    BAYANG_FLAG_FLIP     = 0x08,
    BAYANG_FLAG_VIDEO    = 0x10,
    BAYANG_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    BAYANG_FLAG_INVERT   = 0x80,
};

uint32_t process_Bayang()
{
    uint32_t timeout = micros() + BAYANG_PACKET_PERIOD;
#ifndef RX_MODE
    Bayang_send_packet(0);
#else
    // timeout = micros() + BAYANG_PACKET_PERIOD/10;
    Bayang_recv_packet();
#endif
    return timeout;
}

void Bayang_init()
{
    const u8 bind_address[] = {0,0,0,0,0};
#ifndef RX_MODE
    uint8_t i;
    for(i=0; i<BAYANG_ADDRESS_LENGTH; i++) {
        Bayang_rx_tx_addr[i] = random() & 0xff;
    }
    Bayang_rf_channels[0] = 0x00;
    for(i=1; i<BAYANG_RF_NUM_CHANNELS; i++) {
        Bayang_rf_channels[i] = random() % 0x42;
    }
#endif
    NRF24L01_Initialize();
#ifndef RX_MODE
    NRF24L01_SetTxRxMode(TX_EN);
#else
    NRF24L01_SetTxRxMode(RX_EN);
#endif
    XN297_SetTXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
#ifdef RX_MODE
    XN297_SetRXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
#endif
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
#ifdef RX_MODE
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE); // rx pipe 0
#endif
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // address size
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);

#ifdef RX_MODE
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
#endif
}

void Bayang_bind()
{
#ifndef RX_MODE
    uint16_t counter = BAYANG_BIND_COUNT;
    while(counter) {
        Bayang_send_packet(1);
        delayMicroseconds(BAYANG_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
#else
    Bayang_bind_rx();
#endif
}

void Bayang_bind_rx()
{
    int bind_count = 0;
    uint8_t bind_packet[BAYANG_PACKET_SIZE] = {0};

    uint32_t timeout;

    digitalWrite(ledPin, LOW);

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, BAYANG_RF_BIND_CHANNEL);

    while(bind_count < 10) {
        timeout = millis()+5;

        while(millis()<timeout) {
            delay(1);
            if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from tx
                XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();

                if( packet[0] == 0xA4)
                {
                  if (0 == bind_count)
                  {
                    memcpy(bind_packet, packet, BAYANG_PACKET_SIZE);
                    ++bind_count;
                  }
                  else
                  {
                    if (0 == memcmp(bind_packet, packet, BAYANG_PACKET_SIZE))
                      ++bind_count;
                  }
                }
                break;
            }
        }
    }

    memcpy(Bayang_rx_tx_addr, &packet[1], 5);
    memcpy(Bayang_rf_channels, &packet[6], 4);
    transmitterID[0] = packet[10];
    transmitterID[1] = packet[11];

    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    XN297_SetRXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, Bayang_rf_channels[Bayang_rf_chan++]);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushRx();
    digitalWrite(ledPin, HIGH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))
void Bayang_recv_packet()
{
  static bool is_bound = false;
  static uint16_t failsafe_counter = 0;
  if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from tx
    is_bound = true;
    failsafe_counter = 0;

    int sum = 0;
    uint16_t roll, pitch, yaw, throttle;
    XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

    if( packet[0] == 0xA4)
    {
      // bind packet
      Serial.println("bind packet");
    }
    else if (packet[0] == 0xA5)
    {
      // data packet
      for(int i=0; i<14; i++)
      {
        sum += packet[i];
      }
      if ( (sum&0xFF) == packet[14] )
      {
        // checksum OK
        roll = (packet[4] & 0x0003) * 256 + packet[5];
        pitch = (packet[6] & 0x0003) * 256 + packet[7];
        yaw = (packet[10] & 0x0003) * 256 + packet[11];
        throttle = (packet[8] & 0x0003) * 256 + packet[9];

        // Serial.print(roll, DEC); Serial.print(",");
        // Serial.print(pitch, DEC); Serial.print(",");
        // Serial.print(yaw, DEC); Serial.print(",");
        // Serial.print(throttle, DEC); Serial.println("");

        // roll: 0 .. 1023; servo 0 .. 180 but stop at +-80° due to reduced holding current
        servo.write( - asin( roll / 511.5 - 1.0 ) / ( 3.1416 / 2 ) * 80.0 + 90 );
        // throttle: 0 .. 1023; pwm 0 .. 255
        analogWrite( 6, throttle / 4  );
      }
      else
      {
        //checksum FAIL
        Serial.println("checksum FAIL");
      }
      NRF24L01_WriteReg(NRF24L01_05_RF_CH, Bayang_rf_channels[Bayang_rf_chan++]);
      Bayang_rf_chan %= sizeof(Bayang_rf_channels);
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
      NRF24L01_FlushRx();
    }
    else
    {
        Serial.print("Unrecognized packet: ");
        Serial.println(packet[0], HEX);
    }
  } else if ( is_bound ) {
        ++failsafe_counter;
        if ( failsafe_counter > 1000 ) {
            is_bound = false;
            failsafe_counter = 0;
            // neutral steering position; motor off:
            servo.write( 90 );
            analogWrite( 6, 0 );
            // enable rebinding:
            extern bool reset;
            reset = true;
            Serial.println("reset");
        }
  }
}

void Bayang_send_packet(u8 bind)
{
    union {
        u16 value;
        struct {
            u8 lsb;
            u8 msb;
        } bytes;
    } chanval;

    if (bind) {
        packet[0] = 0xa4;
        memcpy(&packet[1], Bayang_rx_tx_addr, 5);
        memcpy(&packet[6], Bayang_rf_channels, 4);
        packet[10] = transmitterID[0];
        packet[11] = transmitterID[1];
    } else {
        packet[0] = 0xa5;
        packet[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
        packet[2] = GET_FLAG(AUX2, BAYANG_FLAG_FLIP)
                  | GET_FLAG(AUX5, BAYANG_FLAG_HEADLESS)
                  | GET_FLAG(AUX6, BAYANG_FLAG_RTH)
                  | GET_FLAG(AUX3, BAYANG_FLAG_SNAPSHOT)
                  | GET_FLAG(AUX4, BAYANG_FLAG_VIDEO);
        packet[3] = GET_FLAG(AUX1, BAYANG_FLAG_INVERT);
        chanval.value = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 0x3ff);   // aileron
        packet[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[5] = chanval.bytes.lsb;
        chanval.value = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 0x3ff);   // elevator
        packet[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[7] = chanval.bytes.lsb;
        chanval.value = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 0x3ff);   // throttle
        packet[8] = chanval.bytes.msb + 0x7c;
        packet[9] = chanval.bytes.lsb;
        chanval.value = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 0x3ff);   // rudder
        packet[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[11] = chanval.bytes.lsb;
    }
    packet[12] = transmitterID[2];
    packet[13] = 0x0a;
    packet[14] = 0;
    for(uint8_t i=0; i<BAYANG_PACKET_SIZE-1; i++) {
        packet[14] += packet[i];
    }

    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? BAYANG_RF_BIND_CHANNEL : Bayang_rf_channels[Bayang_rf_chan++]);
    Bayang_rf_chan %= sizeof(Bayang_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WritePayload(packet, BAYANG_PACKET_SIZE);
}
