#include "arduinocomm.h"
using namespace std;

//int s_debug = 1;

Arduinocomm::Arduinocomm()
{
}

void Arduinocomm::update()
{
    /*if(input_position >= this->input_size){
        input_position = 0; 
        this->input_size = 0;*/
            //Serial.println("Available?");
        while(Serial.available() > 0)
        {
            //Serial.println("Available");
            //sendcustombyte((uint8_t)Serial.available());
            //byte ret = Serial.readBytes(input_buffer,min(Serial.available(),MAX_BUFFER));
            //sendcustombyte(ret);
            //serial_count++;
            //if(ret>0){this->input_size = ret; process();}
            int val = Serial.read();
            //Serial.println(val);
            if(val>=0)
            { input_buffer[0] = val;}
            this->input_size = 1;

            process();
        }
    /*}
    else if(this->input_size > 0){
        process();
    }*/
}

void Arduinocomm::process()
{
    uint8_t val = 0;
    uint8_t crc = 0;
    uint8_t pb_pos = 0;
    //sendcustombyte(this->input_size);
    //while(input_position < this->input_size)
    {
        val = input_buffer[0];
        //sendcustombyte(val);
        if(val > 127)//Command
        {
            switch(val)
            {
                case START_DATA:
                    packet_ready = false;
                    packet_position = 0;
                    reading_packet = true;
                    packet_size = 0;
                    //input_position++;
                    //sendcustombyte(input_position);
                    break;
                case END_DATA:
                    pb_pos = 0;
                    for(int i=0;i<packet_position;i+=2)
                    {
                        packet_buffer[pb_pos++] = readbyte(i);
                        /*Serial.println("Packet buffer");
                        Serial.print((int)packet_buffer[pb_pos-1]);
                        Serial.println();*/
                        //sendcustombyte(packet_buffer[i/2]);
                    }
                    //Serial.println();

                    crc = CRC8((const uint8_t*)&temp_buffer,packet_position-2,0);
                    if(crc == packet_buffer[pb_pos-1] && reading_packet)
                    {
                        packet_ready = true;
                        packet_size = packet_position;
                    }
                    else
                    {
                        //Serial.println("Fail");
                    }
                    reading_packet = false;
                    //input_position++;
                    return;
                default:
                    //sendcustombyte(22);

                    packet_position = 0;
                    packet_ready = false;
                    reading_packet = false;
                    //input_position++;
                    break;
            }
        }
        else
        {
            if(reading_packet)
            {
                //if(val == 2){sendcustombyte(val);}
                temp_buffer[packet_position] = val;
                //if(val == 2){temp_buffer[packet_position];}
                packet_position++;
                //input_position++;
            }
            /*else
            {
                input_position++;
            }*/
        }
    }
}

//Reads 2 bytes into 1 byte, converting from 7-bit packing to 8-bits.
uint8_t Arduinocomm::readbyte(unsigned int position)
{
    return temp_buffer[position] + (temp_buffer[position+1] << 7);
}

/*uint16_t Arduinocomm::read_uint16(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
  {
  return packet[position] + (packet[position+1] << 8);
  }

  uint32_t Arduinocomm::read_uint32(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
  {
  return packet[position] + (packet[position+1] << 8) + (packet[position+2] << 16) + (packet[position+3] << 24);
  }*/

void Arduinocomm::writecommand(uint8_t byte)
{
    //Serial.write(byte);
    bytes[byte_position++] = byte;
}
//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    //Serial.write(byte & 0x7F);
    bytes[byte_position++] = byte & 0x7F;
    //Serial.write((byte & 0x80) >> 7);
    bytes[byte_position++] =(byte & 0x80) >> 7;
}

void Arduinocomm::writeuint32(uint32_t value)
{
    writebyte(value & 0x000000FF);
    writebyte((value >> 8) & 0x000000FF);
    writebyte((value >> 16) & 0x000000FF);
    writebyte((value >> 24) & 0x000000FF);
}

void Arduinocomm::writeuint16(uint16_t value)
{
    writebyte(value & 0x00FF);
    writebyte((value >> 8) & 0x00FF);
}

void Arduinocomm::sendcustombyte(uint8_t byte)
{
    int len = 8;
    bytes = (uint8_t *) malloc(len);
    writecommand(START_DATA);
    writebyte(DEBUG);
    writebyte(byte);
    writebyte(CRC8((const uint8_t*)bytes,len-4,1));
    writecommand(END_DATA);
    Serial.write(bytes,len);
    Serial.flush();
    free(bytes);
    byte_position = 0;
}

void Arduinocomm::write_ok()
{
    int len = 6;
    bytes = (uint8_t *) malloc(len);
    writecommand(START_DATA);
    writebyte(OK);
    writebyte(CRC8((const uint8_t*)bytes,len-4,1));
    writecommand(END_DATA);
    Serial.write(bytes,len);
    Serial.flush();
    free(bytes);
    byte_position = 0;
}

void Arduinocomm::write_line_position(uint16_t position)
{
    int len = 10;
    bytes = (uint8_t *) malloc(len);
    writecommand(START_DATA);
    writebyte(LINE_POSITION);
    writeuint16(position);
    writebyte(CRC8((const uint8_t*)bytes,len-4,1));
    writecommand(END_DATA);
    if(debug)
    {
        for(int i =0;i<len;i++)
        {
            Serial.print(bytes[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    else
    {
        Serial.write(bytes,len);
        Serial.flush();
    }
    free(bytes);
    byte_position = 0;
}

///CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
uint8_t Arduinocomm::CRC8(const uint8_t *data, uint8_t len, uint8_t start) {
    //Serial.println("crc");
    //Serial.println(len);
    data+=start;
    uint8_t crc = 0x00;
    while (len--) {
        uint8_t extract = *data++;
        //Serial.println(extract);
        for (byte tempI = 8; tempI; tempI--) {
            byte sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}


