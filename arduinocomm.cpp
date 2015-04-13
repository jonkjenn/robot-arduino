#include "arduinocomm.h"
using namespace std;

Arduinocomm::Arduinocomm()
{
}

void Arduinocomm::update()
{
    if(input_position >= this->input_size){
        input_position = 0; 
        this->input_size = 0;
        while(Serial.available() > 0)
        {
            //sendcustombyte((uint8_t)Serial.available());
            //byte ret = Serial.readBytes(input_buffer,min(Serial.available(),MAX_BUFFER));
            //sendcustombyte(ret);
            //serial_count++;
            //if(ret>0){this->input_size = ret; process();}
            int val = Serial.read();
            if(val>0)
                { input_buffer[0] = val;}
            this->input_size = 1;

            process();
        }
    }
    else if(this->input_size > 0){
        process();
    }
}

void Arduinocomm::process()
{
    uint8_t val = 0;
    //sendcustombyte(this->input_size);
    while(input_position < this->input_size)
    {
        val = input_buffer[input_position];
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
                    input_position++;
                    //sendcustombyte(input_position);
                    continue;
                case END_DATA:

                    for(int i=0;i<packet_position;i+=2)
                    {
                        packet_buffer[i/2] = readbyte(i);
                        sendcustombyte(packet_buffer[i/2]);
                    }

                    if(reading_packet)
                    {
                        packet_ready = true;
                        packet_size = packet_position;
                    }
                    reading_packet = false;
                    input_position++;
                    return;
                default:
                    //sendcustombyte(22);

                    packet_position = 0;
                    packet_ready = false;
                    reading_packet = false;
                    input_position++;
                    continue;
            }
        }
        else
        {
            if(reading_packet)
            {
                //if(val == 2){sendcustombyte(val);}
                //sendcustombyte(val);
                temp_buffer[packet_position] = val;
                //if(val == 2){temp_buffer[packet_position];}
                packet_position++;
                input_position++;
            }
            else
            {
                input_position++;
            }
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
    Serial.write(byte);
    Serial.flush();
}
//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    Serial.write(byte & 0x7F);
    Serial.write((byte & 0x80) >> 7);
    Serial.flush();
}

void Arduinocomm::writeuint32(uint32_t value)
{
    writebyte(value & 0x000000FF);
    writebyte((value >> 8) & 0x000000FF);
    writebyte((value >> 16) & 0x000000FF);
    writebyte((value >> 24) & 0x000000FF);
}

void Arduinocomm::sendcustombyte(uint8_t byte)
{
    writecommand(START_DATA);
    writebyte(DEBUG);
    writebyte(byte);
    writecommand(END_DATA);
}

void Arduinocomm::writeok()
{
    writecommand(START_DATA);
    writebyte(OK);
    writecommand(END_DATA);
}
