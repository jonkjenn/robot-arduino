//Header guard
#ifndef ARDUINOCOMM_H
#define ARDUINOCOMM_H

#include "Arduino.h"

class Arduinocomm{
    private:
        static const uint8_t START_DATA = 0x80;
        static const uint8_t END_DATA = 0xFF;

        void writebyte(uint8_t);
        void writeuint16(uint16_t value);
        void writeuint32(uint32_t);
        uint8_t readbyte(unsigned int position);
        static const uint16_t MAX_BUFFER = 25;
        uint8_t input_buffer[MAX_BUFFER];
        uint8_t temp_buffer[MAX_BUFFER];
        uint8_t output_buffer[MAX_BUFFER];
        void process();

        void writecommand(uint8_t byte);

        uint16_t packet_length;
        bool reading_packet = false;

        uint8_t input_position = 0;
        uint8_t input_size = 0;

        //void handlepacket(uint8_t (&packet)[MAX_BUFFER], uint8_t);
        uint8_t CRC8(const uint8_t *data, uint8_t len, uint8_t start);

        uint8_t *bytes = 0;
        uint8_t byte_position = 0;

    public:
        byte debug = 0;
        void update();
        Arduinocomm();
        void write_ok();
        void write_line_position(uint16_t position);
        //void writeDriveCompleted();
        void stop();
        void sendcustombyte(uint8_t byte);

        uint8_t packet_buffer[MAX_BUFFER];
        uint8_t packet_position = 0;
        uint8_t packet_size = 0;
        bool packet_ready = false;

        //uint32_t read_uint32(uint8_t (&packet)[MAX_BUFFER],unsigned int);
        //uint16_t read_uint16(uint8_t (&packet)[MAX_BUFFER],unsigned int);

        //
        //Packet definitions
        static const uint8_t OK = 0x01;//Acknowledge packet
        static const uint8_t DRIVE = 0x02;//Contains specific engine speeds to be sendt to robot
        static const uint8_t DEBUG = 0x03;
        static const uint8_t DRIVE_DISTANCE = 0x04;//Drive straight for duration
        static const uint8_t DRIVE_COMPLETED = 0x05;//Drive straight for duration
        static const uint8_t LINE_POSITION = 0x06;//Drive straight for duration

};

//End header guard
#endif
