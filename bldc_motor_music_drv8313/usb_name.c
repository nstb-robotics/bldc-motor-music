// BLDC Motor Music - USB device name ("Motor 1", "Motor 2", ...), set by MOTOR_NUMBER in config.h
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)

#include "usb_names.h"
#include "config.h"

#define MIDI_NAME      {'M','o','t','o','r',' ','0' + MOTOR_NUMBER}
#define MIDI_NAME_LEN  7

struct usb_string_descriptor_struct usb_string_product_name = {
        2 + MIDI_NAME_LEN * 2,
        3,
        MIDI_NAME
};
