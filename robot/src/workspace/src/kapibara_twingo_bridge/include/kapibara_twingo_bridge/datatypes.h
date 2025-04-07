#pragma once
#pragma GCC push_options
#pragma GCC optimize ("O0")
#pragma pack(1)

#include <stdint.h>

#define PACKET_TYPE_COUNT 4

typedef enum packet_type
{
    PING=0,
    SENSE=1,
    GENERAL_CFG_DATA=5,
    ACK=6,
} packet_type_t;


typedef struct ping_msg
{
    char msg[2];
} ping_msg_t ;

typedef struct sense
{
    uint8_t pin_states;
} sense_t;

typedef struct ack_msg
{
    char msg[2];
} ack_msg_t ;


#pragma GCC pop_options