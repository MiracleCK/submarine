/*
  factory test proto
*/

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <math.h>

#include "Sub.h"

#include "factory_proto.h"
#include "factory.h"

extern const AP_HAL::HAL& hal;

#define FACTORY_SERIAL_BAUD               57600
#define FACTORY_SERIAL_BUFFER_SIZE_RX     128
#define FACTORY_SERIAL_BUFFER_SIZE_TX     256

#define IS_MSG_VALID() \
  ((_msg.sof == TELEMETRY_MSG_SOF) &&\
  (_msg.size >= TELEMETRY_MSG_SVC_SIZE) &&\
  (_msg.size <= TELEMETRY_BUFFER_SIZE))

/* Nibble lookup table for 0x04C11DB7 polynomial. */
static const uint32_t crc_tab[16] = {
    0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,
    0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
    0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,
    0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD
};

// Constructor
Factory_proto::Factory_proto(AP_HAL::UARTDriver *port) :
        _port(port),
        _bytesRequired(TELEMETRY_MSG_HDR_SIZE)
{
}

void Factory_proto::init()
{
    _port->begin(
        FACTORY_SERIAL_BAUD,
        FACTORY_SERIAL_BUFFER_SIZE_RX,
        FACTORY_SERIAL_BUFFER_SIZE_TX);

    _msgPos = (uint8_t *)&_msg;
}

// read - read latest voltage and current
void Factory_proto::read()
{
    update();
}

/**
 * @brief crc32 - calculates CRC32 checksum of the data buffer.
 * @param pBuf - address of the data buffer.
 * @param length - length of the buffer.
 * @return CRC32 checksum.
 */
uint32_t Factory_proto::crc32(uint32_t pBuf[], size_t length)
{
    uint32_t i;
    /* Initial XOR value. */
    uint32_t crc = 0xFFFFFFFF;

    for (i = 0; i < length; i++) {
        /* Apply all 32-bits: */
        crc ^= pBuf[i];

        /* Process 32-bits, 4 at a time, or 8 rounds.
         * - Assumes 32-bit reg, masking index to 4-bits;
         * - 0x04C11DB7 Polynomial used in STM32.
         */
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
    }
    return crc;
}

uint32_t Factory_proto::getCRC32Checksum(TelemetryMessage *pMsg) {
    size_t crc_length = (pMsg->size - TELEMETRY_MSG_CRC_SIZE) / sizeof(uint32_t);
    if ((pMsg->size - TELEMETRY_MSG_CRC_SIZE) % sizeof(uint32_t)) {
        crc_length++;
    }
    return crc32((uint32_t *)pMsg, crc_length);
}

void Factory_proto::readSerialDataResync(uint8_t len) {
    uint8_t i;

    while (len) {
        for (i = 1; i < len; i++) {
            if (((uint8_t *)&_msg)[i] == TELEMETRY_MSG_SOF) {
                break;
            }
        }

        if (len - i > 0) {
            memmove(&_msg, &((uint8_t *)&_msg)[i], len - i);
        }
        len -= i;
        _msgPos = (uint8_t *)&_msg + len;

        if (len < TELEMETRY_MSG_HDR_SIZE) {
            /* ...wait for header to get completed */
            _bytesRequired = TELEMETRY_MSG_HDR_SIZE - len;
            break;
        } else {
            if (IS_MSG_VALID()) {
                if (_msg.size <= len) {
                    /* This may throw away some data at the tail of buffer...*/
                    _bytesRequired = 0;
                } else {
                    _bytesRequired = _msg.size - len;
                }
                break;
            }
        }
    }
}

void Factory_proto::processMessage() 
{
    //printf("get a valid processMessage.id:%d\r\n", _msg.msg_id);
    switch (_msg.msg_id) 
    {
        case FACTORY_USB_REQUEST_MSGID:
            sendFactoryUsbRespMsg(&_msg);
            break;
        case FACTORY_USB_RESPOND_MSGID:
            break;
        case FACTORY_TEST_HISI_RESULT_MSGID:
            sub.factory.setHisiTestResult(_msg.data, _msg.size-TELEMETRY_MSG_HDR_SIZE-TELEMETRY_MSG_CRC_SIZE);
            break;
        default:
        break;
    }
    return;
}

void Factory_proto::sendFactoryTestMsg(uint8_t msg_id, uint8_t sc_res, uint8_t camera_res)
{
    TelemetryMessage msg;

    memset(&msg, 0, sizeof(TelemetryMessage));

    msg.sof  = TELEMETRY_MSG_SOF;
    msg.size = TELEMETRY_MSG_SVC_SIZE + 2;
    msg.res  = 0;

    msg.data[0] = sc_res;
    msg.data[1] = camera_res;
    msg.msg_id = msg_id;
    msg.crc = 0;
    msg.crc = getCRC32Checksum(&msg);

    _port->write((const uint8_t *)&msg, msg.size - TELEMETRY_MSG_CRC_SIZE);
    _port->write((const uint8_t *)&msg.crc, TELEMETRY_MSG_CRC_SIZE);

    return;
}

void Factory_proto::sendFactoryUsbRespMsg(TelemetryMessage *reqMsg)
{
    reqMsg->msg_id = FACTORY_USB_RESPOND_MSGID;
    reqMsg->crc = 0;
    reqMsg->crc = getCRC32Checksum(reqMsg);
    
    _port->write((const uint8_t *)reqMsg, reqMsg->size - TELEMETRY_MSG_CRC_SIZE);
    _port->write((const uint8_t *)&reqMsg->crc, TELEMETRY_MSG_CRC_SIZE);

    return;
}

void Factory_proto::update() {

    /* The following function must be called from within a system lock zone. */
    uint32_t crc, msg_crc;
    size_t bytesAvailable = _port->available();
    if (bytesAvailable)
    {
        //printf("recv %d bytes.\r\n", bytesAvailable);
    }
    
    while (bytesAvailable) {
        if (bytesAvailable >= _bytesRequired) {
            if (_bytesRequired > 0) {
                read_port(_msgPos, _bytesRequired);
                _msgPos += _bytesRequired;
                bytesAvailable -= _bytesRequired;
                _bytesRequired = 0;
            }
        } else {
            read_port(_msgPos, bytesAvailable);
            _msgPos += bytesAvailable;
            _bytesRequired -= bytesAvailable;
            break;
        }

        size_t curReadLen = _msgPos - (uint8_t *)&_msg;
        if (!IS_MSG_VALID()) {
            printf("msg invalid.\r\n");
            readSerialDataResync(curReadLen);
        } else if (curReadLen == TELEMETRY_MSG_HDR_SIZE) {
            _bytesRequired = _msg.size - TELEMETRY_MSG_HDR_SIZE;
        } else if (_bytesRequired == 0) {
            /* Whole packet is read, check and process it... */
            /* Move CRC */
            memmove(&_msg.crc, (uint8_t *)&_msg + _msg.size - TELEMETRY_MSG_CRC_SIZE,
                    TELEMETRY_MSG_CRC_SIZE);
            /* Zero-out unused data for crc32 calculation. */
            memset(&_msg.data[_msg.size - TELEMETRY_MSG_SVC_SIZE], 0,
                   TELEMETRY_BUFFER_SIZE - _msg.size + TELEMETRY_MSG_SVC_SIZE);

            msg_crc = _msg.crc;
            _msg.crc = 0;
            crc = getCRC32Checksum(&_msg);
            if (msg_crc == crc) {
                processMessage();
            } else {
                /* do nothing */
                printf("get crc32 error with (0x%08x, 0x%08x).\r\n", msg_crc, crc);
            }

            /* Read another packet...*/
            _bytesRequired = TELEMETRY_MSG_HDR_SIZE;
            _msgPos = (uint8_t *)&_msg;
        }
    }
}

size_t Factory_proto::read_port(uint8_t *buf, uint32_t len) {
    uint32_t i;

    if (_port->available() < (int16_t)len) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        buf[i] = _port->read();
    }

    return i;
}

void Factory_proto::write_port(uint8_t *buf, uint32_t len) {
    _port->write(buf, len);
}

Factory_proto g_uart_up_port(hal.uartF);
Factory_proto g_uart_down_port(hal.uartG);

