#pragma once

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

/* Telemetry start-of-frame signature. */
#define TELEMETRY_MSG_SOF         0xBD
/* Empty message ID. */
#define TELEMETRY_MSG_NOMSG       0x00
/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE     0x80
/* Telemetry message header size in bytes.  */
#define TELEMETRY_MSG_HDR_SIZE    0x04
/* Telemetry message checksum size in bytes.  */
#define TELEMETRY_MSG_CRC_SIZE    0x04
/* Telemetry message header + crc size in bytes.  */
#define TELEMETRY_MSG_SVC_SIZE    ( TELEMETRY_MSG_HDR_SIZE + TELEMETRY_MSG_CRC_SIZE )


#define FACTORY_USB_REQUEST_MSGID         0x55
#define FACTORY_USB_RESPOND_MSGID         0x57
#define FACTORY_TEST_STM32_RESULT_MSGID     (1)
#define FACTORY_TEST_HISI_RESULT_MSGID      (2)


typedef struct tagTelemetryMessage {
    uint8_t sof;
    uint8_t msg_id;
    uint8_t size;
    uint8_t res;
    uint8_t data[TELEMETRY_BUFFER_SIZE];
    uint32_t crc;
} TelemetryMessage;

class Factory_proto
{
public:

    // Constructor
    Factory_proto(AP_HAL::UARTDriver *port);

    /// init
    void init();
    // Read the battery voltage and current.  Should be called at 10hz
    void read();

    void sendFactoryTestMsg(uint8_t msg_id, uint8_t sc_res, uint8_t camera_res);
    void write_port(uint8_t *buf, uint32_t len);

private:

    uint32_t crc32(uint32_t pBuf[], size_t length);
    uint32_t getCRC32Checksum(TelemetryMessage *pMsg);
    void readSerialDataResync(uint8_t len);
    void processMessage();
    void sendFactoryUsbRespMsg(TelemetryMessage *reqMsg);
    void update();
    void updateRequest();
    size_t read_port(uint8_t *buf, uint32_t len);
    AP_HAL::UARTDriver *_port;
    TelemetryMessage _msg;
    uint8_t *_msgPos;
    size_t _bytesRequired;
};

extern Factory_proto g_uart_up_port;
extern Factory_proto g_uart_down_port;

#endif

