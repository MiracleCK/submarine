#pragma once

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL.h>

#define UART_CAN_BUFFER_SIZE 1024

class UartOverCan : public AP_HAL::UARTDriver {
public:
    UartOverCan();

    void begin(uint32_t baud) override {begin(baud, UART_CAN_BUFFER_SIZE, UART_CAN_BUFFER_SIZE);};
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;


    uint32_t available() override;
    uint32_t txspace() override;

    int16_t read() override;
    int16_t read_locked(uint32_t key) override;
    void _timer_tick(void) override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    // write to a locked port. If port is locked and key is not correct then 0 is returned
    // and write is discarded
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key) override;

    bool lock_port(uint32_t write_key, uint32_t read_key) override { return false; }

    bool set_options(uint16_t options) override{ return true; }
    uint8_t get_options(void) const override { return 0; }

    void set_flow_control(enum flow_control flow_control_setting) override {};
    enum flow_control get_flow_control(void) override { return FLOW_CONTROL_DISABLE; }

    void configure_parity(uint8_t v) override {};
    void set_stop_bits(int n) override {};

    /* unbuffered writes bypass the ringbuffer and go straight to the
     * file descriptor
     */
    bool set_unbuffered_writes(bool on) override { return false; };

    /*
      wait for at least n bytes of incoming data, with timeout in
      milliseconds. Return true if n bytes are available, false if
      timeout
     */
    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override { return false; }

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    uint32_t bw_in_kilobytes_per_second() const override {
        return _baudrate/(9*1024);
    }

    bool  _in_thread, _in_io;

private:
    static thread_t *uoc_thread_ctx;
    static void uoc_thread(void *);

    uint32_t _baudrate;

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};

    virtual_timer_t tx_timeout;
    bool _in_timer;
    bool _blocking_writes;
    bool _initialised;
};
