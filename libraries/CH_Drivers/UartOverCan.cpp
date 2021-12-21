/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "UartOverCan.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

thread_t *UartOverCan::rx_thread_ctx;
thread_t *UartOverCan::tx_thread_ctx;

#if 0
#define ENTER_TH_CRITERIAL p->_in_thread = true; \
    if (p->_in_io)\
        palWriteLine(HAL_GPIO_PIN_LED_3, 0);
#define EXIT_TH_CRITERIAL p->_in_thread = false; \
    palWriteLine(HAL_GPIO_PIN_LED_3, 1);

#define ENTER_IO_CRITERIAL _in_io = true; \
    if (_in_thread)\
        palWriteLine(HAL_GPIO_PIN_LED_4, 0);
#define EXIT_IO_CRITERIAL _in_io = false; \
    palWriteLine(HAL_GPIO_PIN_LED_4, 1);
#else
#define ENTER_TH_CRITERIAL
#define EXIT_TH_CRITERIAL
#define ENTER_IO_CRITERIAL
#define EXIT_IO_CRITERIAL
#endif

uint8_t UartOverCan::rx_ch;
uint8_t UartOverCan::rx_len;
//uint16_t UartOverCan::rx_count;
//uint16_t UartOverCan::rx_bad;
//uint16_t UartOverCan::rx_empty;

UartOverCan::UartOverCan()
{
    _baudrate = 500000;
    _in_thread = false;
    _in_io = false;
    for (int i = 0; i < 4; ++i)
        _test_data[i] = '0';
    for (int i = 0; i < 26; ++i)
        _test_data[i + 4] = 'A' + i;
    for (int i = 0; i < 10; ++i)
        _test_data[i + 30] = '0' + i;
    _test_data[40] = '\n';

    rx_ch = 0xFF;
    rx_len = 0;
    //rx_count = 0, rx_bad = 0, rx_empty = 0;
}

void UartOverCan::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (rx_thread_ctx == nullptr)
    {
        rx_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(128),
                  "apm_can_rx",
                  182,
                  rx_thread,
                  this);
    }
    if (tx_thread_ctx == nullptr)
    {
        tx_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(128),
                "apm_can_tx",
                178,
                tx_thread,
                this);
    }
    if (CAND1.state == CAN_STOP)
    {
        static const CANConfig cancfg = {
                CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
                CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
                CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
        };
        canStart(&CAND1, &cancfg);
        hal.shell->printf("canStart\r\n");
    }

    if (txSpace < UART_CAN_BUFFER_SIZE) {
        txSpace = UART_CAN_BUFFER_SIZE;
    }
    if (rxSpace < UART_CAN_BUFFER_SIZE) {
        rxSpace = UART_CAN_BUFFER_SIZE;
    }
    if (rxSpace != _readbuf.get_size()) {
        _initialised = false;
        _readbuf.set_size(rxSpace);
        _readbuf.clear();
    }
    if (txSpace != _writebuf.get_size()) {
        _initialised = false;
        _writebuf.set_size(rxSpace);
        _writebuf.clear();
    }

    _initialised = true;
    hal.shell->printf("uoc begin %d\r\n", baud);
}

void UartOverCan::rx_thread(void *arg)
{
    UartOverCan *p = (UartOverCan *)arg;
    CANRxFrame rxmsg;
    //chEvtRegister(&canp->rxfull_event, &el, 0);
    while (true) {
        while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_MS2I(100)) == MSG_OK) {
            ENTER_TH_CRITERIAL
            p->_readbuf.write(rxmsg.data8, rxmsg.DLC);
            EXIT_TH_CRITERIAL
            /*
            if (rxmsg.DLC == 0)
            {
                rx_empty++;
                return;
            }

            for (int i = 0; i < rxmsg.DLC; ++i)
            {
                uint8_t c = rxmsg.data8[i];
                if (c == 0xFE)
                {
                    if (rx_len > 0)
                    {
                        if (rx_len != 41)
                            rx_bad++;
                        rx_count++;
                    }
                    rx_len = 1;
                }
                else
                {
                    if (rx_len == 1)
                    {
                        rx_ch += 1;
                        if (rx_ch == 0)
                            rx_ch = c;
                        else if (rx_ch == 0xC8)
                            rx_ch = 0;
                    }

                    if (rx_ch > c)
                    {
                        if (rx_ch < 0xC8)
                        {
                            uint8_t skip = 0xC8 - rx_ch + c;
                            rx_bad += skip;
                            rx_count += skip;
                            rx_empty += skip;
                            rx_len = 1;
                        }
                        rx_ch = c;
                    }
                    else if (rx_ch < c)
                    {
                        uint8_t skip =  c - rx_ch;
                        rx_bad += skip;
                        rx_count += skip;
                        rx_empty += skip;
                        rx_len = 1;
                        rx_ch = c;
                    }

                    rx_len++;
                }
            }*/
        }
    }
    //chEvtUnregister(&canp->rxfull_event, &el);
}

void UartOverCan::tx_thread(void *arg)
{
    UartOverCan *p = (UartOverCan *)arg;
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x03;
    txmsg.RTR = CAN_RTR_DATA;
    while (true) {
        int32_t len = p->_writebuf.available();
        while (len > 0)
        {
            ENTER_TH_CRITERIAL
            int32_t n = p->_writebuf.peekbytes(txmsg.data8, MIN(len, 8));
            EXIT_TH_CRITERIAL
            txmsg.DLC = n;

            if (MSG_OK == canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_US2I(100)))
            {
                len -= n;
                p->_writebuf.advance(n);
            }
        }
        osalThreadSleep(TIME_US2I(1000));
    }
}

/*void UartOverCan::test(void)
{
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x03;
    txmsg.RTR = CAN_RTR_DATA;
    uint8_t *data = _test_data;
    uint32_t ts = AP_HAL::millis();
    for (int a = 0; a < 1; ++a)
    {
        data[0] = '0' + a;
        for (int b = 0; b < 10; ++b)
        {
            data[1] = '0' + b;
            for (int c = 0; c < 10; ++c)
            {
                data[2] = '0' + c;
                for (int d = 0; d < 10; ++d)
                {
                    data[3] = '0' + d;
                    uint32_t len = 41;
                    while (len > 0)
                    {
                        uint32_t n = MIN(8, len);
                        txmsg.DLC = n;
                        memcpy(txmsg.data8, &data[41 - len], n);
                        while(MSG_OK != canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE));
                        len -= n;
                    }
                    osalThreadSleep(TIME_US2I(2300));
                }
            }
        }
    }
    hal.shell->printf("time cost: %d\r\n", AP_HAL::millis() - ts);
}

void UartOverCan::test_send(void)
{
    CANTxFrame txmsg;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x03;
    txmsg.RTR = CAN_RTR_DATA;
    uint8_t *data = _test_data;

    uint32_t len = 41;
    while (len > 0)
    {
        uint32_t n = MIN(8, len);
        txmsg.DLC = n;
        memcpy(txmsg.data8, &data[41 - len], n);
        while(MSG_OK != canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE));
        len -= n;
    }
    for (int i = 3; i >= 0; --i)
    {
        if(++data[i] > '9')
            data[i] = '0';
        else
            break;
    }
}*/

void UartOverCan::end()
{

}

void UartOverCan::flush()
{

}

bool UartOverCan::is_initialized()
{
    return _initialised;
}

void UartOverCan::set_blocking_writes(bool blocking)
{
    _blocking_writes = blocking;
}

bool UartOverCan::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t UartOverCan::available() {
    return _readbuf.available();
}

uint32_t UartOverCan::txspace()
{
    return _writebuf.space();
}

int16_t UartOverCan::read()
{
    uint8_t byte;
    ENTER_IO_CRITERIAL
    if (!_readbuf.read_byte(&byte)) {
        EXIT_IO_CRITERIAL
        return -1;
    }

    EXIT_IO_CRITERIAL
    return byte;
}

int16_t UartOverCan::read_locked(uint32_t key)
{
    return read();
}

/* Empty implementations of Print virtual methods */
size_t UartOverCan::write(uint8_t c)
{
    ENTER_IO_CRITERIAL
    size_t len = _writebuf.write(&c, 1);
    EXIT_IO_CRITERIAL
    return len;
}

size_t UartOverCan::write(const uint8_t *buffer, size_t size)
{
    ENTER_IO_CRITERIAL
    size_t s = _writebuf.write(buffer, size);
    EXIT_IO_CRITERIAL
    return s;
}

/*
   write to a locked port. If port is locked and key is not correct then 0 is returned
   and write is discarded. All writes are non-blocking
*/
size_t UartOverCan::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    return write(buffer, size);
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UartOverCan::_timer_tick(void)
{
    _in_timer = true;

    _in_timer = false;
}
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
uint64_t UartOverCan::receive_time_constraint_us(uint16_t nbytes)
{
    return 0;
}