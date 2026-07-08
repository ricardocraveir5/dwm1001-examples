/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           LASIR debug version for DWM1001-DEV.
*
*           This version:
*           - Keeps the original Decawave SS TWR responder behaviour.
*           - Uses SEGGER RTT instead of printf/stdout.
*           - Prints every received UWB frame.
*           - Detects possible Bitcraze/Loco TWR POLL frames.
*           - Detects original Decawave POLL frames.
*
*           IMPORTANT:
*           This is still mainly the Decawave responder.
*           The Bitcraze/Loco POLL detection is for diagnosis only.
*           It does NOT yet implement the full Bitcraze-compatible:
*
*               POLL -> ANSWER -> FINAL -> REPORT
*
*           cycle.
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* Modified for LASIR / DWM1001-Crazyflie debug.
*/

#include "sdk_config.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#include "SEGGER_RTT.h"

/* ----------------------------------------------------------------------------
 * Debug macro
 * ----------------------------------------------------------------------------
 *
 * We use SEGGER RTT instead of printf() to avoid stdout/linker errors in
 * SEGGER Embedded Studio 8.x.
 */
#define DBG(...) SEGGER_RTT_printf(0, __VA_ARGS__)

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 80

/* ----------------------------------------------------------------------------
 * Original Decawave SS TWR frames
 * ----------------------------------------------------------------------------
 */

/* Frames used in the original Decawave ranging process. */
static uint8 rx_poll_msg[] = {
  0x41, 0x88, 0,
  0xCA, 0xDE,
  'W', 'A',
  'V', 'E',
  0xE0,
  0, 0
};

static uint8 tx_resp_msg[] = {
  0x41, 0x88, 0,
  0xCA, 0xDE,
  'V', 'E',
  'W', 'A',
  0xE1,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0
};

/* Length of the common part of the Decawave message. */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the original Decawave frames. */
#define ALL_MSG_SN_IDX              2
#define RESP_MSG_POLL_RX_TS_IDX     10
#define RESP_MSG_RESP_TX_TS_IDX     14
#define RESP_MSG_TS_LEN             4

/* ----------------------------------------------------------------------------
 * LASIR / Bitcraze-compatible debug definitions
 * ----------------------------------------------------------------------------
 */

/*
 * Only ANCHOR_ID should change between physical anchors.
 *
 * Anchor 0 -> MY_ADDRESS = 0xbccf000000000000
 * Anchor 1 -> MY_ADDRESS = 0xbccf000000000001
 * Anchor 2 -> MY_ADDRESS = 0xbccf000000000002
 * Anchor 3 -> MY_ADDRESS = 0xbccf000000000003
 */
#define ANCHOR_ID        1        /* <<< CHANGE THIS PER BOARD: 0, 1, 2, 3 */

#define PAN_ID           0xbccf
#define TAG_ADDRESS      0xbccf000000000008ULL
#define ANCHOR_BASE      0xbccf000000000000ULL
#define MY_ADDRESS       (ANCHOR_BASE + ANCHOR_ID)

/* Bitcraze/Loco TWR message types. */
#define LPS_TWR_POLL     0x01
#define LPS_TWR_ANSWER   0x02
#define LPS_TWR_FINAL    0x03
#define LPS_TWR_REPORT   0x04

#define LPS_TWR_TYPE     0
#define LPS_TWR_SEQ      1

/*
 * Expected Bitcraze/Loco IEEE 802.15.4 extended-address frame layout:
 *
 * byte 0-1   : FCF = 0x8841, little endian -> 0x41 0x88
 * byte 2     : frame sequence number
 * byte 3-4   : PAN ID = 0xbccf, little endian -> 0xcf 0xbc
 * byte 5-12  : destination address, little endian
 * byte 13-20 : source address, little endian
 * byte 21... : payload
 */
#define BC_FCF_IDX              0
#define BC_SEQ_IDX              2
#define BC_PAN_IDX              3
#define BC_DEST_ADDR_IDX        5
#define BC_SRC_ADDR_IDX         13
#define BC_PAYLOAD_IDX          21

#define BC_MIN_FRAME_LEN        23
#define BC_EXPECTED_FCF         0xDC41

/* ----------------------------------------------------------------------------
 * Common variables
 * ----------------------------------------------------------------------------
 */

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/*
 * Buffer to store received messages.
 *
 * Original Decawave example used a smaller buffer.
 * We increase this because Bitcraze/Loco extended-address frames are longer.
 */
#define RX_BUF_LEN 128
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for debug. */
static uint32 status_reg = 0;

/*
 * UWB microsecond conversion:
 * 1 uus = 512 / 499.2 us.
 * Device time unit is around 15.65 ps.
 */
#define UUS_TO_DWT_TIME 65536

/*
 * Delayed TX timing.
 * 1100 us is a safe starting value for nRF/DWM1001 operation.
 */
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/*
 * Delay from the end of frame transmission to RX enable.
 * Kept from the original example.
 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Timestamps are 40-bit wide, so use 64-bit variables. */
typedef signed long long int64;
typedef unsigned long long uint64;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* ----------------------------------------------------------------------------
 * Static function declarations
 * ----------------------------------------------------------------------------
 */

static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

static uint16 read_u16_le(const uint8 *buf);
static uint64 read_u64_le(const uint8 *buf);

static void print_rx_frame_debug(const uint8 *buf, uint32 frame_len);
static int is_bitcraze_loco_poll(const uint8 *buf, uint32 frame_len);

/* ----------------------------------------------------------------------------
 * Main responder function
 * ----------------------------------------------------------------------------
 */

int ss_resp_run(void)
{
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /*
   * Poll for reception of a frame, error or timeout.
   */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  }

#if 0
  /* Include to determine the type of timeout if required. */
  int temp = 0;

  if (status_reg & SYS_STATUS_RXRFTO)
  {
    temp = 1;
  }
  else if (status_reg & SYS_STATUS_RXPTO)
  {
    temp = 2;
  }
#endif

  if (status_reg & SYS_STATUS_RXFCG)
  {
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* Read frame length. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

    /*
     * A frame has been received. Read it into the local buffer.
     */
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /*
       * LASIR DEBUG:
       * Print every valid UWB frame that reaches the DW1000.
       *
       * If this never appears in RTT Viewer, then the DWM1001 is not receiving
       * any decodable UWB frame with the current PHY configuration.
       */
      print_rx_frame_debug(rx_buffer, frame_len);

      /*
       * LASIR DEBUG:
       * Check whether this frame looks like a Bitcraze/Loco TWR POLL
       * addressed to this anchor.
       */
      if (is_bitcraze_loco_poll(rx_buffer, frame_len))
      {
        uint8 lps_seq;

        lps_seq = rx_buffer[BC_PAYLOAD_IDX + LPS_TWR_SEQ];

        DBG("POLL recebido BITCRAZE, anchor=%u, seq=%u\r\n",
            (unsigned int)ANCHOR_ID,
            (unsigned int)lps_seq);

        /*
         * IMPORTANT:
         * At this stage we only detect the Bitcraze POLL.
         *
         * Full Bitcraze-compatible response must be implemented next:
         *
         *   POLL -> ANSWER -> FINAL -> REPORT
         */
      }
    }
    else
    {
      /*
       * Frame is larger than local buffer. Clear and reset RX.
       */
      DBG("RX frame too large: len=%u, buffer=%u\r\n",
          (unsigned int)frame_len,
          (unsigned int)RX_BUF_LEN);

      dwt_rxreset();
      return 1;
    }

    /*
     * Original Decawave check:
     * Check that the frame is a poll sent by the "SS TWR initiator" example.
     *
     * As the sequence number field of the frame is not relevant, it is cleared
     * to simplify validation of the frame.
     */
    {
      uint8 rx_seq;

      rx_seq = rx_buffer[ALL_MSG_SN_IDX];

      rx_buffer[ALL_MSG_SN_IDX] = 0;

      if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32 resp_tx_time;
        int ret;

        DBG("POLL recebido DECAWAVE, seq=%u, len=%u\r\n",
            (unsigned int)rx_seq,
            (unsigned int)frame_len);

        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();

        /*
         * Compute response message transmission time.
         */
        resp_tx_time = (poll_rx_ts +
                       (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

        dwt_setdelayedtrxtime(resp_tx_time);

        /*
         * Response TX timestamp is the transmission time we programmed plus
         * the antenna delay.
         */
        resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /*
         * Write all timestamps in the response message.
         */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        /*
         * Write and send the response message.
         */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

        ret = dwt_starttx(DWT_START_TX_DELAYED);

        /*
         * If dwt_starttx() returns an error, abandon this ranging exchange and
         * proceed to the next one.
         */
        if (ret == DWT_SUCCESS)
        {
          /*
           * Poll DW1000 until TX frame sent event set.
           */
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
          {
          }

          /* Clear TXFRS event. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

          /*
           * Increment frame sequence number after transmission.
           */
          frame_seq_nb++;
        }
        else
        {
          /*
           * If we end up here then the delayed TX could not be scheduled.
           * POLL_RX_TO_RESP_TX_DLY_UUS may need to be increased.
           */
          DBG("ERRO: dwt_starttx(DWT_START_TX_DELAYED) falhou\r\n");

          /* Reset RX to properly reinitialise LDE operation. */
          dwt_rxreset();
        }
      }
    }
  }
  else
  {
    /*
     * Clear RX error events in the DW1000 status register.
     */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

    /*
     * Reset RX to properly reinitialise LDE operation.
     */
    dwt_rxreset();
  }

  return 1;
}

/* ----------------------------------------------------------------------------
 * Timestamp functions
 * ----------------------------------------------------------------------------
 */

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        This function assumes timestamps are 40 bits.
*
* @param  none
*
* @return  64-bit value of the read timestamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;

  dwt_readrxtimestamp(ts_tab);

  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }

  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value.
*        The least significant byte is at the lower address.
*
* @param  ts_field  pointer to first byte of timestamp field
* @param  ts         timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;

  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

/* ----------------------------------------------------------------------------
 * LASIR debug/helper functions
 * ----------------------------------------------------------------------------
 */

static uint16 read_u16_le(const uint8 *buf)
{
  return ((uint16)buf[0]) |
         (((uint16)buf[1]) << 8);
}

static uint64 read_u64_le(const uint8 *buf)
{
  uint64 value = 0;
  int i;

  for (i = 7; i >= 0; i--)
  {
    value <<= 8;
    value |= buf[i];
  }

  return value;
}

static void print_rx_frame_debug(const uint8 *buf, uint32 frame_len)
{
  /*
   * Print the first bytes of every received frame.
   *
   * This helps distinguish:
   * - no UWB reception at all
   * - UWB reception with wrong protocol
   * - possible Bitcraze/Loco frames
   * - original Decawave frames
   */
  if (frame_len >= 8)
  {
    DBG("RX len=%u bytes=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        (unsigned int)frame_len,
        (unsigned int)buf[0],
        (unsigned int)buf[1],
        (unsigned int)buf[2],
        (unsigned int)buf[3],
        (unsigned int)buf[4],
        (unsigned int)buf[5],
        (unsigned int)buf[6],
        (unsigned int)buf[7]);
  }
  else if (frame_len >= 4)
  {
    DBG("RX len=%u bytes=%02X %02X %02X %02X\r\n",
        (unsigned int)frame_len,
        (unsigned int)buf[0],
        (unsigned int)buf[1],
        (unsigned int)buf[2],
        (unsigned int)buf[3]);
  }
  else
  {
    DBG("RX len=%u\r\n", (unsigned int)frame_len);
  }
}

static int is_bitcraze_loco_poll(const uint8 *buf, uint32 frame_len)
{
  uint16 fcf;
  uint16 pan;
  uint64 dst;
  uint8 type;

  /*
   * Need at least:
   * 2 bytes FCF
   * 1 byte sequence
   * 2 bytes PAN
   * 8 bytes destination
   * 8 bytes source
   * 2 bytes payload minimum: type + seq
   */
  if (frame_len < BC_MIN_FRAME_LEN)
  {
    return 0;
  }

  fcf = read_u16_le(&buf[BC_FCF_IDX]);
  pan = read_u16_le(&buf[BC_PAN_IDX]);
  dst = read_u64_le(&buf[BC_DEST_ADDR_IDX]);
  type = buf[BC_PAYLOAD_IDX + LPS_TWR_TYPE];

  if (fcf != BC_EXPECTED_FCF)
  {
    return 0;
  }

  if (pan != PAN_ID)
  {
    return 0;
  }

  if (dst != MY_ADDRESS)
  {
    return 0;
  }

  if (type != LPS_TWR_POLL)
  {
    return 0;
  }

  return 1;
}

/* ----------------------------------------------------------------------------
 * FreeRTOS task
 * ----------------------------------------------------------------------------
 */

/**@brief SS TWR Responder task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_responder_task_function(void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  SEGGER_RTT_Init();

  DBG("\r\n");
  DBG("========================================\r\n");
  DBG("DWM1001 SS RESPONDER DEBUG START\r\n");
  DBG("ANCHOR_ID=%u\r\n", (unsigned int)ANCHOR_ID);
  DBG("MY_ADDRESS anchor base + id\r\n");
  DBG("Waiting for UWB frames...\r\n");
  DBG("========================================\r\n");

  while (true)
  {
    ss_resp_run();

    /*
     * Delay task for a given number of ticks.
     */
    vTaskDelay(RNG_DELAY_MS);

    /*
     * Tasks must never return.
     */
  }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* This file is based on the Decawave SS TWR responder example.
*
* LASIR modifications:
*
* 1. Added generic RX frame debug using SEGGER RTT:
*
*      RX len=...
*
*    If this appears, the DW1000 is receiving decodable UWB frames.
*
* 2. Added Bitcraze/Loco POLL detection:
*
*      POLL recebido BITCRAZE...
*
*    If this appears, the LP deck is transmitting and the DWM1001 can decode
*    the frame with the current PHY and addressing assumptions.
*
* 3. Added original Decawave POLL detection:
*
*      POLL recebido DECAWAVE...
*
*    If this appears while using another DWM1001 with ss_twr_init, the original
*    Decawave ranging example is working.
*
* 4. This file does not yet send a Bitcraze-compatible ANSWER/REPORT.
*    That is the next implementation step after confirming POLL reception.
*
****************************************************************************************************************************************************/
