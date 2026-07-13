/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           LASIR debug version for DWM1001-DEV.
*
*           This version:
*           - Keeps the original Decawave SS TWR responder behaviour.
*           - Uses SEGGER RTT instead of printf/stdout.
*           - Prints every received UWB frame that is not handled.
*           - Implements the full Bitcraze/Loco TWR anchor cycle:
*
*               POLL -> ANSWER -> FINAL -> REPORT
*
*           The anchor is passive: it only captures 40-bit hardware
*           timestamps (poll RX, answer TX, final RX) and returns them in
*           the REPORT payload. Distance is computed on the Crazyflie.
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* Modified for LASIR / DWM1001-Crazyflie debug.
*/

#include "sdk_config.h"

#include <stdint.h>
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

/*
 * Inter-ranging delay period, in milliseconds.
 *
 * The LP deck polls anchors in a fast round-robin. With the original 80 ms
 * pause between ss_resp_run() calls the anchor would be deaf between
 * exchanges and miss most POLLs, so we only yield for one tick.
 */
#define RNG_DELAY_MS 1

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
#define ANCHOR_ID        0        /* <<< CHANGE THIS PER BOARD: 0, 1, 2, 3 */

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

#define BC_HEADER_LEN           21

/* CRC appended automatically by the DW1000. */
#define BC_CRC_LEN              2

/*
 * Bitcraze/Loco TWR timing.
 *
 * Delays from the HW RX timestamp of the incoming frame to the scheduled HW
 * TX time of the reply, in UWB microseconds. 1100 uus is the value already
 * proven safe for nRF/DWM1001 delayed TX in this project.
 */
#define POLL_RX_TO_ANSWER_TX_DLY_UUS   1100
#define FINAL_RX_TO_REPORT_TX_DLY_UUS  1100

/*
 * RX timeout per rxenable while waiting for the FINAL after our ANSWER, in
 * UWB microseconds. Not specified by the Bitcraze protocol; 5000 uus is a
 * wide margin over the tag turnaround and may be tuned on the bench.
 * Backstop against pure silence; the exchange as a whole is bounded by
 * FINAL_RX_TOTAL_WINDOW_UUS below.
 */
#define FINAL_RX_TIMEOUT_UUS           5000

/*
 * Total time budget for the FINAL, measured from the scheduled ANSWER TX
 * time. Each discarded frame restarts the per-rxenable HW timeout above, so
 * without this absolute deadline the wait would be bounded by frame count
 * instead of time.
 */
#define FINAL_RX_TOTAL_WINDOW_UUS      5000

/*
 * Safety valve only: frames not addressed to us can appear between our
 * ANSWER and the FINAL (the deck polls anchor IDs 0-7 in round-robin). The
 * exit mechanism is the time deadline above; this bound just guarantees the
 * loop terminates even if the deadline logic misbehaves.
 */
#define BC_FINAL_MAX_DISCARDS          16

/* ----------------------------------------------------------------------------
 * Diagnostics for the FINAL wait
 * ----------------------------------------------------------------------------
 *
 * Frames discarded while waiting for the FINAL are captured here (a memcpy,
 * no RTT traffic) and printed in one block only after the exchange has
 * failed, so logging never eats into the TWR timing windows.
 */

#define BC_DIAG_MAX_FRAMES   8

/* Header + payload type + payload seq: enough to classify any frame. */
#define BC_DIAG_HDR_BYTES    (BC_PAYLOAD_IDX + 2)

/* Full per-frame dumps after boot; afterwards only the summary line. */
#define BC_DIAG_DUMP_BUDGET  10

typedef struct {
  uint32 status;                    /* SYS_STATUS when the frame was read */
  uint16 len;
  uint8  bytes[BC_DIAG_HDR_BYTES];
} bc_diag_frame_t;

static bc_diag_frame_t bc_diag[BC_DIAG_MAX_FRAMES];
static int bc_diag_count;
static int bc_diag_dumps_left = BC_DIAG_DUMP_BUDGET;

/*
 * REPORT payload, starting at payload byte 2 (after type + seq).
 * Must match lpsTwrTagReportPayload_t in bitcraze/crazyflie-firmware.
 */
typedef struct {
    uint8_t pollRx[5];    /* 40-bit timestamp */
    uint8_t answerTx[5];  /* 40-bit timestamp */
    uint8_t finalRx[5];   /* 40-bit timestamp */
    float   pressure;     /* 0.0f (no barometer) */
    float   asl;          /* 0.0f */
    uint8_t pressure_ok;  /* 0 */
} __attribute__((packed)) lpsTwrTagReportPayload_t;

#define BC_ANSWER_FRAME_LEN  (BC_HEADER_LEN + 2 + BC_CRC_LEN)
#define BC_REPORT_FRAME_LEN  (BC_HEADER_LEN + 2 + \
                              sizeof(lpsTwrTagReportPayload_t) + BC_CRC_LEN)

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

/* Bitcraze/Loco TWR exchange timestamps. */
static uint64 bc_poll_rx_ts;
static uint64 bc_answer_tx_ts;
static uint64 bc_final_rx_ts;

/* TX buffer for Bitcraze/Loco frames (ANSWER and REPORT). */
static uint8 bc_tx_frame[BC_REPORT_FRAME_LEN];

/* ----------------------------------------------------------------------------
 * Static function declarations
 * ----------------------------------------------------------------------------
 */

static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

static uint16 read_u16_le(const uint8 *buf);
static uint64 read_u64_le(const uint8 *buf);
static void write_u64_le(uint8 *buf, uint64 value);
static void ts_to_bytes40(uint8 *dst, uint64 ts);

static void print_rx_frame_debug(const uint8 *buf, uint32 frame_len);
static int is_bitcraze_loco_poll(const uint8 *buf, uint32 frame_len);
static int is_bitcraze_frame_for_me(const uint8 *buf, uint32 frame_len,
                                    uint8 type, uint8 seq);

static void bc_build_header(uint8 *buf, uint8 mac_seq);
static void bc_handle_twr_poll(void);

static void bc_diag_capture(uint32 status, uint32 frame_len, const uint8 *buf);
static void bc_diag_dump(const char *reason, uint8 seq, uint32 last_status,
                         int discards, int own_polls_ignored);

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
       * Bitcraze/Loco TWR POLL addressed to this anchor: run the whole
       * ANSWER -> FINAL -> REPORT exchange now. The ANSWER TX window is
       * only POLL_RX_TO_ANSWER_TX_DLY_UUS wide, so no logging may happen
       * before the exchange is scheduled inside bc_handle_twr_poll().
       */
      if (is_bitcraze_loco_poll(rx_buffer, frame_len))
      {
        bc_handle_twr_poll();
        return 1;
      }

      /*
       * LASIR DEBUG:
       * Print every valid UWB frame that is not a Bitcraze POLL for us.
       *
       * If this never appears in RTT Viewer, then the DWM1001 is not receiving
       * any decodable UWB frame with the current PHY configuration.
       */
      print_rx_frame_debug(rx_buffer, frame_len);
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

static void write_u64_le(uint8 *buf, uint64 value)
{
  int i;

  for (i = 0; i < 8; i++)
  {
    buf[i] = (uint8)(value >> (i * 8));
  }
}

/*
 * Write a 40-bit DW1000 timestamp as 5 bytes, least significant byte first.
 * Note: resp_msg_set_ts() only writes 4 bytes and cannot be reused here.
 */
static void ts_to_bytes40(uint8 *dst, uint64 ts)
{
  int i;

  for (i = 0; i < 5; i++)
  {
    dst[i] = (uint8)(ts >> (i * 8));
  }
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

/*
 * Check that a frame is a Bitcraze/Loco frame of the given type and payload
 * sequence number, sent by the tag to this anchor. Used to match the FINAL
 * to the POLL currently being served.
 */
static int is_bitcraze_frame_for_me(const uint8 *buf, uint32 frame_len,
                                    uint8 type, uint8 seq)
{
  if (frame_len < BC_MIN_FRAME_LEN)
  {
    return 0;
  }

  if (read_u16_le(&buf[BC_FCF_IDX]) != BC_EXPECTED_FCF)
  {
    return 0;
  }

  if (read_u16_le(&buf[BC_PAN_IDX]) != PAN_ID)
  {
    return 0;
  }

  if (read_u64_le(&buf[BC_DEST_ADDR_IDX]) != MY_ADDRESS)
  {
    return 0;
  }

  if (read_u64_le(&buf[BC_SRC_ADDR_IDX]) != TAG_ADDRESS)
  {
    return 0;
  }

  if (buf[BC_PAYLOAD_IDX + LPS_TWR_TYPE] != type)
  {
    return 0;
  }

  if (buf[BC_PAYLOAD_IDX + LPS_TWR_SEQ] != seq)
  {
    return 0;
  }

  return 1;
}

/* ----------------------------------------------------------------------------
 * Bitcraze/Loco TWR anchor exchange
 * ----------------------------------------------------------------------------
 */

/*
 * Write the 21-byte IEEE 802.15.4 header used by the Bitcraze/Loco frames:
 * FCF 0xDC41 (data frame, PAN compression, 64-bit dest + src addresses),
 * destination = tag, source = this anchor.
 */
static void bc_build_header(uint8 *buf, uint8 mac_seq)
{
  buf[BC_FCF_IDX]     = 0x41;
  buf[BC_FCF_IDX + 1] = 0xDC;
  buf[BC_SEQ_IDX]     = mac_seq;
  buf[BC_PAN_IDX]     = (uint8)(PAN_ID & 0xFF);
  buf[BC_PAN_IDX + 1] = (uint8)(PAN_ID >> 8);

  write_u64_le(&buf[BC_DEST_ADDR_IDX], TAG_ADDRESS);
  write_u64_le(&buf[BC_SRC_ADDR_IDX], MY_ADDRESS);
}

/*
 * Capture one frame discarded during the FINAL wait. Pure memory copy — no
 * RTT traffic — so it is safe inside the TWR timing windows. The captured
 * frames are printed later by bc_diag_dump(), after the exchange has failed.
 */
static void bc_diag_capture(uint32 status, uint32 frame_len, const uint8 *buf)
{
  bc_diag_frame_t *slot;
  uint32 n;

  if (bc_diag_count >= BC_DIAG_MAX_FRAMES)
  {
    return;
  }

  slot = &bc_diag[bc_diag_count++];
  slot->status = status;
  slot->len = (uint16)frame_len;

  memset(slot->bytes, 0, BC_DIAG_HDR_BYTES);

  if (buf != NULL)
  {
    n = (frame_len < BC_DIAG_HDR_BYTES) ? frame_len : BC_DIAG_HDR_BYTES;
    memcpy(slot->bytes, buf, n);
  }
}

/*
 * Print the failure summary plus, while the dump budget lasts, one line per
 * discarded frame: SYS_STATUS at reception, length, FCF, MAC seq, low byte
 * of dest and src addresses (the discriminating byte: anchors 0x00-0x07,
 * tag 0x08), payload type and payload seq. This tells apart at a glance a
 * FINAL failing validation (dst=us, src=08, type=03) from deck round-robin
 * traffic to other anchors (dst!=us, type=01).
 */
static void bc_diag_dump(const char *reason, uint8 seq, uint32 last_status,
                         int discards, int own_polls_ignored)
{
  int i;

  DBG("FINAL falhou (%s), seq=%u, status=%08X, descartadas=%d, polls_ignorados=%d\r\n",
      reason,
      (unsigned int)seq,
      (unsigned int)last_status,
      discards,
      own_polls_ignored);

  if (bc_diag_dumps_left <= 0)
  {
    return;
  }

  bc_diag_dumps_left--;

  for (i = 0; i < bc_diag_count; i++)
  {
    const bc_diag_frame_t *f = &bc_diag[i];

    DBG("  dsc[%d]: st=%08X len=%u fcf=%02X%02X mseq=%u dst=%02X src=%02X type=%02X pseq=%u\r\n",
        i,
        (unsigned int)f->status,
        (unsigned int)f->len,
        (unsigned int)f->bytes[BC_FCF_IDX + 1],
        (unsigned int)f->bytes[BC_FCF_IDX],
        (unsigned int)f->bytes[BC_SEQ_IDX],
        (unsigned int)f->bytes[BC_DEST_ADDR_IDX],
        (unsigned int)f->bytes[BC_SRC_ADDR_IDX],
        (unsigned int)f->bytes[BC_PAYLOAD_IDX + LPS_TWR_TYPE],
        (unsigned int)f->bytes[BC_PAYLOAD_IDX + LPS_TWR_SEQ]);
  }
}

/*
 * Serve one full TWR exchange after a valid POLL has been received into
 * rx_buffer:
 *
 *   POLL (already in rx_buffer) -> ANSWER -> FINAL -> REPORT
 *
 * The anchor only timestamps; the tag computes the distance from the three
 * timestamps returned in the REPORT.
 *
 * One call serves exactly one exchange and always ends in a logged outcome:
 * "FINAL recebido" or "FINAL falhou". A fresh POLL for this anchor arriving
 * while the FINAL is pending is ignored (counted in polls_ignorados), never
 * served, so exchanges cannot overlap and no outcome is ever swallowed.
 *
 * Timing note: the ANSWER must go on air POLL_RX_TO_ANSWER_TX_DLY_UUS after
 * the POLL RX timestamp, so everything up to dwt_starttx() is time critical
 * and must not be interleaved with RTT logging.
 */
static void bc_handle_twr_poll(void)
{
  uint8 lps_seq;
  uint32 answer_tx_time;
  uint32 report_tx_time;
  uint32 final_deadline;
  uint32 frame_len;
  int discards;
  int own_polls_ignored;
  int ret;
  lpsTwrTagReportPayload_t report;

  bc_diag_count = 0;
  discards = 0;
  own_polls_ignored = 0;

  lps_seq = rx_buffer[BC_PAYLOAD_IDX + LPS_TWR_SEQ];

  /* --------------------------------------------------------------------
   * ANSWER — time critical section (no logging until dwt_starttx()).
   * --------------------------------------------------------------------
   */
  bc_poll_rx_ts = get_rx_timestamp_u64();

  answer_tx_time =
      (uint32)((bc_poll_rx_ts +
               (POLL_RX_TO_ANSWER_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

  dwt_setdelayedtrxtime(answer_tx_time);

  /*
   * ANSWER TX timestamp = programmed time (LSB masked, as the DW1000
   * ignores it) shifted back, plus the antenna delay. Same pattern as the
   * original single-sided example.
   */
  bc_answer_tx_ts =
      (((uint64)(answer_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  bc_build_header(bc_tx_frame, frame_seq_nb);
  bc_tx_frame[BC_PAYLOAD_IDX + LPS_TWR_TYPE] = LPS_TWR_ANSWER;
  bc_tx_frame[BC_PAYLOAD_IDX + LPS_TWR_SEQ]  = lps_seq;

  dwt_writetxdata(BC_ANSWER_FRAME_LEN, bc_tx_frame, 0);
  dwt_writetxfctrl(BC_ANSWER_FRAME_LEN, 0, 1);

  /*
   * Re-arm the receiver in hardware right after the ANSWER so the FINAL
   * is caught, with a timeout so a lost FINAL cannot block the anchor.
   */
  dwt_setrxaftertxdelay(0);
  dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

  ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

  if (ret != DWT_SUCCESS)
  {
    /*
     * The ANSWER TX window was missed (e.g. delayed by logging or the
     * scheduler). Abandon this exchange and go back to waiting for POLLs.
     */
    DBG("ERRO: ANSWER dwt_starttx falhou, seq=%u\r\n",
        (unsigned int)lps_seq);

    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxtimeout(0);
    return;
  }

  /*
   * Absolute deadline for the FINAL, in the same >>8 units used by
   * dwt_setdelayedtrxtime() and dwt_readsystimestamphi32(). Each
   * discarded frame restarts the per-rxenable HW timeout, so this is
   * what actually bounds the wait in time.
   */
  final_deadline = answer_tx_time +
      (uint32)((FINAL_RX_TOTAL_WINDOW_UUS * UUS_TO_DWT_TIME) >> 8);

  /* Poll DW1000 until the ANSWER is on air. */
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
  {
  }

  /*
   * Clear ALL TX event bits, not just TXFRS: leftover TXFRB/TXPRS/TXPHS
   * would pollute every SYS_STATUS captured during the FINAL wait.
   */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);

  DBG("ANSWER enviado, anchor=%u, seq=%u\r\n",
      (unsigned int)ANCHOR_ID,
      (unsigned int)lps_seq);

  /* --------------------------------------------------------------------
   * FINAL — wait for the tag's FINAL for this exchange.
   * --------------------------------------------------------------------
   */
  for (;;)
  {
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    }

    if (!(status_reg & SYS_STATUS_RXFCG))
    {
      /* HW timeout or RX error: abandon this exchange without blocking. */
      dwt_write32bitreg(SYS_STATUS_ID,
                        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      dwt_rxreset();
      dwt_setrxtimeout(0);

      bc_diag_dump("timeout/erro RX", lps_seq, status_reg,
                   discards, own_polls_ignored);
      return;
    }

    /*
     * Clear every good-RX event bit (not just RXFCG) so the status
     * captured for the next event reflects that event only.
     */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      if (is_bitcraze_frame_for_me(rx_buffer, frame_len,
                                   LPS_TWR_FINAL, lps_seq))
      {
        break;
      }

      if (is_bitcraze_loco_poll(rx_buffer, frame_len))
      {
        /*
         * Fresh POLL for this anchor while the FINAL for lps_seq is still
         * pending. Serving it would overlap two exchanges and swallow this
         * cycle's outcome (the reinicios=1 race), so it is only counted
         * and then discarded below like any other frame. The current
         * exchange always runs to a logged end first.
         */
        own_polls_ignored++;
      }

      bc_diag_capture(status_reg, frame_len, rx_buffer);
    }
    else
    {
      bc_diag_capture(status_reg, frame_len, NULL);
    }

    discards++;

    if ((int32_t)(dwt_readsystimestamphi32() - final_deadline) > 0)
    {
      dwt_setrxtimeout(0);
      bc_diag_dump("janela esgotada", lps_seq, status_reg,
                   discards, own_polls_ignored);
      return;
    }

    if (discards > BC_FINAL_MAX_DISCARDS)
    {
      dwt_setrxtimeout(0);
      bc_diag_dump("valvula de descartes", lps_seq, status_reg,
                   discards, own_polls_ignored);
      return;
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }

  /* --------------------------------------------------------------------
   * REPORT — time critical section (no logging until dwt_starttx()).
   * --------------------------------------------------------------------
   */
  bc_final_rx_ts = get_rx_timestamp_u64();

  report_tx_time =
      (uint32)((bc_final_rx_ts +
               (FINAL_RX_TO_REPORT_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

  dwt_setdelayedtrxtime(report_tx_time);

  memset(&report, 0, sizeof(report));
  ts_to_bytes40(report.pollRx, bc_poll_rx_ts);
  ts_to_bytes40(report.answerTx, bc_answer_tx_ts);
  ts_to_bytes40(report.finalRx, bc_final_rx_ts);
  report.pressure = 0.0f;
  report.asl = 0.0f;
  report.pressure_ok = 0;

  bc_build_header(bc_tx_frame, frame_seq_nb);
  bc_tx_frame[BC_PAYLOAD_IDX + LPS_TWR_TYPE] = LPS_TWR_REPORT;
  bc_tx_frame[BC_PAYLOAD_IDX + LPS_TWR_SEQ]  = lps_seq;
  memcpy(&bc_tx_frame[BC_PAYLOAD_IDX + 2], &report, sizeof(report));

  dwt_writetxdata(BC_REPORT_FRAME_LEN, bc_tx_frame, 0);
  dwt_writetxfctrl(BC_REPORT_FRAME_LEN, 0, 1);

  ret = dwt_starttx(DWT_START_TX_DELAYED);

  DBG("FINAL recebido, seq=%u\r\n", (unsigned int)lps_seq);

  if (ret != DWT_SUCCESS)
  {
    DBG("ERRO: REPORT dwt_starttx falhou, seq=%u\r\n", (unsigned int)lps_seq);

    dwt_forcetrxoff();
    dwt_rxreset();
    dwt_setrxtimeout(0);
    return;
  }

  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
  {
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

  DBG("REPORT enviado, anchor=%u, seq=%u\r\n",
      (unsigned int)ANCHOR_ID,
      (unsigned int)lps_seq);

  if (discards > 0 || own_polls_ignored > 0)
  {
    DBG("  (troca com %d descartes, %d polls ignorados)\r\n",
        discards, own_polls_ignored);
  }

  frame_seq_nb++;

  /* Back to waiting for POLLs with no RX timeout. */
  dwt_setrxtimeout(0);
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
* 2. Implemented the Bitcraze/Loco TWR anchor exchange
*    (POLL -> ANSWER -> FINAL -> REPORT) in bc_handle_twr_poll().
*    RTT shows one line per state transition:
*
*      ANSWER enviado...
*      FINAL recebido...   (or FINAL timeout/erro...)
*      REPORT enviado...
*
*    The anchor never computes distance; it returns the three 40-bit HW
*    timestamps (poll RX, answer TX, final RX) in the REPORT payload and the
*    Crazyflie does the math. Antenna delays are the uncalibrated defaults
*    from port_platform.h (calibration is a later phase).
*
* 3. Kept the original Decawave POLL detection and response:
*
*      POLL recebido DECAWAVE...
*
*    If this appears while using another DWM1001 with ss_twr_init, the original
*    Decawave ranging example is working.
*
****************************************************************************************************************************************************/
