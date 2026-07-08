# PLAN F3 — Resposta TWR Bitcraze no anchor DWM1001

Objetivo: estender o firmware `examples/ss_twr_resp/` de sniffer (deteção de POLL,
validada em hardware) para responder TWR completo compatível com o LP deck do
Crazyflie: **POLL → ANSWER → FINAL → REPORT**. O anchor é passivo: carimba
timestamps de HW de 40 bits e devolve-os no REPORT; a distância é calculada no
Crazyflie.

## O que existe hoje

- `examples/ss_twr_resp/ss_resp_main.c`
  - `ss_resp_run()`: RX imediato → busy-wait no `SYS_STATUS` → deteta POLL
    Bitcraze (só log via RTT) → responde ao POLL Decawave legacy (`ss_twr_init`).
  - Helpers reutilizados: `get_rx_timestamp_u64()`, `read_u16_le()`,
    `read_u64_le()`, `print_rx_frame_debug()`, `is_bitcraze_loco_poll()`,
    macro `DBG()` (SEGGER RTT).
  - Constantes já corretas: `PAN_ID 0xbccf`, `TAG_ADDRESS`, `MY_ADDRESS =
    ANCHOR_BASE + ANCHOR_ID`, tipos `LPS_TWR_*`, offsets `BC_*` do header
    802.15.4 com endereços de 64 bits, FCF esperado `0xDC41`,
    `UUS_TO_DWT_TIME 65536`, delay de resposta 1100 uus.
- `examples/ss_twr_resp/main.c` — config de rádio **validada em hardware**
  (canal 2, PRF 64M, PLEN 128, PAC8, preamble code 9, SFD standard, 6.8 Mbps,
  sfdTO 129, smart power on). **Não se altera.**
- `deca_driver/port/port_platform.h` — `TX_ANT_DLY`/`RX_ANT_DLY` = 16456,
  por calibrar de propósito (calibração é a F4). Reutilizados tal como estão.

## O que muda (tudo em `ss_resp_main.c`)

### Constantes e estruturas novas

- `POLL_RX_TO_ANSWER_TX_DLY_UUS 1100` e `FINAL_RX_TO_REPORT_TX_DLY_UUS 1100`.
- `FINAL_RX_TIMEOUT_UUS` (ver questões abertas).
- `lpsTwrTagReportPayload_t` packed: `pollRx[5]`, `answerTx[5]`, `finalRx[5]`,
  `float pressure = 0`, `float asl = 0`, `uint8 pressure_ok = 0`.

### Funções novas

- `write_u64_le()` — simétrico do `read_u64_le()` existente.
- `bc_build_header(buf, mac_seq)` — escreve os 21 bytes do header: FCF
  `41 DC` (LE de 0xDC41), seq MAC, PAN `cf bc`, dest = tag
  `0xbccf000000000008` (LE), src = `MY_ADDRESS` (LE).
- `ts_to_bytes40(dst, ts)` — timestamp de 40 bits em 5 bytes LE
  (o `resp_msg_set_ts()` existente escreve só 4 bytes; não serve).
- `bc_send_answer_and_report(...)` — estados B e C abaixo.

### Máquina de estados (uma chamada de `ss_resp_run()` faz o ciclo inteiro)

- **A — espera de POLL** (comportamento atual): `dwt_setrxtimeout(0)`, RX
  imediato. POLL Bitcraze válido → ler `poll_rx_ts` e agendar o ANSWER
  *antes* de qualquer log (orçamento de 1100 uus). Caminho Decawave legacy
  mantém-se para regressão com `ss_twr_init`.
- **B — ANSWER + espera do FINAL**:
  - `answer_tx_time = (poll_rx_ts + 1100 * UUS_TO_DWT_TIME) >> 8` →
    `dwt_setdelayedtrxtime()`.
  - `answer_tx_ts = ((answer_tx_time & 0xFFFFFFFEUL) << 8) + TX_ANT_DLY`
    (mesmo padrão do exemplo single-sided).
  - Payload só `[0]=0x02, [1]=seq` (o seq do POLL). Sem timestamps.
  - `dwt_setrxaftertxdelay(0)` + `dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS)` +
    `dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)` para o RX
    re-armar por HW e apanhar o FINAL.
  - Espera do FINAL: valida FCF/PAN/`dest == MY_ADDRESS`/`src == tag`/
    `tipo == 0x03`/`seq == seq do POLL`. Frames que não batem (round-robin
    para outros IDs) são descartadas e o RX re-armado, com contador de guarda.
    Timeout ou erro de RX → `dwt_rxreset()` e volta a A sem bloquear.
- **C — REPORT**:
  - `report_tx_time = (final_rx_ts + 1100 * UUS_TO_DWT_TIME) >> 8`.
  - Payload `[0]=0x04, [1]=seq` + struct packed a partir de `payload+2` com
    `pollRx`, `answerTx`, `finalRx` (40 bits, 5 bytes LE cada),
    `pressure=0.0f`, `asl=0.0f`, `pressure_ok=0`.
  - `dwt_starttx(DWT_START_TX_DELAYED)`, espera TXFRS, repõe
    `dwt_setrxtimeout(0)` e volta a A.

### Loop da task

- `RNG_DELAY_MS` reduzido de 80 → 1: com 80 ms de pausa entre chamadas o
  anchor ficaria surdo entre trocas e perderia a maioria dos POLLs do
  round-robin do deck. `vTaskDelay(1)` mantém o yield ao FreeRTOS.

## Riscos

- **Janela de 1100 uus** entre POLL RX e o arranque do ANSWER: qualquer
  `DBG()` antes do `dwt_starttx` pode estourar a janela. Todos os logs das
  transições ficam depois do agendamento/TX. Se no banco aparecer
  `dwt_starttx falhou`, é este o primeiro suspeito.
- **Tráfego intercalado** entre ANSWER e FINAL (deck faz POLL aos IDs 0-7):
  tratado com o loop de descarte + guarda; frames para outros anchors são
  ignoradas.
- **Endianness/alinhamento** do payload do REPORT: nRF52 é little-endian e a
  struct é packed; é copiada com `memcpy` para o buffer de TX.

## Questões abertas

1. **`FINAL_RX_TIMEOUT_UUS`** — o briefing não fixa o timeout de espera do
   FINAL depois do ANSWER. Proposta inicial: **5000 uus** (folga larga sobre o
   turnaround do deck; `dwt_setrxtimeout` é uint16, máx. ≈65 ms). Tunável no
   banco se houver timeouts ou lentidão a re-armar.
2. **Seq MAC (byte 2 do header)** das frames TX — o deck valida o seq do
   *payload*, não o do MAC; usa-se o `frame_seq_nb` próprio, incrementado por
   troca completa. A confirmar no banco que o deck não rejeita.
3. **`RNG_DELAY_MS` 80 → 1** — alteração deliberada fora do briefing (ver
   acima); reverter e medir se causar problemas de scheduling do FreeRTOS.

## Fora de âmbito (guardrails)

- Sem alteração da config de rádio nem de `main.c`.
- Sem TDoA, sem cálculo de distância no anchor, sem calibração de antenna
  delay (F4), um anchor só (`ANCHOR_ID` continua `#define`).

## Validação

Impossível validar sem hardware (DWM1001 + Crazyflie + LP deck + RTT Viewer).
Neste repositório só há projetos SES/Keil (sem build de linha de comandos);
o código é entregue compilável por inspeção + syntax check de melhor esforço.
O procedimento de teste no banco fica em `TESTING_F3.md`.
