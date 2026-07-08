# TESTING F3 — Validação no banco do responder TWR Bitcraze

Nada disto pode ser validado sem hardware. O código compila por inspeção
(+ syntax check com gcc no host); o comportamento em RF tem de ser
verificado com o setup real.

## O que está feito (compila, não testado em RF)

- `examples/ss_twr_resp/ss_resp_main.c` implementa o ciclo completo de
  anchor Bitcraze/Loco: **POLL → ANSWER → FINAL → REPORT**
  (`bc_handle_twr_poll()`).
- ANSWER e REPORT com TX agendado por HW (+1100 uus sobre o timestamp de
  RX), FCF `0xDC41`, endereços de 64 bits, PAN `0xbccf`.
- REPORT devolve os três timestamps de 40 bits (`pollRx`, `answerTx`,
  `finalRx`) no payload packed esperado pelo `lpsTwrTag.c` do Crazyflie;
  campos de pressão a zero.
- Filtragem por `destAddress == MY_ADDRESS`, `src == tag`, e seq do payload
  igual ao do POLL em curso; frames alheias (round-robin dos IDs 0-7) são
  descartadas com guarda; timeout de RX de 5000 uus depois do ANSWER evita
  bloqueio se o FINAL se perder.
- Config de rádio, antenna delays (16456, por calibrar — F4) e o caminho
  Decawave legacy ficaram intactos.
- `RNG_DELAY_MS` 80 → 1 para o anchor não ficar surdo entre trocas.

## Setup

- DWM1001-DEV flashado com este firmware (`ANCHOR_ID` em
  `ss_resp_main.c`, atualmente **1**).
- Crazyflie 2.1 Brushless + LP deck (tag `0xbccf000000000008`).
- J-Link RTT Viewer ligado ao DWM1001.
- cfclient no PC.

## Procedimento

1. Compilar no SEGGER Embedded Studio (projeto
   `examples/ss_twr_resp/SES/ss_twr_resp.emProject`) e flashar.
2. Abrir o RTT Viewer; confirmar o banner `DWM1001 SS RESPONDER DEBUG START`.
3. Ligar o Crazyflie com o LP deck.
4. No RTT, procurar por troca completa com o **mesmo seq** nas três linhas:

   ```
   ANSWER enviado, anchor=1, seq=N
   FINAL recebido, seq=N
   REPORT enviado, anchor=1, seq=N
   ```

5. No cfclient → tab **Loco Positioning**: o anchor com o `ANCHOR_ID`
   flashado deve aparecer com **distância ≠ 0 que varia** quando o drone se
   move.

## Critério de sucesso

Distância ≠ 0 e a variar com o movimento. **A precisão não interessa nesta
fase** — os antenna delays estão por calibrar de propósito (F4); espera-se
um offset constante de talvez alguns metros.

## Se falhar — por onde olhar

| Sintoma no RTT | Suspeito | Ação |
|---|---|---|
| Nada recebido (`RX len=` nunca aparece) | PHY/hardware (regressão — isto estava validado) | Reflashar, verificar deck |
| `ERRO: ANSWER dwt_starttx falhou` | Janela de 1100 uus estourada | Ver se há logs antes do agendamento; como último recurso subir `POLL_RX_TO_ANSWER_TX_DLY_UUS` (mas o deck valida este timing do lado dele) |
| `ANSWER enviado` mas sempre `FINAL timeout/erro` | Deck rejeitou o ANSWER (FCF/endereços/seq) ou timeout curto | Confirmar bytes do ANSWER com sniffer; experimentar subir `FINAL_RX_TIMEOUT_UUS` |
| Troca completa no RTT mas distância nunca aparece no cfclient | Payload do REPORT (layout/endianness dos timestamps) | Comparar com `lpsTwrTagReportPayload_t` em `lpsTwrTag.h` da bitcraze |
| Distância aparece mas fixa/absurda | Timestamps trocados ou `answerTx` mal calculado | Verificar ordem pollRx/answerTx/finalRx no payload |

## Questões abertas a fechar no banco

1. `FINAL_RX_TIMEOUT_UUS = 5000` — valor proposto, não vem do protocolo;
   tunável.
2. Seq MAC (byte 2 do header) usa contador próprio do anchor — confirmar que
   o deck não o valida (pela leitura do `lpsTwrTag.c`, só valida o seq do
   payload).
3. `RNG_DELAY_MS = 1` — reverter para um valor maior se causar problemas de
   scheduling no FreeRTOS (LED task partilha a prioridade 2).
