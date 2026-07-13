# DIAG F3 — Diagnóstico "FINAL não chegou" (anchor DWM1001)

Análise do código do responder F3 (`examples/ss_twr_resp/ss_resp_main.c`)
face à evidência de banco: `ANSWER enviado` em todos os ciclos, FINAL nunca
processado (`FINAL nao chegou (5 frames descartadas)` dominante,
`FINAL timeout/erro, status=...` minoritário).

## 1. Descodificação dos SYS_STATUS — CONFIRMADA contra `deca_regs.h`

Verificado contra os defines oficiais (`deca_driver/deca_regs.h:254-312`):

| status | bits RX relevantes | interpretação |
|---|---|---|
| `0x00820072` | `RXRFTO` (0x00020000) | timeout puro — nada detetado ✔ |
| `0x04800172` | `RXSFDTO` (0x04000000) + `RXPRD` (0x00000100) | preâmbulo detetado, SFD nunca válido ✔ |
| `0x00801372` | `RXPHE` (0x00001000) + `RXSFDD` (0x00000200) + `RXPRD` (0x00000100) | preâmbulo + SFD OK, PHY header corrompido ✔ |

A descodificação do briefing estava correta. **Descoberta adicional**: os
nibbles finais `…72` e o `0x00800000` não são eventos RX — são bits *stale*:
`TXFRB|TXPRS|TXPHS` (0x70) do TX do ANSWER (o código só limpava `TXFRS`),
`CPLOCK` (0x2) e `SLP2INIT` (0x800000). Poluíam todas as leituras. O patch
passa a limpar `SYS_STATUS_ALL_TX` depois do ANSWER e
`SYS_STATUS_ALL_RX_GOOD` a cada frame, para cada status registado refletir
só o evento corrente.

## 2. Respostas às perguntas do briefing (a partir do código)

### Payload do ANSWER — exatamente 2 bytes (hipótese 2 REFUTADA no anchor)

`BC_ANSWER_FRAME_LEN = 21 (header) + 2 (type+seq) + 2 (CRC) = 25` no ar.
O tag calcula `dataLength - MAC802154_HEADER_LENGTH = 23 - 21 = 2`, que não
é `> 3`, logo não cai no caminho LPP do `lpsTwrTag.c`. O ANSWER está
conforme o protocolo.

### Config de rádio — sem drift (hipótese 4 REFUTADA)

Entre o TX do ANSWER e o RX do FINAL só se chama `dwt_setrxaftertxdelay(0)`
e `dwt_setrxtimeout(...)`. `dwt_config_t`, smart power e PHR mode são
configurados uma única vez no `main.c` e nunca retocados.

### "5 frames descartadas" — era contagem, não tempo (BUG CONFIRMADO)

`BC_FINAL_MAX_DISCARDS` era 4 (a mensagem dispara em 5), e cada
`dwt_rxenable(DWT_START_RX_IMMEDIATE)` **reinicia o timeout HW de 5000 uus
do zero**. A "janela" do FINAL era ilimitada no tempo e limitada apenas por
número de frames. Corrigido: deadline absoluto de
`FINAL_RX_TOTAL_WINDOW_UUS` (5000 uus) ancorado no instante de TX agendado
do ANSWER, verificado com `dwt_readsystimestamphi32()`; a contagem passou a
válvula de segurança alta (16).

### Instrumentação — incompleta (CONFIRMADO)

Só o caminho timeout/erro logava `SYS_STATUS`. O caminho dominante
(descartes) não registava **nada** sobre os frames descartados — nem
status, nem um byte. Este era o ponto cego que impedia o diagnóstico.

### `anchor=0` nos logs

O repo tinha `ANCHOR_ID 1`; o build do banco foi editado localmente para 0.
Não afeta o protocolo (o anchor responde ao que for endereçado a si), mas o
repo foi alinhado para `ANCHOR_ID 0`.

## 3. Interpretação-chave e hipóteses vivas

`FINAL nao chegou (5 frames descartadas)` significa que chegaram 5 frames
**com CRC bom** dentro da espera e nenhum passou a validação de FINAL
(`is_bitcraze_frame_for_me`: FCF, PAN, dest=anchor, src=tag, type=0x03,
seq do POLL). Duas hipóteses compatíveis com a evidência:

- **(a) Os frames descartados SÃO os FINALs**, a falhar a validação num
  campo (seq? src? type?). Seria um bug de validação/protocolo no anchor —
  os bytes do dump vão mostrar exatamente o campo divergente.
- **(b) Os frames descartados são tráfego do round-robin do deck** (POLLs
  para outros IDs, ou POLLs novos para nós com seq novo) e o FINAL
  verdadeiro nunca chega ou falha ao nível PHY. Os status minoritários
  `RXSFDTO`/`RXPHE` encaixam aqui: re-armar o RX a meio de um frame alheio
  já em curso produz exatamente SFD timeout / PHY header error.

Nota sobre (b): um POLL novo *para nós* durante a espera era descartado e
desperdiçado — a troca nova morria também. O patch passa a reiniciar a
troca imediatamente nesse caso (o timestamp de RX está fresco e a janela de
1100 uus intacta). **[REVERTIDO — ver §6: este reinício era uma race
condition que engolia o outcome do ciclo interrompido.]**

Não é possível decidir (a) vs (b) sem os bytes dos frames descartados —
por isso o entregável é instrumentação, não um fix especulativo.

## 4. Como ler o output novo no banco

Falha de troca produz um bloco único (impresso *depois* de a troca morrer,
nunca dentro das janelas de timing):

```
FINAL falhou (<motivo>), seq=N, status=XXXXXXXX, descartadas=D, polls_ignorados=P
  dsc[0]: st=XXXXXXXX len=25 fcf=DC41 mseq=M dst=08 src=00 type=03 pseq=N
  ...
```

Motivos: `timeout/erro RX` (timeout HW de 5000 uus ou erro de RX — o
`status` diz qual), `janela esgotada` (deadline temporal excedido com
tráfego a chegar), `valvula de descartes` (>16 frames — anómalo).
Os dumps detalhados limitam-se aos primeiros 10 falhanços após boot; depois
fica só a linha-resumo. Uma troca com sucesso após percalços imprime
`(troca com D descartes, P polls ignorados)` a seguir ao `REPORT enviado`.
(O campo chamava-se `reinicios` antes do §6; a semântica mudou de "trocas
reiniciadas" para "POLLs próprios ignorados durante a espera".)

Leitura dos campos `dsc[]` (dst/src = byte baixo do endereço: anchors
0x00-0x07, tag 0x08):

| padrão observado | conclusão |
|---|---|
| `dst=00 src=08 type=03 pseq=N` (N = seq da troca) | é o FINAL a ser rejeitado → bug de validação no anchor; comparar campo a campo com `is_bitcraze_frame_for_me` |
| `dst=00 src=08 type=03 pseq≠N` | FINAL com seq errado → dessincronização de seq entre POLL e FINAL (olhar lado do tag) |
| `dst≠00 type=01` | POLLs para outros anchors → o tag nunca enviou o FINAL para nós; investigar aceitação do ANSWER no tag (timing da janela de RX do deck vs 1100 uus) |
| `polls_ignorados>0` frequente | o deck volta a fazer POLL a este anchor dentro da espera — confirma que o tag abandonou a troca anterior |
| `status` com `RXSFDTO`/`RXPHE` (agora sem bits stale) | falha ao nível PHY na receção — com os bits limpos, o padrão por evento passa a ser fiável |

## 5. Alterações feitas (patch mínimo, sem tocar no ciclo TWR)

Tudo em `examples/ss_twr_resp/ss_resp_main.c`:

1. Captura diferida (`bc_diag_capture`/`bc_diag_dump`): status + primeiros
   23 bytes de cada frame descartado, impressos só após a falha; budget de
   10 dumps completos por boot.
2. Deadline temporal `FINAL_RX_TOTAL_WINDOW_UUS = 5000` uus ancorado no TX
   do ANSWER; contagem relegada a válvula (16).
3. `SYS_STATUS` capturado e impresso em **todos** os caminhos de falha;
   limpeza de bits stale (TX e good-RX) para statuses limpos por evento.
4. Reinício imediato da troca quando chega POLL novo para este anchor
   durante a espera do FINAL. **[REVERTIDO — ver §6.]**
5. `ANCHOR_ID` 1 → 0 (board do banco).

Intocado: FCF `0xDC41`, `dwt_config_t`/`main.c`, `RNG_DELAY_MS 1`, delays
de 1100 uus, antenna delays, e a própria validação do FINAL (agora apenas
observável).

## 6. Race condition do reinício — CONFIRMADA no banco e corrigida

A evidência de banco (89 ciclos) condenou o item 4 do §5 com correlação
100%, sem exceções:

```
dup_then_reinicios1: 16      nodup_then_reinicios1: 0
dup_then_reinicios0: 0       nodup_then_reinicios0: 44
```

`reinicios=1` ocorria sempre e só quando havia dois `ANSWER enviado`
consecutivos sem outcome entre eles. Os 19 ANSWERs "engolidos" (89 enviados
− 70 outcomes) eram exatamente os ciclos interrompidos pelo reinício: o
branch fazia `break`+`continue` e servia o POLL novo **sem nunca logar o
outcome do ciclo abandonado**. Além disso `discards` e o buffer `bc_diag`
não eram limpos no reinício, contaminando as contagens e dumps do ciclo
seguinte. (Não havia reset de rádio nenhum no reinício — o custo não era
tempo, era estatística corrompida.)

Correção em `bc_handle_twr_poll()`:

- **Exclusão mútua de ciclo**: um POLL novo para este anchor durante a
  espera do FINAL é agora **ignorado** — contado em `polls_ignorados` e
  descartado como qualquer frame alheia. O ciclo em curso corre sempre até
  um término logado.
- O loop externo de reinício, `restart` e `restarts` desapareceram; a
  função é single-pass: um POLL aceite → um ANSWER → exatamente um outcome
  (`FINAL recebido` ou `FINAL falhou (...)`) — garantido por construção.
- `tools/check_f3_log.py` verifica mecanicamente o critério de aceitação
  (zero ANSWERs sem outcome) e produz a contagem-padrão sobre um dump RTT.

Só depois de confirmar no banco **0 ciclos sem outcome** é que faz sentido
avaliar se os `00806F02` restantes justificam alargar
`FINAL_RX_TOTAL_WINDOW_UUS`.
