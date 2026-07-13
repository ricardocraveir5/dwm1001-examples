#!/usr/bin/env python3
"""Verifica um log RTT do anchor F3 (ss_resp_main.c) contra o critério de
aceitação da correção da race condition: todo o "ANSWER enviado" tem de ser
seguido de exatamente um outcome ("FINAL recebido" ou "FINAL falhou") antes
do próximo ANSWER.

Uso:
    python3 tools/check_f3_log.py rtt_dump.txt
    cat rtt_dump.txt | python3 tools/check_f3_log.py

Aceita tanto o formato novo (polls_ignorados=P) como o antigo (reinicios=R),
para poder comparar sessões de antes e depois da correção.

Sai com código 0 se não houver ciclos sem outcome; 1 caso contrário.
"""

import re
import sys
from collections import Counter

RE_ANSWER = re.compile(r"ANSWER enviado, anchor=(\d+), seq=(\d+)")
RE_FINAL_OK = re.compile(r"FINAL recebido, seq=(\d+)")
RE_FINAL_FAIL = re.compile(
    r"FINAL falhou \(([^)]+)\), seq=(\d+), status=([0-9A-Fa-f]+), "
    r"descartadas=(\d+), (?:polls_ignorados|reinicios)=(\d+)"
)
RE_REPORT = re.compile(r"REPORT enviado, anchor=(\d+), seq=(\d+)")


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1], "r", errors="replace") as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    answers = 0
    finals_ok = 0
    reports = 0
    fail_by_reason = Counter()
    fail_by_status = Counter()
    polls_ignorados_total = 0
    polls_ignorados_cycles = 0

    # Pareamento ANSWER -> outcome.
    swallowed = []          # (linha, seq) de ANSWERs sem outcome
    orphan_outcomes = []    # (linha, texto) de outcomes sem ANSWER anterior
    pending = None          # (linha, seq) do ANSWER à espera de outcome

    for lineno, line in enumerate(lines, 1):
        m = RE_ANSWER.search(line)
        if m:
            answers += 1
            if pending is not None:
                swallowed.append(pending)
            pending = (lineno, int(m.group(2)))
            continue

        m = RE_FINAL_OK.search(line)
        if m:
            finals_ok += 1
            if pending is None:
                orphan_outcomes.append((lineno, line.strip()))
            pending = None
            continue

        m = RE_FINAL_FAIL.search(line)
        if m:
            reason, _seq, status, _disc, ignored = m.groups()
            fail_by_reason[reason] += 1
            fail_by_status[status.upper()] += 1
            n = int(ignored)
            polls_ignorados_total += n
            if n > 0:
                polls_ignorados_cycles += 1
            if pending is None:
                orphan_outcomes.append((lineno, line.strip()))
            pending = None
            continue

        m = RE_REPORT.search(line)
        if m:
            reports += 1

    # Um ANSWER no fim do log sem outcome pode ser só o corte do dump —
    # conta à parte, não como violação.
    tail_pending = pending

    fails = sum(fail_by_reason.values())
    outcomes = finals_ok + fails

    print("=== Contagem F3 ===")
    print(f"ANSWER enviado:            {answers}")
    print(f"FINAL recebido:            {finals_ok}")
    print(f"FINAL falhou (total):      {fails}")
    for reason, n in fail_by_reason.most_common():
        print(f"  - {reason}: {n}")
    print("FINAL falhou por status:")
    for status, n in fail_by_status.most_common():
        print(f"  - {status}: {n}")
    print(f"REPORT enviado:            {reports}")
    print(f"Outcomes logados:          {outcomes}")
    print(f"Ciclos SEM outcome:        {len(swallowed)}"
          + (" (+1 pendente no fim do log)" if tail_pending else ""))
    print(f"Ciclos com polls ignorados: {polls_ignorados_cycles} "
          f"(total de polls ignorados: {polls_ignorados_total})")

    if orphan_outcomes:
        print(f"\nAVISO: {len(orphan_outcomes)} outcome(s) sem ANSWER anterior "
              "(log truncado no início?):")
        for lineno, text in orphan_outcomes[:5]:
            print(f"  linha {lineno}: {text}")

    if swallowed:
        print("\nFALHA: ANSWERs engolidos (sem outcome antes do ANSWER "
              "seguinte):")
        for lineno, seq in swallowed[:20]:
            print(f"  linha {lineno}: ANSWER seq={seq}")
        return 1

    if answers == 0:
        print("\nAVISO: nenhum 'ANSWER enviado' encontrado — log errado?")
        return 1

    print("\nOK: todos os ANSWERs têm outcome logado.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
