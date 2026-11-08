#!/bin/bash
# run_sizes.sh — 크기별로 ping을 돌리고 결과 줄만 모아 저장 (약 19분)
SIZES="64 1024 1300 1500 4096 8192 32768 60000 70000 262144 524288 1048576"
N=250          # 크기당 전송 개수 (10Hz → 25초)
HZ=10.0
REPEAT=3
OUT=results_$(date +%m%d_%H%M).txt

total=$(( $(echo $SIZES | wc -w) * REPEAT ))
i=0
start=$(date +%s)
echo "총 ${total}회, 예상 약 $(( total * 31 / 60 ))분" | tee -a "$OUT"

for r in $(seq 1 $REPEAT); do
  for s in $SIZES; do
    i=$((i + 1))
    echo "== [$i/$total] rep=$r size=$s $(date +%H:%M:%S)" | tee -a "$OUT"
    timeout 70 ros2 run my_ping my_ping_node --ros-args \
      -p N:=$N -p hz:=$HZ -p size:=$s 2>&1 | grep "size=" | tee -a "$OUT"
    sleep 2
  done
done

echo "완료: $(( ($(date +%s) - start) / 60 ))분 소요, 저장: $OUT" | tee -a "$OUT"
