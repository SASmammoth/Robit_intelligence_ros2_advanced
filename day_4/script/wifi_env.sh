#!/usr/bin/env bash
# wifi_env.sh — 실험 환경(기기 · 와이파이 링크 · ROS/커널 설정)을 markdown 표로 기록
#
# 사용:
#   ./wifi_env.sh                                   # 이 기기만
#   ./wifi_env.sh -p nano1@nano1                    # 이 기기 + Nano (ssh로 같이 수집)
#   ./wifi_env.sh -p nano1@nano1 -n "거리 3m, 장애물 없음" -o env.md
# 옵션: -p 상대 기기(ssh)  -i 무선 인터페이스(기본 자동)  -n 비고(거리·장애물)  -o 저장 파일
# 스캔(채널 사용률·겹치는 AP·AP 모델)은 sudo가 필요 → 실행 전에 `sudo -v` 해 두면 채워진다.
# Nano 쪽 스캔까지 채우려면 Nano에서 iw를 비밀번호 없이 sudo로 돌릴 수 있어야 한다 (없으면 '?' 표시).

set -u
PEER=""; IFACE=""; NOTE=""; OUT=""
while getopts "p:i:n:o:h" o; do
  case $o in
    p) PEER=$OPTARG ;; i) IFACE=$OPTARG ;; n) NOTE=$OPTARG ;; o) OUT=$OPTARG ;;
    *) sed -n '2,11p' "$0"; exit 0 ;;
  esac
done
OUT=${OUT:-wifi_env_$(date +%y%m%d_%H%M).md}

# ───────────── 한 기기에서 값 모으기 (key=value 줄로 출력) ─────────────
# ssh로 상대 기기에 그대로 보내서 실행하므로 이 함수 안에서 모든 걸 정의한다.
collect() {
  kv()   { printf '%s=%s\n' "$1" "$2"; }
  rate() { awk '{r=$1" Mbps"; for(i=2;i<=NF;i++){ if($i~/MHz$/) r=r", "$i; if($i~/MCS$/) r=r", "$i" "$(i+1); if($i~/NSS$/) r=r", NSS "$(i+1)} print r}'; }
  chan() { local f=$1; if ((f<3000)); then echo $(( (f-2407)/5 )); elif ((f<5925)); then echo $(( (f-5000)/5 )); else echo $(( (f-5950)/5 )); fi; }

  local IF=${IFACE:-$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')}

  # ── 기기
  local model
  if [ -r /proc/device-tree/model ]; then model=$(tr -d '\0' </proc/device-tree/model)
  else model=$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null); fi
  kv host "$(hostname)"
  kv model "${model:-?}"
  kv os "$(. /etc/os-release; echo "$PRETTY_NAME")"
  kv kernel "$(uname -r)"

  # ── ROS / 커널 설정 (원격은 ~/.bashrc 기준)
  local rosenv k
  if [ "${REMOTE:-0}" = 1 ]; then rosenv=$(bash -ic env </dev/null 2>/dev/null); else rosenv=$(env); fi
  for k in ROS_DISTRO RMW_IMPLEMENTATION RMW_FASTRTPS_PUBLICATION_MODE ROS_DOMAIN_ID \
           ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE; do
    kv "$k" "$(grep -m1 "^$k=" <<<"$rosenv" | cut -d= -f2-)"
  done
  kv rmem_max     "$(cat /proc/sys/net/core/rmem_max)"
  kv rmem_default "$(cat /proc/sys/net/core/rmem_default)"
  kv ipfrag_high  "$(cat /proc/sys/net/ipv4/ipfrag_high_thresh)"

  # ── Jetson CPU 모드
  if command -v nvpmodel >/dev/null 2>&1; then
    kv nvpmodel "$( { nvpmodel -q 2>/dev/null || sudo -n nvpmodel -q 2>/dev/null; } | awk -F': ' '/Power Mode/{print $2; exit}')"
    local mn mx
    mn=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null)
    mx=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null)
    [ -n "$mn" ] && kv clocks "$([ "$mn" = "$mx" ] && echo '고정 (jetson_clocks 켬)' || echo '가변 (jetson_clocks 끔)')"
  fi

  [ -z "$IF" ] && { kv iface "무선 없음"; return; }
  kv iface "$IF"
  kv ip  "$(ip -4 -o addr show "$IF" | awk '{print $4; exit}')"
  kv mtu "$(cat /sys/class/net/$IF/mtu)"
  kv power_save "$(iw dev "$IF" get power_save 2>/dev/null | awk '{print $NF}')"

  # ── 랜카드
  local dev drv nic a
  dev=$(readlink -f /sys/class/net/$IF/device)
  drv=$(basename "$(readlink -f "$dev/driver")")
  if [[ $dev == */usb* ]]; then
    nic="USB $(lsusb -d "$(cat "$dev/../idVendor"):$(cat "$dev/../idProduct")" 2>/dev/null | cut -d' ' -f7-)"
  else
    a=$(basename "$dev")
    nic=$(lspci -vs "$a" 2>/dev/null | awk -F': ' '/Subsystem/{print $2; exit}')
    [ -z "$nic" ] && nic=$(lspci -s "$a" 2>/dev/null | cut -d: -f3- | sed 's/^ //')
  fi
  kv nic "${nic:-?}"; kv driver "$drv"

  # ── 상대 기기까지 경로 + ping (ping 덕분에 rx bitrate가 비콘이 아닌 데이터 기준이 됨)
  if [ -n "${TARGET:-}" ]; then
    local rdev p
    rdev=$(ip route get "$TARGET" 2>/dev/null | grep -o 'dev [^ ]*' | cut -d' ' -f2)
    kv route "$rdev $([ -d "/sys/class/net/$rdev/wireless" ] && echo '(무선)' || echo '(유선)')"
    p=$(ping -c 10 -i 0.2 -s 1000 -W 1 "$TARGET" 2>/dev/null)
    kv ping_loss "$(grep -o '[0-9.]*% packet loss' <<<"$p" | cut -d% -f1)"
    kv ping_rtt  "$(awk -F'= ' '/rtt|round-trip/{print $2}' <<<"$p" | cut -d' ' -f1)"
  fi

  # ── 링크
  local link info chl bssid freq width center
  link=$(iw dev "$IF" link)
  bssid=$(awk '/Connected to/{print $3}' <<<"$link")
  kv bssid  "$bssid"
  kv ssid   "$(awk -F': ' '/SSID/{print $2}' <<<"$link")"
  kv signal "$(awk '/signal/{print int($2)}' <<<"$link")"
  kv tx     "$(awk -F': ' '/tx bitrate/{print $2}' <<<"$link" | rate)"
  kv rx     "$(awk -F': ' '/rx bitrate/{print $2}' <<<"$link" | rate)"
  kv dtim   "$(awk '/dtim period/{print $3}' <<<"$link")"
  kv beacon "$(awk '/beacon int/{print $3}' <<<"$link")"
  info=$(iw dev "$IF" info)
  chl=$(grep -m1 'channel' <<<"$info")          # channel 100 (5500 MHz), width: 80 MHz, center1: 5530 MHz
  freq=$(sed -nE 's/.*\(([0-9]+) MHz.*/\1/p' <<<"$chl")
  width=$(sed -nE 's/.*width: ([0-9]+) MHz.*/\1/p' <<<"$chl")
  center=$(sed -nE 's/.*center1: ([0-9]+) MHz.*/\1/p' <<<"$chl")
  kv channel "$(awk '{print $2}' <<<"$chl")"
  kv freq "$freq"; kv width "$width"; kv center "$center"
  kv txpower "$(awk '/txpower/{print $2}' <<<"$info")"

  # ── 스캔: BSS Load, AP 모델(WPS), 같은 채널 범위의 다른 AP
  local scan="" try bss u st m mo mn apm o ov="" lo hi b f s n x
  for try in 1 2 3; do
    scan=$(sudo -n iw dev "$IF" scan 2>/dev/null) && [ -n "$scan" ] && break
    sleep 2
  done
  [ -z "$scan" ] && { kv scan "실패"; return; }
  bss=$(awk '
    function out(){ if(b!="") print b"|"f"|"s"|"n"|"u"|"st"|"a"|"m"|"mo"|"mn }
    /^BSS /                { out(); b=substr($2,1,17); f=s=n=u=st=m=mo=mn=""; a=($0~/associated/)?1:0 }
    /^\tfreq:/             { f=int($2) }
    /^\tsignal:/           { s=int($2) }
    /^\tSSID:/             { n=substr($0, index($0,":")+2) }
    /channel utilisation/  { split($NF,x,"/"); u=x[1] }
    /station count/        { st=$NF }
    /\* Manufacturer:/     { m=substr($0, index($0,":")+2) }
    /\* Model:/            { mo=substr($0, index($0,":")+2) }
    /\* Model Number:/     { mn=substr($0, index($0,":")+2) }
    END { out() }' <<<"$scan")

  IFS='|' read -r x x x x u st x m mo mn <<<"$(awk -F'|' '$7==1' <<<"$bss")"
  kv util "$u"; kv stations "$st"
  apm=$(echo "$m $mo $mn" | xargs)
  if [ -z "$apm" ] && [ -r /usr/share/ieee-data/oui.txt ]; then
    o=$(echo "${bssid:0:8}" | tr 'a-f:' 'A-F-')
    apm="$(grep -m1 "^$o" /usr/share/ieee-data/oui.txt | awk -F'\t' '{print $NF}') (제조사만)"
  fi
  kv ap_model "${apm:-?}"

  if [ -n "$center" ] && [ -n "$width" ]; then
    lo=$((center - width/2)); hi=$((center + width/2))
    while IFS='|' read -r b f s n x x a x; do
      [ -z "$f" ] || [ "$a" = 1 ] && continue
      if (( f + 10 > lo && f - 10 < hi )); then          # 상대 AP의 기본 20MHz가 내 범위와 겹치면
        n=$(printf '%b' "$n"); ov+="${n:-(숨김)} (ch$(chan "$f"), ${s}dBm), "
      fi
    done <<<"$bss"
  fi
  kv overlap "${ov%, }"
}

# ───────────── 수집 ─────────────
declare -A L R
TARGET=""
[ -n "$PEER" ] && TARGET=$(getent ahostsv4 "${PEER#*@}" | awk 'NR==1{print $1}')

echo "[1/2] $(hostname) 수집 중..." >&2
while IFS='=' read -r k v; do L[$k]=$v; done < <(TARGET=$TARGET collect)

if [ -n "$PEER" ]; then
  echo "[2/2] $PEER 수집 중 (ssh)..." >&2
  myip=${L[ip]:-}; myip=${myip%/*}
  while IFS='=' read -r k v; do R[$k]=$v; done < <(
    { echo "REMOTE=1; TARGET='$myip'; IFACE=''"; declare -f collect; echo collect; } \
      | ssh -o ConnectTimeout=5 "$PEER" bash -s)
  if [ ${#R[@]} -eq 0 ]; then echo "  ssh 수집 실패 → 이 기기만 기록" >&2; PEER=""; fi
fi

# ───────────── 표 출력 ─────────────
g() { local -n A=$1; local v=${A[$2]:-}; echo "${v:-?}"; }
row() { if [ -n "$PEER" ]; then printf '| %s | %s | %s |\n' "$1" "$2" "$3"; else printf '| %s | %s |\n' "$1" "$2"; fi; }
both() { row "$1" "$($2 L)" "$($2 R)"; }
head2() { row "항목" "$(g L host)" "$(g R host)"; [ -n "$PEER" ] && echo "|---|---|---|" || echo "|---|---|"; }

f_dev()   { echo "$(g $1 model) / $(g $1 os) / $(g $1 kernel)"; }
f_nic()   { echo "$(g $1 nic) / $(g $1 driver)"; }
f_if()    { echo "$(g $1 iface) / $(g $1 ip) / MTU $(g $1 mtu)"; }
f_route() { local -n A=$1; echo "${A[route]:-?}"; }
f_ps()    { g $1 power_save; }
f_cpu()   { local -n A=$1; if [ -n "${A[nvpmodel]+x}" ]; then echo "${A[nvpmodel]:-?} / ${A[clocks]:-?}"; else echo "-"; fi; }
f_ap()    { echo "$(g $1 bssid) / $(g $1 ap_model)"; }
f_band()  {
  local -n A=$1; local f=${A[freq]:-0} c=${A[channel]:-?} bd dfs=""
  if ((f==0)); then echo "?"; return; fi
  if ((f<3000)); then bd=2.4GHz; elif ((f<5925)); then bd=5GHz; ((c>=52 && c<=144)) && dfs=" (DFS)"; else bd=6GHz; fi
  echo "${A[ssid]:-?} / $bd / $c$dfs / ${A[width]:-?}MHz"
}
f_sig()   {
  local s; s=$(g $1 signal); [[ $s =~ ^-?[0-9]+$ ]] || { echo "?"; return; }
  if ((s>=-50)); then echo "$s dBm (아주 좋음)"; elif ((s>=-67)); then echo "$s dBm (좋음)"
  elif ((s>=-75)); then echo "$s dBm (불안정)"; else echo "$s dBm (나쁨)"; fi
}
f_tx()    { g $1 tx; }
f_rx()    { g $1 rx; }
f_dtim()  { echo "$(g $1 dtim) / $(g $1 beacon) TU"; }
f_txp()   { echo "$(g $1 txpower) dBm"; }
f_util()  {
  local u st j=""; u=$(g $1 util); st=$(g $1 stations)
  if [[ $u =~ ^[0-9]+$ ]]; then local p=$((u*100/255))
    if ((p>50)); then j=혼잡; elif ((p>30)); then j=보통; else j=여유; fi
    echo "$p% ($u/255, $j) / ${st}대"
  else echo "? (스캔 실패: sudo -v 후 다시)"; fi
}
f_ov()    { local -n A=$1; if [ "${A[scan]:-}" = 실패 ]; then echo "?"; else echo "${A[overlap]:-없음}"; fi; }
f_ping()  { local -n A=$1; [ -n "${A[ping_rtt]+x}" ] || { echo "-"; return; }
            echo "손실 ${A[ping_loss]:-?}% / RTT ${A[ping_rtt]:-?} ms (min/avg/max/mdev)"; }
f_rosv()  { local -n A=$1; echo "${A[ROS_DISTRO]:-?} / ${A[RMW_IMPLEMENTATION]:-기본(rmw_fastrtps_cpp)}"; }
f_pub()   { local -n A=$1; echo "${A[RMW_FASTRTPS_PUBLICATION_MODE]:-기본(SYNCHRONOUS)}"; }
f_dom()   { local -n A=$1; echo "${A[ROS_DOMAIN_ID]:-기본(0)}"; }
f_disc()  { local -n A=$1; echo "${A[ROS_AUTOMATIC_DISCOVERY_RANGE]:-기본(SUBNET)} / ${A[ROS_STATIC_PEERS]:-없음(멀티캐스트)}"; }
f_xml()   { local -n A=$1; echo "${A[FASTRTPS_DEFAULT_PROFILES_FILE]:-없음}"; }
kb()      { [[ $1 =~ ^[0-9]+$ ]] && echo "$(( $1/1024 ))KB" || echo "?"; }
f_buf()   { echo "$(kb "$(g $1 rmem_max)") / $(kb "$(g $1 rmem_default)")"; }
f_frag()  { kb "$(g $1 ipfrag_high)"; }

{
  echo "# 측정 환경 — $(date '+%Y-%m-%d %H:%M')"
  echo
  echo "## 1. 기기"
  echo
  head2
  both "기기 / OS / 커널"            f_dev
  both "무선랜 칩 / 드라이버"        f_nic
  both "인터페이스 / IP / MTU"       f_if
  both "상대 기기까지 경로"          f_route
  both "절전 (power_save)"           f_ps
  both "CPU 모드 (nvpmodel / 클럭)"  f_cpu
  echo
  echo "## 2. 와이파이 링크"
  echo
  head2
  both "AP (BSSID / 모델)"                f_ap
  both "SSID / 대역 / 채널 / 폭"          f_band
  both "신호 세기"                        f_sig
  both "tx bitrate"                       f_tx
  both "rx bitrate"                       f_rx
  both "DTIM / 비콘 주기"                 f_dtim
  both "송신 출력"                        f_txp
  both "채널 사용률 / 접속 기기 수"       f_util
  both "같은 채널 범위의 다른 AP"         f_ov
  both "ping (1000B × 10, 상대 기기로)"   f_ping
  row  "기기 간 거리 / 장애물" "${NOTE:-?}" "${NOTE:-?}"
  row  "측정 시각" "$(date '+%Y-%m-%d %H:%M:%S')" "〃"
  echo
  echo "## 3. 네트워크 · ROS 설정"
  echo
  head2
  both "ROS 배포판 / RMW"                       f_rosv
  both "발행 모드 (FASTRTPS_PUBLICATION_MODE)"  f_pub
  both "ROS_DOMAIN_ID"                          f_dom
  both "Discovery 범위 / STATIC_PEERS"          f_disc
  both "Fast DDS XML 프로필"                    f_xml
  both "UDP 수신 버퍼 rmem_max / default"       f_buf
  both "IP 조각 버퍼 ipfrag_high_thresh"        f_frag
} | tee "$OUT"

echo >&2; echo "저장: $OUT" >&2
