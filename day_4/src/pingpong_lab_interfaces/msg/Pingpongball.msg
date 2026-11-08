# msg/Pingpongball.msg
std_msgs/Header header   # 송신 시각 (지연 계산용)
uint32 seq               # 일련번호 (손실 계산용)
uint32 size              # 의도한 공의 크기 (기록용)
uint8[] data             # 크기를 바꿀 부분
