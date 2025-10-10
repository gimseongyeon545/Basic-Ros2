[heartbeat_pub] --( /mini/heartbeat : std_msgs/String )--> [status_sub]
        │                                                   ▲
        └─ uses Timer(hz) & Parameters(text,hz) ────────────┘

[qos_matrix_demo] <─┐ 실험용: 퍼블/섭 양쪽 QoS 바꿔가며 손실/지연 관찰
                    └─( /mini/heartbeat )<─[heartbeat_pub]

[ros2 param]* ──(set/get)──> [heartbeat_pub]   *CLI로 hz,text 동적 변경
