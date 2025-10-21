```
[heartbeat_pub] --( /mini/heartbeat : std_msgs/String )--> [status_sub]
        │                                                   ▲
        └─ uses Timer(hz) & Parameters(text,hz) ────────────┘

[ros2 param]* ──(set/get)──> [heartbeat_pub]   *CLI로 hz,text 동적 변경
```
