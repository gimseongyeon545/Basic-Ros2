'''
/joy (sensor_msgs/Joy)
      │
      ▼
[joy_to_twist] ──(Twist /mini/cmd_vel_raw)──▶ [cmd_gain] ──(Twist /mini/cmd_vel)──▶ (리맵 옵션) /turtle1/cmd_vel
                               ▲
                               │ (Bool /mini/enabled [TL])
                    [enable_server] ─────────────────────────────────────────────▶
                               ▲                               ▲
                               │                               │
      [enable_client] ──SetBool /mini/enable ⇒─────────────────┘
        (주기적 토글 요청)

'''

