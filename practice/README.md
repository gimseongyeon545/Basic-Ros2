```
[heartbeat_pub] ──(String /mini/heartbeat)──▶ [status_sub]


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


[counter_with_reset] ──(Int32 /mini/count)──▶ (소비자/RViz/echo 등)
     ▲
     └── Empty /mini/reset ⇐ ros2 service call (외부 도구 또는 별도 클라)


[latched_config_pub] ──(String /mini/config [TL])──▶ [config_listener]


[tf_pose_relay] : (PoseStamped /mini/pose_in) ──TF 변환(target_frame)──▶ (PoseStamped /mini/pose_out)
                     └── TF Buffer/Listener ── 조회 ──▶ (TF tree: ex. base_link, camera_link ...)


[joint_state_dummy_pub] ──(sensor_msgs/JointState /joint_states)──▶ (RViz2/상태 뷰어)


[fibonacci_action_client] ⇄ (action /mini/fibonacci) ⇄ [fibonacci_action_server]
         (order, 취소옵션)                           (피드백/결과)

```

</br>

[1] 1.py, 2.py
- 도식
  ```
  [heartbeat_pub] ──(String /mini/heartbeat)──▶ [status_sub]
  ```
- 1.py: heartbeat_pub
- 2.py: status_sub
