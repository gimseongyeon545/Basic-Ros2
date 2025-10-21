# 1. pub/sub/timer/qos/param
```
[heartbeat_pub] --( /mini/heartbeat : std_msgs/String )--> [status_sub]
        │                                                   ▲
        └─ uses Timer(hz) & Parameters(text,hz) ────────────┘

[qos_matrix_demo] <─┐ 실험용: 퍼블/섭 양쪽 QoS 바꿔가며 손실/지연 관찰
                    └─( /mini/heartbeat )<─[heartbeat_pub]

[ros2 param]* ──(set/get)──> [heartbeat_pub]   *CLI로 hz,text 동적 변경
```

</br>

# 2. service, action (request, response, cancel, feedback)
```
[adder_client] --call--> (AddTwoInts.srv) --serve--> [adder_service]
      ▲                                         │
      └────────────── result <──────────────────┘

[gripper_action_client] ==goal==> (GripperCommand.action server)
       ▲                 <== feedback/result ==
       └==================== cancel =====================

[traj_action_client] ==goal(JointTrajectory)==> (FollowJointTrajectory server)
        ▲                     <== feedback/result ==
        └========================= cancel =========================
```

</br>

# 3. TF2 & pose 변환
```
(static TF)         (dynamic TF, 학습용)
[static_frames] ──> base_link ──> tool_frame
[vr_dummy_broadcaster] ──> steamvr_world ──> controller

[tf_echo_logger] --lookup(base→tool, world→controller)--> 로그

[pose_relay_transform]
   subscribes:  (/in_pose : PoseStamped [frame=A])
   uses:        TF2 Buffer/Listener
   publishes:   (/out_pose : PoseStamped [frame=B])
   실패시: 재시도/타임아웃 로그
```
