```
[gripper_action_client] ==goal==> (GripperCommand.action server)
       ▲                 <== feedback/result ==
       └==================== cancel =====================

[traj_action_client] ==goal(JointTrajectory)==> (FollowJointTrajectory server)
        ▲                     <== feedback/result ==
        └========================= cancel =========================
```
