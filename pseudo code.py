# --- helper ---
function quat_from_rpy(roll, pitch, yaw):
    compute quaternion from roll/pitch/yaw
    return (qx, qy, qz, qw)

# --- main class ---
class PickPlaceMin(Node):

    function __init__():
        # parameters
        declare group_name, ee_link, base_frame
        declare ik_service, traj_action, gripper_action
        declare joint_names
        declare pre_offset, lift_height, grip_open, grip_close, move_time

        # state
        self.joint_state = None
        self.pick_pose   = None
        self.place_pose  = None

        # comm
        subscribe('/joint_states', _js_cb)
        create service client(GetPositionIK, ik_service)
        create action client(FollowJointTrajectory, traj_action)
        create action client(GripperCommand, gripper_action)
        create TF2 Buffer/Listener
        subscribe('/pick_pose', _on_pick)
        subscribe('/place_pose', _on_place)

        log("Ready...")

    # --- callbacks ---
    function _js_cb(msg: JointState):
        self.joint_state = msg

    function _on_pick(msg: PoseStamped):
        self.pick_pose = msg
        self._try_run()

    function _on_place(msg: PoseStamped):
        self.place_pose = msg
        self._try_run()

    # --- main sequence ---
    function _try_run():
        if pick_pose or place_pose not ready: return

        base = param(base_frame)
        pick  = _to_base(pick_pose, base)
        place = _to_base(place_pose, base)

        pre     = param(pre_offset)
        lift_h  = param(lift_height)
        T       = param(move_time)
        open_v  = param(grip_open)
        close_v = param(grip_close)

        pick_above = _offset_z(pick, pre)
        pick_down  = _offset_z(pick, 0.0)
        pick_lift  = _offset_z(pick, lift_h)

        place_above = _offset_z(place, pre)
        place_down  = _offset_z(place, 0.0)
        place_up    = _offset_z(place, lift_h)


    # --- util ---
    function _to_base(pose, base_frame):
        if pose.frame == base_frame: return pose
        try TF2 transform(pose → base_frame)
        except: warn and return original

    function _offset_z(pose, dz):
        copy pose
        pose.position.z += dz
        return pose

    # --- IK + execution ---
    function _move_to_pose(target_pose, move_time):
        q = _compute_ik(target_pose)
        if q is None: return False
        return _exec_joint_positions(q, move_time)

    function _compute_ik(target_pose):
        if joint_state missing: return None
        if ik service not available: return None

        build IK request:
            group_name, ee_link, pose_stamped, timeout, seed = current joint_state
        call service
        if error_code != SUCCESS: return None
        map solution.joint_state → joint_names order
        return positions list

    function _exec_joint_positions(positions, move_time):
        if traj action server not available: return False
        build JointTrajectory with positions and time_from_start
        send FollowJointTrajectory goal
        wait for result
        if status == SUCCEEDED (4) or (0): return True else False

    # --- gripper ---
    function _gripper(position, effort=40.0):
        if gripper server not available: return False
        build GripperCommand.Goal(position, effort)
        send goal, wait for result
        return success/fail
        

# --- main entry ---
function main():
    rclpy.init()
    node = PickPlaceMin()
    spin(node)
    shutdown()
