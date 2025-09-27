# service_client_ik.py
1. `from moveit_msgs.srv import GetPositionIK`
2. `self.cli = self.create_client(GetPositionIK, '/compute_ik')`
3. `while not self.cli.wait_for_service(timeout_sec=1.0):`
4. `req = GetPositionIK.Request()`
   - `req.ik_request.group_name = 'arm'`             # 환경에 맞게
   - `req.ik_request.ik_link_name = 'end_effector_link'`
   - `req.ik_request.attempts = 5`
   - `req.ik_request.timeout.sec = 2`
