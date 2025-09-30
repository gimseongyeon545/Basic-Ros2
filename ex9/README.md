# planning_scene_box.py
1. `from geometry_msgs.msg import PoseStamped`
2. `from moveit_msgs.msg import PlanningScene, CollisionObject`
   - `self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)`
3. `self.timer = self.create_timer(0.5, self.tick)`
   - https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.create_timer
     > `create_timer(timer_period_sec, callback, callback_group=None, clock=None)`
   - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1855
     > `timer = Timer(
            callback, callback_group, timer_period_nsec, clock, context=self.context,
            autostart=autostart)`
     - Timer 객체 생성
     - 생성된 Timer 객체 `self._timers.append(timer)` 로 `self._timers` 에 저장
       - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L1860
     - `self.tick` callback 호출 되는 과정
        ```
        [1] `spin()`
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L353
        [2] `spin_once()`
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L941
        [3] `_spin_once_impl`
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L919
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L925
        [4] `wait_for_ready_callbacks`
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L892
        [5] `_wait_for_ready_callbacks`
          - node.timers 수집: `node.timers` 로 Node 클래스의 timers property에 접근
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L697
            - timers property: https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/node.py#L301
          - `_make_handler` 실행 및 호출할 함수는 `self._take_timer` (`handler = self._make_handler(tmr, node, self._take_timer)`)
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L832
              > `for tmr in node.timers:`
        [6] `_make_handler` & handler
          - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L575
          - `_spin_once_impl` 로 다시 와서 handler() func 실행
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L934
          - 받는 인자가 entity, node, take_from_wait_list
            > `def _make_handler(
                  self,
                  entity: 'EntityT',
                  node: 'Node',
                  take_from_wait_list: Callable[['EntityT'],
                                                Optional[Callable[[], Coroutine[None, None, None]]]],
                  ) -> Task[None]:`
          -  위 `handler = self._make_handler(tmr, node, self._take_timer)`에서 호출한대로 인자 받고 `call_coroutine = take_from_wait_list(entity)`
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L603
            - `take_from_wait_list(entity)` 는 `self._take_timer(tmr)` 을 실행
          - `_take_timer`
            - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L431
            - callback 함수 실행: `await await_or_execute(partial(tmr.callback, **prefilled_arg))`
              - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L459
              - `async def await_or_execute`
                - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/executors.py#L130
       ```
4. `from shape_msgs.msg import SolidPrimitive`
5. `from builtin_interfaces.msg import Time` 
