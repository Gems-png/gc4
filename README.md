校赛要运行的版本
就是运行serial_tag, 识别tag然后发给下危机就好

初赛版本，也会在这里更新，因为已经添加了git仓库
数据链一tag：
相机2识别tag，然后发给/cv/tag_recognize_topic，识别后就终止
serial_comm 订阅话题，然后根据协议发给下危机

<!-- 数据链二：
complete_flow节点的部分流程test
订阅tag_recognize_topic(其他测试节点接受传入参数)
根据状态调用grab_1 的 service

调用material的服务，传入颜色代码，得到物料的位置
相对
位置传入my_ik,得到机械臂电机角度
serial_comm 发给下危机

grab根据订阅joint_states和real_joint_states的偏差判断到位没有，进行反馈
到位后，发送下降，闭合夹爪等任务
一样判断到位没有，然后返回

complete 收到grab反馈
调用put_x
如果是放置在圆环内，和grab类似的操作
如果是放置在托盘上
要根据记忆调用plate，转来空盘
然后completeflow发布关节角度话题和gripper话题 -->

数据链路2的2.0
grab1
要抓指定颜色的物料，当然也可以不指定yanse
先让机械臂移动到一个俯瞰的位置，大概0, 90, 90, 0（发给下危机的角度）
直接发布到机械臂关节节点，给串口发送，然后
暂时根据物料的圆形属性，调用maincam的circle服务
等待正确的物料颜色，如果里画面中心太远，就把机械臂水平平移过去
到差不多正中心，然后根据距离下降高度。这个圆是物料的底面圆，所以不会太高，只会底，这个没什么关系

数据链三，手眼标定：
serial得到电机坐标发布在real_joint_states
然后myfk正运动学得到末端位姿
HECal会识别出标定版的位姿，然后订阅末端位姿
进行手眼标定，然后保存

机械臂运动学的接口（myfk / myik）：

| 话题 | 类型 | 单位 | 谁发 |
| --- | --- | --- | --- |
| /goal_position | PointStamped | mm，base_link 下 | material / circle 服务 |
| /joint_states | JointState | rad，已减零位偏置 | myik |
| /real_joint_states | JointState | rad | serial_comm（下位机回传） |
| /fk_position | PointStamped | mm | myfk |
| /fk_pose | PoseStamped | mm | myfk（手眼标定的输入） |
| /fk_camera_pose | PoseStamped | mm，末端上方 5cm | myfk（暂定值，标定后改 camera_offset_mm） |
| /fk_arm_marker | MarkerArray | m | myfk（RViz 里画机械臂，单位米是 RViz 规定的） |
| /ik_target_marker | Marker | m | myik（绿=可达 红=不可达） |

看可视化：ros2 launch myfk arm_kinematics.launch.py
（没有下位机想干跑：加 joint_topic:=/joint_states；RViz 固定坐标系用 base_link）

长度单位统一毫米：杆长、/goal_position、位姿话题全是 mm，
只有 RViz 的 Marker 用米，换算只在 myfk 画图那几行里做。
myik 与 myfk 的 l1~l4、bias2、bias4 必须完全一致，否则 FK(IK(p)) != p。

正运动学（末端水平分支）：
    a1 = q2 + bias2,  a2 = q3,  a3 = q4 + bias4     （都是相对水平面的仰角）
    第3杆仰角 θ2 = a1 - a2,  第4杆仰角 θ3 = a1 - a2 - a3
    R = l2·cos(a1) + l3·cos(θ2) + l4·cos(θ3)
    Z = l1 + l2·sin(a1) + l3·sin(θ2) + l4·sin(θ3)
    X = cos(q1)·R,  Y = sin(q1)·R
myik 是它的反解：末端保持水平（θ3 = 0）时第 4 根杆整段落在径向，
先减掉 l4 变成两杆问题，解完再令 a3 = a1 - a2 把末端掰回水平。





