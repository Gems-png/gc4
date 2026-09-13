校赛要运行的版本
就是运行serial_tag, 识别tag然后发给下危机就好

初赛版本，也会在这里更新，因为已经添加了git仓库
数据链一tag：
相机2识别tag，然后发给/cv/tag_recognize_topic，识别后就终止
serial_comm 订阅话题，然后根据协议发给下危机

数据链二：
complete_flow节点订阅tag_recognize_topic(其他测试节点接受传入参数)
根据状态调用grab_x 的 service

调用circle的服务，传入颜色代码，得到circle的位置
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
然后completeflow发布关节角度话题和gripper话题

数据链三，手眼标定：
serial得到电机坐标发布在real_joint_states
然后myfk正运动学得到末端位姿
HECal会识别出标定版的位姿，然后订阅末端位姿
进行手眼标定，然后保存





