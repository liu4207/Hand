BVH（Biovision Hierarchy）
Hierarchy（骨架层级）：
 定义一棵骨骼树：根节点（Hips/Root）→ 躯干 → 手臂/手指…
 每个骨骼都有 offset（相对父骨的长度/方向）和通道（哪些自由度）
Motion（动作数据）：
 一帧一行（或等价结构），每帧给出每个骨骼的 旋转（通常是欧拉角），根节点还可能包含 平移。
 还有帧率/帧时间（Frame Time）。
✅ 用途
把 mocap 动作导入 Unity/Unreal/Blender/MuJoCo/机器人 retargeting 管线
GMR/IK 通常会从 BVH 里读到：根位姿 + 各关节旋转，然后映射到你的机器人关节
Attention@！
旋转是 欧拉角，而且有固定顺序（比如 ZXY/XYZ），顺序错了会“扭麻花”
单位/坐标系（Y-up / Z-up）和左右手系经常要转换
只有骨骼旋转，没有力/接触信息

ROS_DOMAIN_ID=42 可以隔离ros工作空间环境 不同的环境互不兼容
一定要source .bashrc
root@6d6319e8713f:~/intelligent_motion/ros2_mocap_ws# python3 mujoco_hand_visualizer.py --robot xhand_left --topic /retargeted_joint_state
root@6d6319e8713f:~# ros2 run motion_retarget_pkg motion_retarget_hand --robot xhand_left --publish_topic_name /retargeted_joint_state

root@6d6319e8713f:~/intelligent_motion/ros2_mocap_ws# ros2 bag play /root/rosbag2_2025_12_29-10_04_05 --loop 
git管理代码仓库在codeup retarget分支
root@6d6319e8713f:~/intelligent_motion/ros2_mocap_ws# git remote -v
codeup  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_retargeting.git (fetch)
codeup  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_retargeting.git (push)
origin  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_motion.git (fetch)
origin  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_motion.git (push)
git add .
git commit -m "lsk add"
git管理代码仓库在codeup retarget分支
root@6d6319e8713f:~/intelligent_motion/ros2_mocap_ws# git remote -v
codeup  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_retargeting.git (fetch)
codeup  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_retargeting.git (push)
origin  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_motion.git (fetch)
origin  git@codeup.aliyun.com:68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_motion.git (push)
git add .
git commit -m "lsk add"
git push codeup HEAD:xhand
git push codeup HEAD:xhand

常用指令

source /opt/ros/humble/setup.bash
source /workspace/kai_bot_ws/ros2_ws/install/setup.bash
ros2 topic hz /manus_glove_r
flatpak run io.github.input_leap.input-leap
xhost +local:
xhost +SI:localuser:root
export DISPLAY=:1(宿主机当前的 X server（或 Xwayland）很可能运行在 :1，所以 socket 是 X1。
 容器里却写死成 :0，就连不上。)

Mujoco
urdf--xml 
urdf更考虑结构  xml更关注内在的物理关系

/root/.mujoco/mujoco210/bin/simulate /workspace/model.xml 
~/.mujoco/mujoco210/bin/simulate /workspace/kai_bot_ws/robot_description/leaphand/robot.xml (old version)
mujoco.viewer()

Git stash


cd /root/intelligent_motion/ros2_mocap_ws
骨架+手+轨迹 
MUJOCO_GL=egl python3 mujoco_dual_visualizer.py   --robot xhand_left   --glove_topic /manus_glove_l   --motion_topic /kaibot/mocap_retarget    --post_rot_y_deg -90   --show_world --show_body_frames --show_joints

MUJOCO_GL=egl python3 mujoco_dual_visualizer.py   --robot xhand_left   --model /workspace/kai_bot_ws/ros2_ws/src/robot_description/xhand_left/mjmodel.xml   --glove_topic /manus_glove_l   --motion_topic /kaibot/mocap_retarget  --my_zero_body left_hand_root   --align_mode translation   --post_rot_y_deg -90   --show_world   --show_body_frames   --show_joints

MUJOCO_GL=egl python3 mujoco_dual_visualizer.py   --robot xhand_left   --glove_topic /manus_glove_l   --motion_topic /retargeted_joint_state   --goal_topic /retarget_goal_tips   --my_zero_body left_hand_root   --align_mode translation   --post_rot_y_deg -90   --show_world --show_body_frames --show_joints

骨架+手部模型
MUJOCO_GL=egl python3 mujoco_dual_visualizer.py \
  --robot xhand_left \
  --model /root/intelligent_motion/ros2_mocap_ws/config/general_motion_retargeting/assets/xhand_left/mjmodel.xml \
  --glove_topic /manus_glove_l \
  --motion_topic /retargeted_joint_state \
  --my_zero_body left_hand_root \
  --align_mode translation \
  --post_rot_y_deg -90 \
  --raw_print_hz 10 \
  --watch_body left_hand_root \
  --debug_print_hz 1 \
  --show_body_frames --show_joints --show_world
手部模型
python3 mujoco_hand_visualizer.py   --robot xhand_left   --topic /retargeted_joint_state
骨架
MUJOCO_GL=egl python3 mujoco_glove_skeleton.py \
  --robot xhand_left \
  --glove_topic /manus_glove_l \
  --my_zero_body left_hand_root \
  --ref_node 0 \
  --align_mode translation \
  --post_rot_y_deg -90 \
  --print_hz 10 \
  --pub_raw_tip
rosbag
ros2 bag play rosbag2_2025_12_29-10_04_05 --loop
ros2 topic
ros2 topic echo /retarget_joint_state
rosrun
ros2 run motion_retarget_pkg motion_retarget_hand --robot xhand_left --publish_topic_name /retargeted_joint_state
ros2 run manus_ros2 manus_data_publisher
ros2 run manus_client manus_right
ros2 run plotjuggler plotjuggler


roslaunch
ros2 launch motion_retarget motion_retarget_hand.launch.py hand:=revo2
ros2 launch intelligent_simulation simulate_hand.launch.py  rname:=revo2

ros2 run motion_retarget motion_retarget_hand --ros-args -p hand:=leaphand
ros2 run motion_retarget motion_retarget_hand --ros-args -p hand:=xhand_left
ros2 run motion_retarget motion_retarget_hand --ros-args -p hand:=revo2
source /workspace/kai_bot_ws/ros2_ws/install/setup.bash && ros2 run motion_retarget motion_retarget_hand --ros-args -p hand:=leaphand

ros2 launch intelligent_simulation simulate_hand.launch.py  rname:=revo2

编译
Source .bashrc
source /opt/kbot_package_x86_64/kbot/cerebellum/setup.bash
Build
cd /root/intelligent_motion/ros2_mocap_ws
colcon build --packages-select motion_retarget_pkg
source install/setup.bash
触觉手套资料
https://vxqf27novia.feishu.cn/drive/folder/QxeCf2S5NlREKfd5LJIc6Iz9nFe
软件仅支持win8及以上系统 以及 MAC操作系统
manus手套
GMR 环境准备 这个步骤要求将两个 general_motion_retargeting 文件夹复制到你的 conda 环境的 site-packages 文件夹下，目的是确保 GMR 所需的配置文件在 Python 环境中可以正确访问。
retarget仓库链接
https://codeup.aliyun.com/68a82b32efefeec54ed5eb5f/kai-bot/aiu/intelligent_retargeting
阿里云账户

remmina远程服务器
使用方法：
1 终端输入remmina
2 选择 RDP 协议，输入 Ubuntu 主机 IP“192.168.20.222:3390”
3 liushenkai
4 Lsk@#@2654

RL
读仿真给的“观测”，输出 12 维动作（手的 12 个关节目标）；根据奖励更新网络参数。
Proximal Policy Optimization（近端策略优化）

修复 Vulkan —— 注册 NVIDIA ICD（核心问题） 实现geort容器内 可视化
你的容器里有 RTX 4060 Ti GPU，NVIDIA 驱动库也都挂载进来了（libGLX_nvidia.so、libnvidia-glcore.so 等），但是缺少关键的 NVIDIA Vulkan ICD 注册文件。/usr/share/vulkan/icd.d/ 目录下只有 Intel、AMD、llvmpipe 等的 ICD，没有 nvidia_icd.json。所以 Vulkan 加载器找不到 NVIDIA GPU，SAPIEN 创建 VulkanRenderer 时报：> ErrorExtensionNotPresent因为它走了软件渲染的 lvp（llvmpipe），不支持硬件加速。
你已经有 nvidia_icd.json（在 /workspace/isaacgym/docker/ 下），但 library_path 写的是 libGLX_nvidia.so.0，这其实不对——NVIDIA Vulkan 真正的 ICD 库应该是 libGLX_nvidia.so.0 或专门的 Vulkan 路径。不过在你的容器里这个文件实际可用。只需把它拷贝/链接到 Vulkan 能找到的地方：
cp /workspace/isaacgym/docker/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json
或者临时用环境变量指定：
export VK_ICD_FILENAMES=/workspace/isaacgym/docker/nvidia_icd.json
上面两个用哪个都行 已经执行了第一个 永久生效应该

export DISPLAY=:1
