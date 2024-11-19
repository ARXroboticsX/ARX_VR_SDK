# ARX VR Control
# 方舟无限 虚拟现实设备 遥操作

- [中文](#中文)
- [English](#english)
- [常见问题FAQ](#常见问题)
- [FAQ](#FAQ)

---

# 中文
## 简介
虚拟现实设备Meta Quest3（以下简称VR），VR代替传统示教器操纵机械臂，有助于大规模大批量采集机械臂遥操作数据。
## 产品图片
~~产品开箱图片~~
### 主要操作端产品图片
<div style="text-align: center;">
    <img src="img/暂时的产品图片 操作端.jpg" alt="alt text" style="max-width: 100%; height: auto;">
    <p style="margin: 0;">（从左至右 手柄右 虚拟现实设备 手柄左）</p>
</div>

## 前置准备
### 所需设备一览
<table style="border-collapse: collapse; width: 100%; border: 1px solid black;">
    <thead>
        <tr>
            <th style="text-align: center; border: 1px solid white;">名称</th>
            <th style="text-align: center; border: 1px solid white;">描述</th>
            <th style="text-align: center; border: 1px solid white;">数量</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td style="text-align: center; border: 1px solid white;">电脑</td>
            <td style="text-align: center; border: 1px solid white;">安装 Ubuntu 20.04 系统并安装好 ROS</td>
            <td style="text-align: center; border: 1px solid white;">1</td>
        </tr>
        <tr>
            <td style="text-align: center; border: 1px solid white;">路由器</td>
            <td style="text-align: center; border: 1px solid white;">推荐支持 WiFi 5 及以上的路由器</td>
            <td style="text-align: center; border: 1px solid white;">1</td>
        </tr>
        <tr>
            <td style="text-align: center; border: 1px solid white;">机械臂</td>
            <td style="text-align: center; border: 1px solid white;">暂时支持 ARX X5、ARX L5、ARX L5 Pro、ARX Lift、ARX X7</td>
            <td style="text-align: center; border: 1px solid white;">N</td>
        </tr>
    </tbody>
</table>

### Wifi设置
以XiaoMi AX3000 路由器为例  
将电脑和VR连接到路由器上  
打开浏览器，地址栏输入  
```
http://192.168.31.1/cgi-bin/luci/web 进入小米路由器后台  
```
![alt text](img/路由器登录.png)
输入密码后依次点击->常用设置->局域网设置->下拉到最下方，找到DHCP静态IP分配  
![alt text](img/路由器IP设置.jpg)
点击添加，将你的电脑固定IP：192.168.31.137，MAC地址不变，点击应用  
![alt text](img/路由器更改IP地址.png)  
重启电脑，在terminal中输入
``` terminal
ifconfig
```
如果有如下显示，表示设置成功
``` terminal
enp7s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.31.137  netmask 255.255.255.0  broadcast 192.168.31.255
        inet6 fe80::beec:a0ff:fe4c:4ed6  prefixlen 64  scopeid 0x20<link>
        ether bc:ec:a0:4c:4e:d6  txqueuelen 1000  (以太网)
        RX packets 21631  bytes 27924572 (27.9 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 7991  bytes 1142345 (1.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```
### Meta Quest3 基础操作

### wifi延迟测试
固定机械臂，电脑和路由器，将VR放置到操作位置  
1.打开电脑和路由器  
~~2.打开VR，wifi设置，我的网络，IP地址，记下IP地址~~  
3.打开terminal 
``` terminal
#测试电脑到路由器延迟
ping 192.168.31.137
#测试电脑到VR的延迟(废弃)
ping <your_VR_IP>
```
时间以平时小于70ms，波动小于120ms为最佳
（由网络延迟导致的机械臂卡顿不在本产品考虑范围内，请注意电磁环境）
#### 如果追求极致低延迟可以考虑有线连接
插上ARX提供的TypeC网口拓展坞和网线，关闭Wifi，即可完成有线连接
### 开始操作机械臂
在ubuntu上克隆VR SDK和机械臂对应SDK
``` terminal
#VR SDK
git clone https://e.gitee.com/arx_enterprise/code/repos
cd vrsdk
catkin_make
./VR

#ARM SDK(以L5 Pro举例)
git clone https://e.gitee.com/arx_enterprise/code/repos
cd vrsdk
catkin_make
./VR
``` 

调整坐姿，确保您可以看到机械臂操纵位置，佩戴VR，拿起手柄（开启应用后请勿随意变换方向）  
打开VR菜单栏，选择右下角 X5 MR，手柄食指扳机键按下打开应用  
应用开始后会开启透视，画面会呈现周围姿态（透视模式）  
语音提示会重复提醒 锁定，锁定，锁定，锁定  

<table style="width:100%; border-collapse: collapse; text-align: left;">
    <tr>
        <td colspan="3" style="border: 1px solid black;">开机启动操作</td>
    </tr>
    <tr>
        <td colspan="3" style="border: 1px solid black;">1.同时按下 AB(右手)XY（左手）双手持握解锁 （锁定语音消失，表示手持部分已解锁）</td>
    </tr>
    <tr>
        <td colspan="3" style="border: 1px solid black;">2.同时长按 左右摇杆键（直至语音提示控制器上线）机械臂完整解锁</td>
    </tr>
    <tr>
        <th style="border: 1px solid black;">操作方式</th>
        <th style="border: 1px solid black;">按键映射</th>
        <th style="border: 1px solid black;">行为</th>
    </tr>
    <tr>
        <td style="border: 1px solid black;">按下右手</td>
        <td style="border: 1px solid black;">食指扳机</td>
        <td style="border: 1px solid black;">右侧机械臂1比1空间移动</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">按下左手</td>
        <td style="border: 1px solid black;">食指扳机</td>
        <td style="border: 1px solid black;">左侧机械臂1比1空间移动</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">松开右手</td>
        <td style="border: 1px solid black;">食指扳机</td>
        <td style="border: 1px solid black;">右侧机械臂保持空间位姿</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">松开左手</td>
        <td style="border: 1px solid black;">食指扳机</td>
        <td style="border: 1px solid black;">左侧机械臂保持空间位姿</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">按下右手</td>
        <td style="border: 1px solid black;">侧方扳机</td>
        <td style="border: 1px solid black;">右侧机械臂夹爪线性闭合</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">按下左手</td>
        <td style="border: 1px solid black;">侧方扳机</td>
        <td style="border: 1px solid black;">左侧机械臂夹爪线性闭合</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">长按右手</td>
        <td style="border: 1px solid black;">A按键</td>
        <td style="border: 1px solid black;">右侧机械臂归0</td>
    </tr>
    <tr>
        <td style="border: 1px solid black;">长按左手</td>
        <td style="border: 1px solid black;">X按键</td>
        <td style="border: 1px solid black;">左侧机械臂归0</td>
    </tr>
</table>


<table style="width: 100%; text-align: center; border: 1px solid black; border-collapse: collapse;">
  <tr>
    <th colspan="3" style="font-weight: bold; border: 1px solid black;">Rift空间抓取额外按键</th>
  </tr>
  <tr>
    <th style="border: 1px solid black;">操作方式</th>
    <th style="border: 1px solid black;">按键映射</th>
    <th style="border: 1px solid black;">行为</th>
  </tr>
  <tr>
    <td style="border: 1px solid black;">左摇杆</td>
    <td style="border: 1px solid black;">前进</td>
    <td style="border: 1px solid black;">底盘向前走</td>
  </tr>
  <tr>
    <td style="border: 1px solid black;">左摇杆</td>
    <td style="border: 1px solid black;">横移</td>
    <td style="border: 1px solid black;">NULL</td>
  </tr>
  <tr>
    <td style="border: 1px solid black;">右摇杆</td>
    <td style="border: 1px solid black;">前进</td>
    <td style="border: 1px solid black;">工作平台上移</td>
  </tr>
  <tr>
    <td style="border: 1px solid black;">右摇杆</td>
    <td style="border: 1px solid black;">横移</td>
    <td style="border: 1px solid black;">底盘转向</td>
  </tr>
</table>

# English
## Introduction
The Meta Quest3 Virtual Reality (VR) device (hereafter referred to as VR) replaces traditional teaching pendants for controlling robotic arms, facilitating large-scale, high-volume data collection for robotic arm teleoperation.

## Product Images
~~Product Unboxing Images~~  
### Main Operation End Product Images
<div style="text-align: center;">
    <img src="img/暂时的产品图片 操作端.jpg" alt="alt text" style="max-width: 100%; height: auto;">
    <p style="margin: 0;">(From left to right: Right Handle, VR Device, Left Handle)</p>
</div>

## Preparation
### Required Equipment List
<table style="border-collapse: collapse; width: 100%; border: 1px solid black;">
    <thead>
        <tr>
            <th style="text-align: center; border: 1px solid white;">Name</th>
            <th style="text-align: center; border: 1px solid white;">Description</th>
            <th style="text-align: center; border: 1px solid white;">Quantity</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td style="text-align: center; border: 1px solid white;">Computer</td>
            <td style="text-align: center; border: 1px solid white;">Installed with Ubuntu 20.04 system and ROS</td>
            <td style="text-align: center; border: 1px solid white;">1</td>
        </tr>
        <tr>
            <td style="text-align: center; border: 1px solid white;">Router</td>
            <td style="text-align: center; border: 1px solid white;">Recommended: router supporting WiFi 5 or above</td>
            <td style="text-align: center; border: 1px solid white;">1</td>
        </tr>
        <tr>
            <td style="text-align: center; border: 1px solid white;">Robotic Arm</td>
            <td style="text-align: center; border: 1px solid white;">Currently supports ARX X5, ARX L5, ARX L5 Pro, ARX Lift, ARX X7</td>
            <td style="text-align: center; border: 1px solid white;">N</td>
        </tr>
    </tbody>
</table>

### Wi-Fi Setup
Using the XiaoMi AX3000 router as an example  
- Connect both the computer and VR to the router  
- Open your browser and enter in the URL bar: http://192.168.31.1/cgi-bin/luci/web
- Enter the Xiaomi router Management Center
  ![alt text](img/路由器登录.png)  
- After entering the password, click -> Common Settings -> LAN Settings -> Scroll to the bottom, find DHCP Static IP Assignment  
 ![alt text](img/路由器IP设置.jpg)  
- Click Add, fix your computer's IP to 192.168.31.137, leave the MAC address unchanged,   and click Apply  
 ![alt text](img/路由器更改IP地址.png)  
- Restart your computer , open terminal, enter:
 ``` terminal
 ifconfig
 ```
- If you see the following output, the setup was successful:
```
  enp7s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.31.137  netmask 255.255.255.0  broadcast 192.168.31.255
        inet6 fe80::beec:a0ff:fe4c:4ed6  prefixlen 64  scopeid 0x20<link>
        ether bc:ec:a0:4c:4e:d6  txqueuelen 1000  (Ethernet)
        RX packets 21631  bytes 27924572 (27.9 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 7991  bytes 1142345 (1.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```
### Meta Quest3 Basic Operations

### Wi-Fi Latency Test
Fixed the robotic arm, computer, router, and place the VR in the operating position
1.Turn on the computer and connect router  
~~2. Turn on VR, Wi-Fi settings, my network, IP address, and record the IP address~~  
3.Open Ubuntu terminal 
``` terminal
#Test latency between computer and router
ping 192.168.31.137
#Test latency between computer and VR (deprecated)
ping <your_VR_IP>
```
The ideal latency is less than 70ms, with fluctuations smaller than 120ms.  
(Note: Robotic Arm stuttering caused by network latency is not within the scope of this product's consideration. Please pay attention to the surrounding electromagnetic environment.)
#### If you want the ultimate latency, you can consider a wired connection
Plug in the TypeC networkPort expansion Dock and network cable provided by ARX, turn off VR Wifi, finish wired connection
### Start Operating Robotic Arm
Clone the VR SDK and corresponding robotic arm SDK on Ubuntu:
``` terminal
#VR SDK
(deprecated) git clone https://e.gitee.com/arx_enterprise/code/repos 
cd vrsdk
catkin_make
./ARX_VR.sh

#ARM SDK(L5 Pro)
(deprecated) git clone https://e.gitee.com/arx_enterprise/code/repos 
cd vrsdk
catkin_make
./ARX_ARM.sh 
(The specific commands command depends on the Robotic Arm you use.)
``` 
- Adjust your posture to ensure you can see the robotic arm's manipulation position. Put on the VR headset, hold the controllers (do not change direction after starting the application)
- Open the VR menu, select the bottom right X5 MR, press the index trigger on the controller to open the application
- After starting, the application will switch to pass-through mode, displaying the surrounding environment (pass-through mode)
The voice prompt will repeatedly remind you(Chinese): "Locked, locked, locked, locked"
<table style="width:100%; border-collapse: collapse; text-align: left;"> <tr> <td colspan="3" style="border: 1px solid black;">Startup Procedure</td> </tr> <tr> <td colspan="3" style="border: 1px solid black;">1. Simultaneously press AB (right hand) and XY (left hand) to unlock (the locking voice will disappear, indicating that the handheld part is unlocked)</td> </tr> <tr> <td colspan="3" style="border: 1px solid black;">2. Simultaneously long press the left and right joysticks (until the voice prompt says the controller is onlin(控制器上线)) to fully unlock the robotic arm</td> </tr> <tr> <th style="border: 1px solid black;">Operation Method</th> <th style="border: 1px solid black;">Button Mapping</th> <th style="border: 1px solid black;">Action</th> </tr> <tr> <td style="border: 1px solid black;">Press the right hand</td> <td style="border: 1px solid black;">Index trigger</td> <td style="border: 1px solid black;">Right arm moves in a 1:1 space</td> </tr> <tr> <td style="border: 1px solid black;">Press the left hand</td> <td style="border: 1px solid black;">Index trigger</td> <td style="border: 1px solid black;">Left arm moves in a 1:1 space</td> </tr> <tr> <td style="border: 1px solid black;">Release the right hand</td> <td style="border: 1px solid black;">Index trigger</td> <td style="border: 1px solid black;">Right arm holds spatial position</td> </tr> <tr> <td style="border: 1px solid black;">Release the left hand</td> <td style="border: 1px solid black;">Index trigger</td> <td style="border: 1px solid black;">Left arm holds spatial position</td> </tr> <tr> <td style="border: 1px solid black;">Press the right hand</td> <td style="border: 1px solid black;">Side trigger</td> <td style="border: 1px solid black;">Right arm gripper closes linearly</td> </tr> <tr> <td style="border: 1px solid black;">Press the left hand</td> <td style="border: 1px solid black;">Side trigger</td> <td style="border: 1px solid black;">Left arm gripper closes linearly</td> </tr> <tr> <td style="border: 1px solid black;">Long press right hand</td> <td style="border: 1px solid black;">A button</td> <td style="border: 1px solid black;">Right arm returns to zero</td> </tr> <tr> <td style="border: 1px solid black;">Long press left hand</td> <td style="border: 1px solid black;">X button</td> <td style="border: 1px solid black;">Left arm returns to zero</td> </tr> </table> <table style="width: 100%; text-align: center; border: 1px solid black; border-collapse: collapse;"> <tr> <th colspan="3" style="font-weight: bold; border: 1px solid black;">Rift Space Grabbing Additional Buttons</th> </tr> <tr> <th style="border: 1px solid black;">Operation Method</th> <th style="border: 1px solid black;">Button Mapping</th> <th style="border: 1px solid black;">Action</th> </tr> <tr> <td style="border: 1px solid black;">Left joystick</td> <td style="border: 1px solid black;">Move forward</td> <td style="border: 1px solid black;">Chassis moves forward</td> </tr> <tr> <td style="border: 1px solid black;">Left joystick</td> <td style="border: 1px solid black;">Move sideways</td> <td style="border: 1px solid black;">NULL</td> </tr> <tr> <td style="border: 1px solid black;">Right joystick</td> <td style="border: 1px solid black;">Move forward</td> <td style="border: 1px solid black;">Work platform moves up</td> </tr> <tr> <td style="border: 1px solid black;">Right joystick</td> <td style="border: 1px solid black;">Move sideways</td> <td style="border: 1px solid black;">Chassis turns</td> </tr> </table> ```

# 常见问题
- __Question :__(终端)找不到python3或者python  
  __Answer :__ 运行
    ```
    # Terminal
    sudo ln -s /usr/bin/python3 /usr/bin/python
    ```
- __Question :__ 在VR操作过程中机械臂突然卡住一小会
  __Answer :__ 检查网络,考虑WiFi周围的电磁干扰,考虑[有线连接](#如果追求极致低延迟可以考虑有线连接).
# FAQ
- __Question :__(Terminal)NO Python3 or NO Python  
    __Answer :__ RUN
    ```
    # Terminal
    sudo ln -s /usr/bin/python3 /usr/bin/python
    ```
- __Question :__ The RoboticArm suddenly stuck for a while during VR operation.
  __Answer :__ Check Internet. Please pay attention to the surrounding electromagnetic environment. Consider about [wired connection](#if-you-want-the-ultimate-latency-you-can-consider-a-wired-connection).


