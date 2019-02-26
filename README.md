# AJK　草刈り機のROSパッケージ  

## 必要環境  
Robot Operating Systemのインストール


## 必要な機器  
twelite（無線モジュール）  
Ublox C94-M8P（RTK用のGNSS受信機）
Ublox C94-M8P or NEO-M8P（Moving-base用のGNSS受信機）
Bosch BNO055（IMU）


## インストール  
このレポジトリはROSのワークスペースとなっている。  
ROSのテキストを参考にして、catkin_makeをすること。  


## 草刈り機への通信  
tweliteを接続し下記のコマンドを実行する。  
roslaunch sanyokiki ajk.launch  
  
ターミナル内で" o "を入力し、エンターキーを押すとエンジン始動。  
" p "を入力するとエンジン停止。  
  
w,a,s,dなどのキー入力により草刈り機を操作できます。
w:前進、a:左旋回、s:後進、d:右旋回


## RTK-GNSS、Moving-baseとIMUの利用  
RTKのRoverとMoving-baseのRoverをPCに接続し、下記のコマンドを実行する。  
roslaunch ubx_analyzer imu_moving_base.launch
  
/gnss_odom　(UTM座標とMoving_baseの絶対方位)、/relposned（UBX-NAV-RELPOSNED）、  
/navpvt（UBX-NAV-PVT）  
等のメッセージが配信されます。
  
  
## MAVLinkの利用   
下記のコマンドを実行する。  
rosrun mavlink_ajk mavlink_ajk_node  
  
このノードを立ち上げた事により
QGCなどのGCSソフトでUDP接続すれば、ウェイポイント機能を利用できる。
  
  
## 自律走行ノードの起動
下記のコマンドを実行する。  
rosrun look_ahead qgc_look_ahead.py  
  
  
## 自律走行
以上全てを実行したうえで、QGCよりウェイポイントを送信。
MISSION STARTを実行すれば、自律走行が開始する。



