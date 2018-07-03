# AJK　草刈り機のROSパッケージ  
  
## 必要な機器  
twelite（無線モジュール）  
Bosch BNO055（IMU）  
Ublox C94-M8P（GNSSモジュール）  
  


## インストール  
このレポジトリはROSのワークスペースとなっている。  
ROSのテキストを参考にして、catkin_makeをすること。  
  


## 草刈り機への通信  
tweliteを接続し下記のコマンドを実行する。  
roslaunch sanyokiki ajk.launch  
  
ターミナル内で" o "を入力し、エンターキーを押すとエンジン始動。  
" p "を入力するとエンジン停止。  
  
このプログラムを起動すれば、cmd_velの配信により走行できます。  
linear.xは0.55、angular.zは0.45くらいの数値がちょうど良い感じです。  
  
## 草刈り機のマニュアル操作  
下記のコマンドを実行  
rosrun sanyokiki key.py  
  
w,a,s,dなどのキー入力により草刈り機を操作できます。  
  


## GNSSの利用  
C94-M8Pを接続し、下記のコマンドを実行する。  
rosrun ubx_analyzer navpvt.py  
  
navpvt.pyによってUBX-NAV-PVTのプロトコルが読みこまれ、
/utm　(UTM座標)、/gnss (緯度経度)、/gpstime (GPSタイム) の  
メッセージが配信されます。
  


## IMUの利用  
BNO055を接続し、下記のコマンドを実行する。  
roslaunch imu_jetson.launch  
  


## GNSSの絶対座標による機体角の取得（やや不安定）  
下記のコマンドを実行する。  
roslaunch gnss_yaw gnss.launch  
  
そして機体をまっすぐ前進させる。
上手くいけば、/gnss_yawというUTM座標の変化から求められた
機体角のメッセージが配信される（直進性が低いと配信されない）。

IMUとGNSSが正常であれば、  
UTM座標と機体角が統合された/gnss_imuメッセージが配信される。  
  


## 自律走行とウェイポイント  
上記の項目を全て実行した後、下記のコマンドを実行する。  
rosrun pid_navi route_write.py  
  
任意の場所に草刈り機を移動し、ターミナル内の指示に従って、  
ウェイポイントを登録する。  
route.csvが保存されたらスタート位置に移動し、下記のコマンドを実行する。  
rosrun pid_navi mower_rtrip.py  

自律走行が開始されるはずである。  

