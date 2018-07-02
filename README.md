# AJK　草刈り機のROSパッケージ  
  
## 必要な機器  
twelite  
Bosch BNO055(IMU)  
Ublox C94-M8P  
  
## 草刈り機への通信  
草刈り機への通信はtweliteを使う。  

下記のコマンドを実行  
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
下記のコマンドを実行  
rosrun ubx_analyzer navpvt.py  
  
UBX-NAV-PVTのプロトコルを読み込み、
/utm　(UTM座標)、/gnss (緯度経度)、/gpstime (GPSタイム) の
メッセージを配信するプログラムです。
  
