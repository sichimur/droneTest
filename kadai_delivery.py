# encoding: utf-8
import math
import time

from dronekit import LocationGlobalRelative, VehicleMode, connect

def delivery():
    if __name__ == '__main__':
        #Rover（市村）
        #機体接続
        rover1 = connect('tcp:127.0.0.1:5812', wait_ready=True, timeout=60)
        #GUIDEモードに切り替え
        rover1.mode = VehicleMode("GUIDED")
        toALocation = LocationGlobalRelative(35.879768, 140.348495, 0)
        fromAALocation = LocationGlobalRelative(35.876991, 140.348026, 0)
        #目的地まで移動
        rover1.simple_goto(toALocation, groundspeed=1000, airspeed=1000)
        #荷下ろし
        time.sleep(10)
        #元に場所に戻る
        rover1.simple_goto(fromAALocation, groundspeed=1000, airspeed=1000)
        rover1.close()

        #Boat（竹林）
        # ボートの接続
        boat = connect('tcp:127.0.0.1:5812', wait_ready=True, timeout=60)
        # モードはGUIDED
        boat.mode = VehicleMode("GUIDED")
        # 目標の緯度・経度、高度を設定する
        mainport_Location = LocationGlobalRelative(35.878275, 140.338069, 0)
        Waypoint1 = LocationGlobalRelative(35.8778464, 140.3384576, 0)
        Waypoint2 = LocationGlobalRelative(35.8805952, 140.3473408, 0)
        taiganport_Location = LocationGlobalRelative(35.879768, 140.348495, 0)
        # 初期位置へ移動
        vehicle.simple_goto(mainport_Location, groundspeed=1000, airspeed=1000)

        # 荷物の載せ替え待ち
        time.sleep(10)
        # 目的地へ移動
        vehicle.simple_goto(Waypoint1 , groundspeed=1000, airspeed=1000)
        vehicle.simple_goto(Waypoint2, groundspeed=1000, airspeed=1000)
        vehicle.simple_goto(taiganport_Location, groundspeed=1000, airspeed=1000)
        # 接続を閉じる
        # vehicle.close()

        #Copter（須藤）
        # 距離を計算する関数
        def get_distance_metres(aLocation1, aLocation2):
            dlat = aLocation2.lat - aLocation1.lat
            dlong = aLocation2.lon - aLocation1.lon
            return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

        # 目的地に到達したかどうかをチェックする関数
        def is_at_location(targetLocation, currentLocation, threshold=1.0):
            distance = get_distance_metres(targetLocation, currentLocation)
            return distance <= threshold

        # コプターの接続
        copter = connect('tcp:127.0.0.1:5772', wait_ready=True)

        def arm_and_takeoff_copter(target_altitude):
            while not copter.is_armable:
                print("初期化中です")
                time.sleep(1)
            
            copter.mode = VehicleMode("GUIDED")
            copter.armed = True

            while not copter.armed:
                print("アームを待ってます")
                time.sleep(1)
            
            copter.simple_takeoff(target_altitude)
            while True:
                if copter.location.global_relative_frame.alt >= target_altitude * 0.95:
                    print("目標高度に到達しました")
                    break
                time.sleep(1)

        # 離陸
        arm_and_takeoff_copter(10)  # 10メートルに離陸

        # メインポートから隣接ポートへ移動
        target_location = LocationGlobalRelative(35.867003, 140.305987, 10)
        copter.simple_goto(target_location)

        # 目的地に到達するまで待機
        while not is_at_location(target_location, copter.location.global_relative_frame):
            print(f"移動中: {copter.location.global_relative_frame}")
            time.sleep(1)

        # 荷物の載せ替え中
        time.sleep(10)

        # 隣接ポートからメインポートへ戻る
        target_location = LocationGlobalRelative(35.878275, 140.338069, 10)
        copter.simple_goto(target_location)

        # 目的地に到達するまで待機
        while not is_at_location(target_location, copter.location.global_relative_frame):
            print(f"移動中: {copter.location.global_relative_frame}")
            time.sleep(1)

        # 着陸
        copter.mode = VehicleMode("LAND")
        copter.close()
