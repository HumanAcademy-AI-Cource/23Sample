#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ライブラリをインポート
import rospy
import roslib.packages
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import signal
import sys
import time
import random
import boto3
from botocore.exceptions import ClientError
import json

# 自作のライブラリをインポート
from aws_face_collection import AWSFaceCollection
from aws_face_search import AWSFaceSearch


class DetectSpecifiedFace():
    def __init__(self):
        """ 初期化処理 """
        # サブスクライバーを定義
        rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # 画像を保持する変数
        self.image = None

        # 画像の保存先
        self.image_path = roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'
        # ディレクトリがない場合は新しく作成
        if not os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'):
            os.mkdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/')

        # 自作ライブラリの準備
        self.afc = AWSFaceCollection()
        self.afs = AWSFaceSearch()

        # Ctrl-Cが実行されたときの処理用
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_callback(self, data):
        """ 画像のコールバック関数 """
        try:
            self.image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_save(self):
        """ 画像を保存する関数 """
        cv2.imwrite(self.image_path + "camera.jpg", self.image)
        print("カメラ画像を保存しました。")

    def search_collection(self, collection_id):
        """ 顔認証を行う関数 """
        # 顔認証を実行
        result_data = self.afs.search_collection(collection_id, self.image_path + "camera.jpg")
        # 顔認証が正常に完了した場合
        if result_data != None:
            # マッチした顔が見つかった場合
            if len(result_data["FaceMatches"]) > 0:
                print("-------------------------------")
                print("登録されている顔を検出しました。")
                print("-------------------------------")
                print("信頼度: {}％".format(result_data["FaceMatches"][0]["Face"]["Confidence"]))
                print("類似度: {}％".format(result_data["FaceMatches"][0]["Similarity"]))
                # ロボットに速度指令を送る
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.2
                self.cmd_pub.publish(cmd_msg)
                rospy.sleep(1.0)
                # ロボットを止める
                cmd_msg.linear.x = 0.0
                self.cmd_pub.publish(cmd_msg)
            else:
                print("-------------------------------")
                print("登録されていない顔を検出しました。")
            print("-------------------------------")

    def signal_handler(self, signal, frame):
        """ Ctrl-Cが実行されたときの処理用の関数 """
        sys.exit(0)

    def local_id_list(self):
        """ ローカルに保存されているディレクトリ（ID）を一覧表示 """
        images_path = roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'
        dirs = os.listdir(images_path)
        print("-----------------------------------------------")
        print("このコンピュータ内に保存されているID")
        dir_counter = 0
        for d in dirs:
            if os.path.isdir(os.path.join(images_path, d)):
                print("○ {}".format(d))
                dir_counter += 1
        if dir_counter == 0:
            print("-> 保存されていません。")

    def main(self):
        """ メインで実行する関数 """
        while(1):
            # IDのリストを表示
            self.local_id_list()
            print("-----------------------------------------------")
            collection_id = raw_input("登録した自身のIDを入力してください: ")
            # ローカルにディレクトリ（ID）存在するか確認
            if not os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + collection_id):
                print("-----------------------------------------------")
                print("このコンピュータに保存されているIDを入力してください。")
                rospy.sleep(2.0)
                continue
            # IDが存在するか確認
            if self.afc.check_collection_id(collection_id):
                # 登録されていた場合は、インスタンスに設定を反映
                self.afc.collection_id = collection_id
                break
            else:
                print("ID: 「{}」 はAWS上に登録されていません。".format(collection_id))
                rospy.sleep(2.0)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.image is not None:
                raw_input("Enterキーを押すと顔認識を開始します:")
                # 画像を保存
                self.image_save()
                # 顔認証処理を開始
                self.search_collection(self.afc.collection_id)
            else:
                print("画像が未取得です。")
            rate.sleep()


if __name__ == '__main__':
    # ノードを初期化
    rospy.init_node('face_search')
    # クラスのインスタンスを作成し、メイン関数を実行
    DetectSpecifiedFace().main()
