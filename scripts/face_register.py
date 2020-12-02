#!/usr/bin/env python
# -*- coding: utf-8 -*-


# ライブラリをインポート
import os
import shutil
import datetime
import rospy
import roslib.packages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import signal
import sys

# 自作のライブラリをインポート
from aws_face_collection import AWSFaceCollection


class FaceImageRegister(object):
    def __init__(self):
        """ 初期化処理 """
        # サブスクライバを定義
        # 全体画像をサブスクライブするとき、image_callback関数を呼び出す
        rospy.Subscriber("/face_detection/image", Image, self.image_callback)
        # 顔のみの画像をサブスクライブするとき、faceimage_callback関数を呼び出す
        rospy.Subscriber("/face_detection/face_image", Image, self.faceimage_callback)

        # 画像を保持する変数を定義
        self.image = None
        self.faceimage = None

        # 画像の保存先ディレクトリのパスを定義
        self.image_path = roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'
        # ディレクトリがない場合は新しく作成
        if not os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'):
            os.mkdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/')

        # 自作ライブラリの準備
        self.afc = AWSFaceCollection()

        # 2秒間処理を待つ
        rospy.sleep(2.0)

        # Ctrl-Cが実行されたときの処理用
        signal.signal(signal.SIGINT, self.signal_handler)

    def image_callback(self, input_image):
        """ 画像のコールバック関数 """
        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        try:
            self.image = CvBridge().imgmsg_to_cv2(input_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def faceimage_callback(self, input_image):
        """ 顔画像のコールバック関数 """
        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        try:
            self.faceimage = CvBridge().imgmsg_to_cv2(input_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def signal_handler(self, signal, frame):
        """ Ctrl-Cが実行されたときの処理用の関数 """
        sys.exit(0)

    def local_id_list(self):
        """ ローカルに保存されているディレクトリ（ID）を一覧表示 """
        images_path = roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/'
        dirs = os.listdir(images_path)
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
        # メニューの分岐用変数
        mode = "0"
        while(1):
            # トップメニュー
            if mode != "1" and mode != "2":
                # IDのリストを表示
                print("-----------------------------------------------")
                self.local_id_list()
                print("-----------------------------------------------")
                print("[1] IDの登録")
                print("[2] IDの削除")
                print("-----------------------------------------------")
                mode = raw_input("キーを入力してください: ")
                print("-----------------------------------------------")
            # IDの登録
            if mode == "1":
                # IDのリストを表示
                self.local_id_list()
                print("-----------------------------------------------")
                collection_id = raw_input("登録するIDを入力してください: ")
                if collection_id == "":
                    print("IDが入力されていません。")
                    rospy.sleep(2.0)
                    # トップメニューに戻る
                    mode = "0"
                    continue
                if self.afc.register_collection_id(collection_id):
                    print("ID: 「{}」 の登録を行います。".format(self.afc.collection_id))
                    # 顔画像を保存するディレクトリを作成
                    if not os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + self.afc.collection_id):
                        os.mkdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + self.afc.collection_id)
                        print("顔画像を保存するディレクトリを作成しました。")
                    break
                else:
                    print("ID: 「{}」 は登録済みです。".format(collection_id))
                    rospy.sleep(2.0)
                    # トップメニューに戻る
                    mode = "0"
            # IDの削除
            elif mode == "2":
                # IDのリストを表示
                self.local_id_list()
                print("-----------------------------------------------")
                print("○ IDの削除後に顔画像が保存されたディレクトリを削除します。")
                target_collection_id = raw_input("削除するIDを入力してください: ")
                if self.afc.check_collection_id(target_collection_id):
                    if os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + target_collection_id):
                        self.afc.collection_id = target_collection_id
                        self.afc.delete_collection_id()
                        print("ID: 「{}」 の登録情報を削除しました。".format(target_collection_id))
                        # 顔画像を保存するディレクトリを削除
                        shutil.rmtree(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + target_collection_id)
                        rospy.sleep(2.0)
                    else:
                        print("このコンピュータに保存されているIDを入力してください。")
                        rospy.sleep(2.0)
                    # トップメニューに戻る
                    mode = "0"
                else:
                    print("ID: 「{}」はAWS上に登録されていません。".format(target_collection_id))
                    # AWS側にIDがない状態でこのコンピュータ内にディレクトリがある場合は削除する
                    if os.path.isdir(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + target_collection_id):
                        shutil.rmtree(roslib.packages.get_pkg_dir('myface_recognition') + '/scripts/images/' + target_collection_id)
                    rospy.sleep(2.0)
                    # トップメニューに戻る
                    mode = "0"
            else:
                # トップメニューに戻る
                mode = "0"

        # しばらく待つ
        rospy.sleep(2.0)
        print("-----------------------------------------------")
        print("顔画像をAWSに登録するノード")
        print("[s]キーを押すことでAWSに顔画像を登録できます。")
        print("キーを押すときは、カメラ画像が写っているウィンドウを選択した状態で行ってください。")
        print("-----------------------------------------------")

        # 一定周期でループを実行するための変数を定義
        rate = rospy.Rate(10)
        # 画像の連番用のカウンタ

        # ループを実行
        while not rospy.is_shutdown():
            # 全体画像が保存されているか確認
            if not self.image is None:
                # 画像を表示
                cv2.imshow("FaceImageRegister", self.image)
                # キーボードが押されるまで処理を待つ
                # 1ms待機して入力がなければ次の処理へ進む
                k = cv2.waitKey(1)
                # ESCキーを押したら終了
                if k == 27:
                    break
                # sキーを押したら画像を保存
                elif k == 115:
                    # 画像を指定したディレクトリに保存
                    if not self.faceimage is None:
                        cv2.imwrite(self.image_path + self.afc.collection_id + "/" + str(self.afc.face_counter) + ".jpg", self.faceimage)
                        print("画像を保存しました。")

                        # AWSに顔画像を登録する
                        if self.afc.register_face_image(self.image_path + self.afc.collection_id + "/" + str(self.afc.face_counter) + ".jpg"):
                            print("ID: 「{}」 に顔画像を登録しました。（合計{}枚）".format(self.afc.collection_id, self.afc.face_counter))
                        else:
                            print("登録する顔が見つかりませんでした。")
                            os.remove(self.image_path + self.afc.collection_id + "/" + str(self.afc.face_counter) + ".jpg")
                            print("画像を削除しました。")
                    else:
                        print("顔が認識されていません。")
                    print("---------------------------------------------")
        # 一定時間処理を待つ
        rate.sleep()


if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node("face_register")
    # クラスのインスタンスを作成し、メイン関数を実行
    FaceImageRegister().main()
