# MyFace Recognition
顔認証をして、モーターを動かすパッケージ

## Directory structure
* launch
    * face_register.launch
        * 顔画像とIDの登録または削除をするローンチファイル
    * face_search.launch
        * 顔認証をして、モーターを動かすローンチファイル
* scripts
    * example_image
        * example.jpg
            * 顔画像のサンプル
    * aws_face_collection.py
        * 顔画像とIDの登録や削除、確認をするライブラリ
    * aws_face_search.py
        * 登録した顔画像とカメラ画像を比較して、顔認証をするライブラリ
    * face_register.py
        * 顔画像とIDの登録や削除をするプログラム
    * face_search.py
        * 顔認証をして、モーターを動かすプログラム
* .gitignore
    * githubとか使うときに作るもの
* CMakeLists.txt
    * ROSパッケージの設定ファイル
* package.xml
    * ROSパッケージの設定ファイル
* LICENCE
    * ROSパッケージのライセンスファイル
* README.md
    * パッケージの説明書

## Requirements
* [uvc_camera](http://wiki.ros.org/uvc_camera)
* [opencv_apps](http://wiki.ros.org/opencv_apps)

## Installation
```sh
cd ~/catkin_ws
rosdep install -r -y -i --from-paths src
```

## Usage
* 顔画像とIDの登録や削除
```sh
roslaunch myface_recognition face_register.launch
```
* 顔認証をして、モーターを動かす
```sh
roslaunch myface_recognition face_search.launch
```