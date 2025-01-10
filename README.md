# 4次ルンゲクッタの車両運動学の制御器シミュレータ

## 必要環境
 - Eigen3
 - Armadillo
 - Python3 (結果の可視化用に使用)

## 必要環境のセットアップ方法例(wsl環境版)
 - Eigen3
```bash
sudo apt install libeigen3-dev
```

 - Armadillo
    - cmake, LAPACK, BLASも一緒にインストールする前提で記載
    - ArmadilloのDL元URLは現時点最新のver.を記載
    - wget実行時のディレクトリは必要に応じて適宜変更すること
```bash
sudo apt-get install cmake
sudo apt-get install liblapack-dev
sudo apt-get install libblas-dev

wget https://sourceforge.net/projects/arma/files/armadillo-14.2.2.tar.xz

tar Jxfv armadillo-14.2.2.tar.xz

cd armadillo-14.2.2  

cmake .
make
sudo make install
 ```

 - Python3
    - Pythonインストール済みであれば実施不要
 ```bash
sudo apt install python3.11-dev
sudo apt install -y python3-matplotlib
sudo apt install -y python3-numpy
 ```

## 使用方法
 - buildしてbuild生成物を実行する
    - プロジェクトのルートフォルダで下記を実行する
```bash
mkdir build
cmake -S . -B build
cmake --build build

cd build
./SIM_RK4 
```
 - 画面表示に従って、コマンドライン上で制御モードと制御パラメータを設定する
 - シミュレーション結果を確認する

## 注意事項
 - 制御の基本構成要素の学習用教材として作成したため詳細動作検証はしていない
 - 制御器やダイナミクス周りは作成途中の部分がある(最低限の教材部分のみ仕上げた)
 - numpyのver.>=2.0だとmatplotlibcppがエラーするので、可視化関数はコメントアウトするかnumpyのver.を下げること
    - numpy 1.21.5で各種動確を実施
