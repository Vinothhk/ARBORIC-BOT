# Arboric bot - 🌳 🤖

A bot navigates around employing BehaviorTree &amp; ComputerVision

## Clone 
Clone the repository
```
mkdir -p arboricbot && cd arboricbot
```
```
git clone https://github.com/Vinothhk/ARBORIC-BOT.git
```

## Dependencies
This project needs [onnxruntime_cpp](https://github.com/microsoft/onnxruntime) & obviously [BehaviorTreeCPP](https://github.com/BehaviorTree/BehaviorTree.CPP.git) as dependencies.

Please refer to [btree-installation](https://youtu.be/4PUiDmD5dkg?si=nzxqR6XrV4x_cPXq) for installing BehaviorTreeCPP.

To install onnxruntime_cpp
```
pip install onnxruntime
```
```
mkdir onnxruntime_cpp && cd onnxruntime_cpp
```
```
wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.0/onnxruntime-linux-x64-1.16.0.tgz
```
```
tar -xvzf onnxruntime-linux-x64-1.16.0.tgz
```
Note: Don't forget to modify the onnxruntimecpp path in the CMakeLists.txt


## Build

Next, Build the files


Open a directory to build the packages
```
mkdir build && cd build
```
```
cmake ..
```
Make them
```
make
```
