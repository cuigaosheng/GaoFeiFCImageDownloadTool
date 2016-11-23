
#第一步：确认飞控设备已经连接到android主板
adb devices -l

#第二步：通过adb传输px4_uploader到Android板
adb remount
adb push ../../../px4_uploader /system/bin


#第三步：通过adb传输ardupilot镜像文件到Android板
adb remount
adb push ArduCopter-v2_50hz.px4 /system/bin

#第四步：通过adb进入android shell终端，执行程序
adb shell 
#cd /system/bin
#./px4_uploader --path ./system/bin/ArduCopter-v2.px4



