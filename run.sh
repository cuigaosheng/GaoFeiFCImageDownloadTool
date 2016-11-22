#!/bin/sh
#python px_uploader.py --port /dev/ttyACM0 px4fmu-v2_default.px4
if [ ! -e "jsoncpp-src-0.6.0-rc2.tar.gz" ]; #TODO check json exited
then
wget https://sourceforge.net/projects/jsoncpp/files/jsoncpp/0.6.0-rc2/jsoncpp-src-0.6.0-rc2.tar.gz
fi

if [ ! -e "scons-2.1.0.tar.gz" ]; #TODO check scons exited
then
wget http://sourceforge.net/projects/scons/files/scons/2.1.0/scons-2.1.0.tar.gz 
fi

sudo tar -zxvf jsoncpp-src-0.6.0-rc2.tar.gz -C /opt 
sudo tar -zxvf scons-2.1.0.tar.gz -C /opt
'''
sudo chown cuigaosheng.cuigaosheng /mnt/opt/scons-2.1.0
sudo chown cuigaosheng.cuigaosheng /mnt/opt/jsoncpp-src-0.6.0-rc2
'''
cd /opt/scons-2.1.0

sudo python setup.py install 

cd /opt/jsoncpp-src-0.6.0-rc2

sudo python /usr/bin/scons platform=linux-gcc #TODO DEBUG

sudo cp -R /opt/jsoncpp-src-0.6.0-rc2/include/json /usr/include

sudo cp /opt/jsoncpp-src-0.6.0-rc2/libs/linux-gcc-4.9.2/libjson_linux-gcc-4.9.2_libmt.a /usr/local/lib

sudo cp /opt/jsoncpp-src-0.6.0-rc2/libs/linux-gcc-4.9.2/libjson_linux-gcc-4.9.2_libmt.so /usr/local/lib

JSONLIB=/usr/local/lib
NEW_LINE="export LD_LIBRARY_PATH="$JSONLIB""
echo "$NEW_LINE" >> $HOME/.profile
sudo source $HOME/.profile

sudo  scons 
