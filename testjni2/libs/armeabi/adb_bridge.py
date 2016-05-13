import os
import sys
from pyadb import ADB
def main():

	os.system('adb devices -l')
	os.system('adb remount')
	os.system('adb push /home/cuigaosheng/ruimei_work/testjni2/libs/armeabi/px4_uploader /system/bin')

	os.system('adb remount')
	os.system('adb push /home/cuigaosheng/APM/ardupilot/ArduCopter/ArduCopter-v2.px4 /system/bin')
	
	adb = ADB()
	if adb.set_adb_path('/usr/bin/adb') is True:
		print 'Version: %s' % adb.get_version()
	else:
		print 'Check ADB binary path'
	print 'Waiting for device...'

	adb.wait_for_device()
	
	err, dev = adb.get_devices()
	print 'err = %s\n' % err
	print 'dev = %s\n' % dev

	print '3%s\n' % adb.get_output()
	if len(dev) == 0:
		print 'Unexpected error'
		return
	print 'Selecting: %s' % dev[0]

	adb.set_target_device(dev[0])
	


	print 'Executing command'
	adb.shell_command('adb shell')
	print '5%s\n' % adb.get_output()

	adb.shell_command('cd /system/bin')

	adb.shell_command('./system/bin/px4_uploader --path ./system/bin/ArduCopter-v2.px4')
	print '7%s\n' % adb.get_output()

	adb.shell_command('rm ./system/bin/ArduCopter-v2.px4')	

	adb.shell_command('rm ./system/bin/px4_uploader')	

if __name__ == '__main__':
	main()
