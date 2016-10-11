TARGETNAME = 'px4_uploader' 

FILELIST = Glob('*.cpp')
HEADFILEPATH = Split('/usr/include/') 

LIB = Split('json_linux-gcc-5.4.0_libmt z')
LIB_DIR = Split('/usr/local/lib')

#CPPFLAGS = Split('-std=c++11 -g') 
CPPFLAGS = Split('-g') 
Program(target = TARGETNAME, source = FILELIST, LIBS = LIB , CPPPATH = HEADFILEPATH, LIBPATH = LIB_DIR,CCFLAGS = CPPFLAGS)
