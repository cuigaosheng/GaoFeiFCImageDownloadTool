TARGETNAME = 'px4_uploader' 
LIB = Split('json_linux-gcc-5.4.0_libmt z')
FILELIST = Glob('*.cpp')
HEADFILEPATH = Split('/usr/include/') 
LIB_DIR = Split('/usr/local/lib')
CPPFLAGS = Split('-std=c++11 -g') 
Program(target = TARGETNAME, source = FILELIST, LIBS = LIB , CPPPATH = HEADFILEPATH, LIBPATH = LIB_DIR,CCFLAGS = CPPFLAGS)
