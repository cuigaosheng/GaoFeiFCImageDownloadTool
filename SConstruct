TARGETNAME = 'px4_uploader' 

FILELIST = Glob('*.cpp')
HEADFILEPATH = Split('/usr/include/') 

LIB = Split('json_linux-gcc-4.9.2_libmt z pthread')
LIB_DIR = Split('/usr/local/lib')

CPPFLAGS = Split('-g') 
Program(target = TARGETNAME, source = FILELIST, LIBS = LIB , CPPPATH = HEADFILEPATH, LIBPATH = LIB_DIR,CCFLAGS = CPPFLAGS)
