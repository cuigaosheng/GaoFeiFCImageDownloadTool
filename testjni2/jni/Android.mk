LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
TARGET_PLATFORM := android-3
LOCAL_MODULE    := base64
LOCAL_SRC_FILES := base64.cpp
LOCAL_LDLIBS    := -llog
include $(BUILD_STATIC_LIBRARY)



include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	px4_uploader_main.cpp \
	write_parameter.cpp \
	serial_port.cpp \
	autopilot_interface.cpp

//LOCAL_LDFLAGS := 
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../jsoncpp  
LOCAL_STATIC_LIBRARIES := \
	jsoncpp \
	base64
LOCAL_CPPFLAGS := -DJSON_IS_AMALGAMATION -fexceptions -fpermissive
LOCAL_MODULE := libpx4_uploader
LOCAL_LDLIBS := -L$(call host-path, $(LOCAL_PATH)/../../libs/armeabi)  -llog -lz 
#include $(BUILD_SHARED_LIBRARY)
include $(BUILD_EXECUTABLE)
include $(call all-makefiles-under,$(LOCAL_PATH))
