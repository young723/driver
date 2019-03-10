LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= src/main.c

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include 

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libutils \
	libnetutils

LOCAL_CFLAGS := -W -Wall

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:= qmp6988d

include $(BUILD_EXECUTABLE)

