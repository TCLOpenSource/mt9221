LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    mma_api.c \
    mma_test.c

LOCAL_CFLAGS += -DGL_GLEXT_PROTOTYPES -DEGL_EGLEXT_PROTOTYPES -DMSOS_TYPE_LINUX

#LOCAL_CFLAGS += -Wall -Werror -Wunreachable-code


LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libutils \
    libutopia
    
LOCAL_C_INCLUDES := \
    $(TARGET_UTOPIA_LIBS_DIR)/include \

    
LOCAL_MODULE:= mma_test

LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_EXECUTABLE)
