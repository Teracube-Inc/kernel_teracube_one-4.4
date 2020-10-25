#
#design for fake dual camera
#

ifeq ($(strip $(VANZO_FEATURE_FAKE_DUAL_CAMERA_BY_NAME)),yes)

ifneq ($(strip $(CUSTOM_FAKE_CAMERA)),)
    custom_fake_camera := $(addsuffix /,$(shell echo $(CUSTOM_FAKE_CAMERA)))

    CUSTOM_FAKE_CAMERA_FLAGS := $(addprefix -D, $(shell echo $(CUSTOM_FAKE_CAMERA) | tr a-z A-Z))

    #$(info $(CUSTOM_FAKE_CAMERA_FLAGS))

    subdir-ccflags-y += $(CUSTOM_FAKE_CAMERA_FLAGS)

    obj-y += $(custom_fake_camera)
    obj-y += fake_camera.o
endif

endif
