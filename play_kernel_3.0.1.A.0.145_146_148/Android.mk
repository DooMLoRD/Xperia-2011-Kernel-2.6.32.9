#
# Use this kernel source as base for kernel if PREBUILT is not choosen
#
ifeq ($(TARGET_PREBUILT_KERNEL),)
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_OUT_CONFIG := $(KERNEL_OUT)/.config

TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage

KERNEL_SOURCE_PATH := $(call my-dir)
KERNEL_SOURCE_CONFIG := $(KERNEL_SOURCE_PATH)/arch/arm/configs/$(KERNEL_DEFCONFIG)

# Select choosen defconfig
install_defconfig_always:
$(KERNEL_OUT_CONFIG): install_defconfig_always
	@mkdir -p $(KERNEL_OUT)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=../$(KERNEL_OUT) \
            ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)

# Build kernel from selected defconfig
$(TARGET_PREBUILT_KERNEL): $(KERNEL_OUT_CONFIG)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=../$(KERNEL_OUT) \
                    ARCH=arm CROSS_COMPILE=arm-eabi-

# Install selected defconfig, run menuconfig and then copy back
export KCONFIG_NOTIMESTAMP=true
.PHONY: kernelconfig
kernelconfig: $(KERNEL_OUT_CONFIG)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=../$(KERNEL_OUT) \
	             ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	$(hide) cp $(KERNEL_OUT)/.config $(KERNEL_SOURCE_CONFIG)
endif

#
# Rules for packing kernel into elf and sin
#
$(PRODUCT_OUT)/kernel.elf: $(TARGET_PREBUILT_KERNEL) $(PRODUCT_OUT)/ramdisk.img | sin-tools
	$(hide) $(HOST_OUT_EXECUTABLES)/mkelf.py -o  $(dir $@)/kernel-part.img \
		$(TARGET_PREBUILT_KERNEL)@0x208000 $(PRODUCT_OUT)/ramdisk.img@0x1000000,ramdisk && \
		$(SEMCSC) -c $(PRODUCT_PARTITION_CONFIG) -p Kernel -t internal -i $(dir $@)/kernel-part.img -o $@


$(PRODUCT_OUT)/kernel.si_: $(PRODUCT_OUT)/kernel.elf | sin-tools
	$(hide) $(HOST_OUT_EXECUTABLES)/create_sin_header Kernel $(PRODUCT_PARTITION_CONFIG) $@
	$(hide) cat $< >> $@

$(PRODUCT_OUT)/kernel.sin: $(PRODUCT_OUT)/kernel.si_ $(PRODUCT_PARTITION_CONFIG) | sin-tools
	@echo target SIN: $(notdir $@)
	$(hide) $(SEMCSC) -c $(PRODUCT_PARTITION_CONFIG) -p Kernel -t external -i $< -o $@

#
# Add kernel to system wide PHONY target sin and s1images
#
.PHONY: sin
.PHONY: s1images

sin: $(PRODUCT_OUT)/kernel.sin
s1images: $(PRODUCT_OUT)/kernel.si_

#
# boot.img requires $(INSTALLED_KERNEL_TARGET) to exist
#
$(INSTALLED_KERNEL_TARGET): $(TARGET_PREBUILT_KERNEL) | $(ACP)
	$(transform-prebuilt-to-target)

