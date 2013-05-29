#
# Copyright (C) 2011 The Android Open-Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

ifeq ($(TARGET_PREBUILT_KERNEL),)
LOCAL_KERNEL := device/ti/jacinto6evm/kernel
else
LOCAL_KERNEL := $(TARGET_PREBUILT_KERNEL)
endif

PRODUCT_COPY_FILES := \
	$(LOCAL_KERNEL):kernel \
	device/ti/jacinto6evm/tablet_core_hardware_jacinto6evm.xml:system/etc/permissions/tablet_core_hardware_jacinto6evm.xml \
	device/ti/jacinto6evm/init.jacinto6evmboard.rc:root/init.jacinto6evmboard.rc \
	device/ti/jacinto6evm/init.jacinto6evmboard.usb.rc:root/init.jacinto6evmboard.usb.rc \
	device/ti/jacinto6evm/ueventd.jacinto6evmboard.rc:root/ueventd.jacinto6evmboard.rc \
	device/ti/jacinto6evm/fstab.jacinto6evm:root/fstab.jacinto6evm \
	device/ti/jacinto6evm/media_profiles.xml:system/etc/media_profiles.xml \
	device/ti/jacinto6evm/media_codecs.xml:system/etc/media_codecs.xml \
	device/ti/jacinto6evm/audio/audio_policy.conf:system/etc/audio_policy.conf \
	frameworks/native/data/etc/android.hardware.wifi.xml:system/etc/permissions/android.hardware.wifi.xml \
	frameworks/native/data/etc/android.hardware.wifi.direct.xml:system/etc/permissions/android.hardware.wifi.direct.xml \
	frameworks/native/data/etc/android.hardware.usb.host.xml:system/etc/permissions/android.hardware.usb.host.xml \
	frameworks/native/data/etc/android.hardware.usb.accessory.xml:system/etc/permissions/android.hardware.usb.accessory.xml

# to mount the external storage (sdcard)
PRODUCT_COPY_FILES += \
	device/ti/jacinto6evm/vold.fstab:system/etc/vold.fstab

# These are the hardware-specific features
PRODUCT_COPY_FILES += \
	frameworks/native/data/etc/android.hardware.camera.xml:system/etc/permissions/android.hardware.camera.xml \
	frameworks/native/data/etc/android.hardware.camera.front.xml:system/etc/permissions/android.hardware.camera.front.xml \

PRODUCT_PACKAGES := \
	make_ext4fs \
	e2fsck \
	com.android.future.usb.accessory

PRODUCT_PROPERTY_OVERRIDES := \
	hwui.render_dirty_regions=false

PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
	persist.sys.usb.config=mtp

PRODUCT_PROPERTY_OVERRIDES += \
	ro.opengles.version=131072

PRODUCT_CHARACTERISTICS := tablet

DEVICE_PACKAGE_OVERLAYS := \
	device/ti/jacinto6evm/overlay

PRODUCT_TAGS += dalvik.gc.type-precise

PRODUCT_PACKAGES += \
	librs_jni \
	com.android.future.usb.accessory

# Filesystem management tools
PRODUCT_PACKAGES += \
	make_ext4fs

PRODUCT_PROPERTY_OVERRIDES += \
	ro.sf.lcd_density=160

# WI-Fi
PRODUCT_PACKAGES += \
	dhcpcd.conf \
	hostapd.conf \
	wifical.sh \
	TQS_D_1.7.ini \
	TQS_D_1.7_127x.ini \
	crda \
	regulatory.bin \
	wlconf

PRODUCT_PACKAGES += \
    CameraOMAP \
    Camera \
    camera_test

# Audio HAL module
PRODUCT_PACKAGES += audio.primary.jacinto6
PRODUCT_PACKAGES += audio.hdmi.jacinto6

# Audioout libs
PRODUCT_PACKAGES += libaudioutils

PRODUCT_PACKAGES += \
	tinymix \
	tinyplay \
	tinycap

PRODUCT_PACKAGES += \
	boardidentity \
	libboardidentity \
	libboard_idJNI \
	Board_id

# BlueZ a2dp Audio HAL module
PRODUCT_PACKAGES += audio.a2dp.default

# BlueZ test tools & Shared Transport user space mgr
PRODUCT_PACKAGES += \
	hciconfig \
	hcitool

# MultiDisplay
PRODUCT_PACKAGES += \
    TIMultiDisplay

$(call inherit-product, frameworks/native/build/tablet-7in-hdpi-1024-dalvik-heap.mk)
$(call inherit-product-if-exists, hardware/ti/omap4xxx/jacinto6.mk)
$(call inherit-product-if-exists, hardware/ti/wpan/ti-wpan-products.mk)
$(call inherit-product-if-exists, vendor/ti/omap5sevm/device-vendor.mk)
$(call inherit-product-if-exists, device/ti/proprietary-open/jacinto6/ti-jacinto6-vendor.mk)
$(call inherit-product-if-exists, device/ti/common-open/s3d/s3d-products.mk)
$(call inherit-product-if-exists, device/ti/proprietary-open/omap5/ducati-full_omap5panda.mk)
$(call inherit-product-if-exists, device/ti/proprietary-open/wl12xx/wlan/wl12xx-wlan-fw-products.mk)
$(call inherit-product-if-exists, device/ti/proprietary-open/wl12xx/wpan/wl12xx-wpan-fw-products.mk)
