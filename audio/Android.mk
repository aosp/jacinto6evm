# Copyright (C) 2013 Texas Instruments
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

LOCAL_PATH := $(call my-dir)

# build multizone audio if the OMAP_MULTIZONE_AUDIO flag is set to true
ifeq ($(OMAP_MULTIZONE_AUDIO),true)
include $(CLEAR_VARS)

include $(LOCAL_PATH)/multizone/Android.mk

else # build the legacy audio if the OMAP_MULTIZONE_AUDIO flag is set to false

include $(CLEAR_VARS)

include $(LOCAL_PATH)/legacy/Android.mk

endif
