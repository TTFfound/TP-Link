#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include

obj-y	+= step_counter.o

obj-$(CONFIG_MTK_STEPSIGNHUB)		+=	stepsignhub/

ifeq ($(CONFIG_CUSTOM_KERNEL_STEP_COUNTER),"OTHER_VENDOR")
obj-y += other_vendor/
endif

# [fengyunliang start] Porting sensors
ifeq ($(CONFIG_CUSTOM_KERNEL_STEP_COUNTER),y)
obj-$(CONFIG_MTK_BMI160_STC) += bmi160_stc/
endif
# [fengyunliang end]
