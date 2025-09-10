# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0


zephyr_library()
zephyr_include_directories(.)
zephyr_library_sources(cilo72/ic/tmc5160.cpp cilo72/motion/tmc5160.cpp)

    
