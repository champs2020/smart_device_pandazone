# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2/components/bootloader/subproject"
  "C:/Espressif/smart_device_pandazone/build/bootloader"
  "C:/Espressif/smart_device_pandazone/build/bootloader-prefix"
  "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/tmp"
  "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/src"
  "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Espressif/smart_device_pandazone/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
