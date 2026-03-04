# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Projects/Espressif/.esp-idf/frameworks/v5.3.1/esp-idf/components/bootloader/subproject"
  "C:/PlatformIO/telemeter_dzen/build/bootloader"
  "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix"
  "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/tmp"
  "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/src/bootloader-stamp"
  "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/src"
  "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/PlatformIO/telemeter_dzen/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
