#!/bin/bash
if grep "PROPERTY POSITION_INDEPENDENT_CODE ON" CMakeLists.txt
then
    echo ""
    echo " SDK_LIBRARY POSITION_INDEPENDENT_CODE flag already set to ON."
    echo ""
else
    sed -i '/add_library(${SDK_LIBRARY} STATIC "")/a set_property(TARGET ${SDK_LIBRARY} PROPERTY POSITION_INDEPENDENT_CODE ON)' CMakeLists.txt
    echo ""
    echo " adding POSITION_INDEPENDENT_CODE flag to sdk_core/CMakeLists.txt."
    echo ""
fi