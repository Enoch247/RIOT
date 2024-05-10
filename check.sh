#! /bin/sh -e

pattern=$1
files=build/stm32/cmsis/*/Include/*.h

for file in $files
do
    if `grep -q $pattern $file`
    then
        echo $file
    fi
done
