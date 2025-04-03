#! /bin/sh -e

#boards=`make info-boards`
boards=""
boards="$boards `ls -1 ../../boards/ | grep stm32`"
boards="$boards `ls -1 ../../boards/ | grep nucleo`"

for board in $boards
do
	make BOARD=$board
done
