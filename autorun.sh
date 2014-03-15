#!/bin/bash

pkill -f "kn2cssl"
read -rsp $'Press any key to continue...\n' -n1 key

cd bin

while true; do

	pkill -f "kn2cssl"
	./kn2cssl nogui &

	old_md5 = $(md5sum kn2cssl)
	new_md5 = $old_md5
	while [ "$old_md5" == "$new_md5" ] do
		sleep 15
		new_md5 = $(md5sum kn2cssl)
	done

done
