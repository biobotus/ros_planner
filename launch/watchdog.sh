#!/bin/bash

watch='true'
while [ "$watch" == "true" ];do
        ping -c 3 10.43.156.135  2>&1 > /dev/null
        if (echo "$?" -eq "0")
        then
            echo "Host reachable"
        else
            echo "echo error code is: "
            echo $host_status
            watch='false'
        fi
done

