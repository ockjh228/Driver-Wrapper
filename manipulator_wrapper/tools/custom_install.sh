#!/bin/bash
# note:
# if you want to install multiple packages, copy this file and rename it.

installation_name="custom_install"
installed_flag=$(false) 
# you should put the condition for installation check
# example)
# installed_flag=$(ls /lib/modules | grep -c "chromium-browser")

if [[ $installed_flag == 0 ]]; then
    echo -e " - $installation_name: \e[93m[missing]\e[0m"
    #######################################################
    # your install script should be here!



    #######################################################
    echo -e " - $installation_name: \e[93m[missing]\e[0m \e[32m-> [installed]\e[0m"
else
    echo -e " - $installation_name: \e[32m[installed]\e[0m"
fi
