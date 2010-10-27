#!/bin/bash
RTT_LUA_MODULES=`rospack find ocl`/build/orocos-toolchain-ocl/lua/modules/?.lua
RFSM_LUA_MODULES=`rospack find rfsm`/?.lua
export LUA_PATH=";;;$RTT_LUA_MODULES;$RFSM_LUA_MODULES"
rosrun xacro xacro.py `rospack find brics_research_camp_lwr_setup`/deploys/test.xml.xacro -o `rospack find brics_research_camp_lwr_setup`/deploys/test.xml
`rospack find ocl`/install/bin/deployer-gnulinux -s `rospack find brics_research_camp_lwr_setup`/deploys/test.xml