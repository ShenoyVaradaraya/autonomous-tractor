#!/bin/bash
# xterm -T roscore -e bash -c 'roscore' &
# roscore_pid=$!
# sleep 2

xterm -T Embedded -e bash -c 'ssh mic-730ai@169.254.184.174 && rosrun rosserial_python serial_node.py /dev/stm' &
embedded_pid=$!
xterm -T Ceres_core -e bash -c 'cd ceres_ws && source devel/setup.bash && rosrun ceres_core ceres_core.py' &
core_pid=$!
xterm -T PlannerServer -e bash -c 'cd ceres_pp_ws && source devel/setup.bash && roslaunch boustrophedon_server boustrophedon_server.launch' &
pserver_pid=$!
xterm -T PlannerClient -e bash -c 'cd ceres_pp_ws && source devel/setup.bash && rosrun boustrophedon_server boustrophedon_client' &
pclient_pid=$!
xterm -T LocalPlanner -e bash -c 'cd ceres_pp_ws && source devel/setup.bash && rosrun boustrophedon_server local_planner' &
plocal_pid=$!
xterm -T CeresPP -e bash -c 'cd ceres_ws && source devel/setup.bash && rosparam set /obstacle false && rosparam set /end_of_trajectory false && rosrun ceres_op CeresPP.py' &
cpure_pid=$!
xterm -T Ardusimple -e bash -c 'cd ceres_ws && source devel/setup.bash && roslaunch ublox_gps ardusimple.launch' &
mlardu_pid=$!
xterm -T Localizer -e bash -c 'cd ceres_ws && source devel/setup.bash && roslaunch ceres_localization ceres_localization.launch' &
mlocalizer_pid=$!
xterm -T Ceres_init -e bash -c 'cd ceres_ws && source devel/setup.bash && rosrun ceres_op Ceres_init.py' &
cinit_pid=$!
xterm -T DL -e bash -c 'ssh mic-730ai@169.254.184.174' &
dl_pid=$!

if `ps -p $pserver_pid > /dev/null `; then
    echo "Starting stacks ... "
    (trap exit SIGINT ; read -r -d '' _</dev/tty )
else 
    kill $embedded_pid &
    kill $core_pid &
    kill $pserver_pid &
    kill $pclient_pid &
    kill $plocal_pid &
    kill $cpure_pid &
    kill $mlardu_pid &
    kill $mlocalizer_pid &
    kill $cinit_pid &
    kill $dl_pid &
fi
