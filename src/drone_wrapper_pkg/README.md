# Drone Wrapper Package

This package maps MAVROS topics and commands from the Drone Network (Domain 11) to the Student Network (Domain 0) safely using `domain_bridge`.

## Architecture Note on Services!

Due to DDS limits bridging large services (`mavros_msgs/srv/SetMode`), we bypass `domain_bridge` for arming sequences! 
The `arming_node` runs natively on **Domain 11** via isolated `ROS_DOMAIN_ID` launch env vars, meaning it has direct, fast access to MAVROS services. 
It exposes a lightweight `/drone11/cmd/arming` alias service (`CommandBool`) to Domain 0 using the `domain_bridge`, effectively securing the student network while avoiding FastRTPS bridge MTU overruns.
