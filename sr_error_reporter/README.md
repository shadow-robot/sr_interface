# sr_error_reporter

A package for calculating and publishing errors.

This node can be started by:
```
rosrun sr_error_reporter error_reporter
```

## Underactuation error

Underactuation error is calculated by listening to topic `/joint_states` for
actual values from sensors and by listening to topics `/lh_trajectory_controller/command`
and `/lh_trajectory_controller/command` for desired values.
When one of these topics changes, kinematic model is reset to zeros and only J1
and J2 joints for underactuated fingers are set to either actual or desired
values. Then fingertip position is calculated by running forward kinemates.
Cartesian distance between actual and desired fingertips are calculated and
published to a topic per finger:
* /underactuation_error/rh_fftip
* /underactuation_error/rh_lftip
* /underactuation_error/rh_mftip
* /underactuation_error/rh_rftip
