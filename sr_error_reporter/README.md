# sr_error_reporter

A package for calculating and publishing errors.

This node can be started by:
```
rosrun sr_error_reporter error_reporter
```

## Underactuation error

Underactuation error is calculated by listening to topics
`/lh_trajectory_controller/state` and `/rh_trajectory_controller/state` and
comparing `actual.positions` field to `desired.positions` field.
When one of these topics changes, the kinematic model is reset to zeros and
only J1 and J2 joints for underactuated fingers are set to either actual or
desired values. Then fingertip position is calculated by running forward
kinematics. Cartesian distance between actual and desired fingertips is
calculated and published to a topic per finger:
* /underactuation_error/rh_fftip
* /underactuation_error/rh_lftip
* /underactuation_error/rh_mftip
* /underactuation_error/rh_rftip
