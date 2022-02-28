# Hand E Demos

Here are some examples of demo's used for Hand E. To run these, the robot should be launched first (instructions for this can be found [here](https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_robot_launch)).

| Demo          | Arguments     |  Options     |
| ------------- | ----------- | ------------- |
| open.py       | -s --side  |     'right', 'left' or 'both' |
| close.py      | -s --side  | 'right', 'left' or 'both' |
| burn_in.py    | -s --side <br> -ht --hand_type | 'right', 'left' or 'both' <br> 'hand_e', 'hand_e_plus', 'hand_lite' or 'hand_extra_lite' |
|  demo.py      | -s --side <br> -ht --hand_type <br> -tac --tactiles | 'right', 'left' or 'both' <br> 'hand_e', 'hand_lite' or 'hand_extra_lite' <br> Include this argument if the hand has tactiles |

## Example launch

```
rosrun sr_demos demo.py -s right -ht hand_e -tac
```
```
rosrun sr_demos open.py --side both
```
