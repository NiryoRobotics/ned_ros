# niryo_robot_reports

## Cron installation

```bash
crontab -e
```
paste this line at the end of the file:
```bash
@daily /home/niryo/catkin_ws/src/niryo_robot_reports/pub_new_day.sh >/dev/null 2>&1
@reboot /home/niryo/catkin_ws/src/niryo_robot_reports/pub_new_day.sh >/dev/null 2>&1
```