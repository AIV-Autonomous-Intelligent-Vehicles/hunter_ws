#!~/bash
# # indoor
# v4l2-ctl -d /dev/video2 -c focus_automatic_continuous=0 -c white_balance_automatic=0 -c auto_exposure=1
# v4l2-ctl -d /dev/video4 -c focus_automatic_continuous=0 -c white_balance_automatic=0 -c auto_exposure=1


# # v4l2-ctl -d /dev/video2 -c exposure_time_absolute=600
# # v4l2-ctl -d /dev/video2 -c exposure_time_absolute=300
# v4l2-ctl -d /dev/video2 -c white_balance_temperature=4000


# v4l2-ctl -d /dev/video4 -c exposure_time_absolute=800
# v4l2-ctl -d /dev/video4 -c white_balance_temperature=5500

# outdoor
v4l2-ctl -d /dev/video2 -c focus_automatic_continuous=0 #-c white_balance_automatic=0 -c auto_exposure=1
v4l2-ctl -d /dev/video4 -c focus_automatic_continuous=0 #-c white_balance_automatic=0 -c auto_exposure=1


# v4l2-ctl -d /dev/video2 -c exposure_time_absolute=600
# v4l2-ctl -d /dev/video2 -c exposure_time_absolute=30
# v4l2-ctl -d /dev/video2 -c white_balance_temperature=4800


# v4l2-ctl -d /dev/video4 -c exposure_time_absolute=10
# v4l2-ctl -d /dev/video4 -c white_balance_temperature=4300