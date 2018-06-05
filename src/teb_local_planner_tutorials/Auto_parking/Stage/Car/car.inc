define laser ranger
(
  sensor
  (
    range [ 0.0  30.0 ] fov 360 samples 1440
    # range_max 6.5
    #fov 58.0
    #samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]

)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.4

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]


 size [ 4.0 1.70 1.50 ]  #real car
 #size [ 0.6 0.25 0.40 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
 #origin [ 0.0 0.0 0.0 0.0]    # Central at the center

 #origin [ -1.25 0.0 0.0 0.0]    # Central at the front

 origin [ 1.25 0.0 0.0 0.0]    # Central at the rear
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  #wheelbase 0.4 # distance between both axles
  wheelbase 2.50
  # spawn sensors
   laser(pose [ 0.0 0.0 -0.11 0.0 ])
  #laser(pose [ -1.25 0.0 -0.11 0.0 ])
)