import numpy as np
from world import World
from agents import Car, RingBuilding, CircleBuilding, Painting, Pedestrian
from geometry import Point
import time
from tkinter import *
from steering_control import *
from matplotlib import pyplot as plt
import sys

dt = 0.1  # time steps in terms of seconds. In other words, 1/dt is the FPS.
world_width = 120  # in meters
world_height = 120
inner_building_radius = 30
num_lanes = 2
lane_marker_width = 0.5
num_of_lane_markers = 50
lane_width = 3.5

w = World(dt, width=world_width, height=world_height,
          ppm=6)  # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Let's add some sidewalks and RectangleBuildings.
# A Painting object is a rectangle that the vehicles cannot collide with.
# So we use them for the sidewalks / zebra crossings / or creating lanes.
# A CircleBuilding or RingBuilding object is also static -- they do not move.
# But as opposed to Painting, they can be collided with.

# To create a circular road, we will add a CircleBuilding and then a RingBuilding around it
cb = CircleBuilding(Point(world_width / 2, world_height / 2), inner_building_radius, 'gray80')
w.add(cb)
rb = RingBuilding(Point(world_width / 2, world_height / 2),
                  inner_building_radius + num_lanes * lane_width + (num_lanes - 1) * lane_marker_width,
                  1 + np.sqrt((world_width / 2) ** 2 + (world_height / 2) ** 2), 'gray80')
w.add(rb)

# Let's also add some lane markers on the ground. This is just decorative. Because, why not.
for lane_no in range(num_lanes - 1):
	lane_markers_radius = inner_building_radius + (lane_no + 1) * lane_width + (lane_no + 0.5) * lane_marker_width
	lane_marker_height = np.sqrt(2 * (lane_markers_radius ** 2) * (1 - np.cos(
		(2 * np.pi) / (2 * num_of_lane_markers))))  # approximate the circle with a polygon and then use cosine theorem
	for theta in np.arange(0, 2 * np.pi, 2 * np.pi / num_of_lane_markers):
		dx = lane_markers_radius * np.cos(theta)
		dy = lane_markers_radius * np.sin(theta)
		w.add(Painting(Point(world_width / 2 + dx, world_height / 2 + dy), Point(lane_marker_width,
		                                                                         lane_marker_height),
		               'white', heading=theta))

# A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
c1 = Car(Point(91.75, 60), np.pi / 2)
c1.max_speed = 30.0  # let's say the maximum is 30 m/s (108 km/h)
c1.velocity = Point(0, 3.0)
w.add(c1)

c2 = Car(Point(60, 95.25), np.pi)
c2.max_speed = 30.0  # let's say the maximum is 30 m/s (108 km/h)
c2.velocity = Point(0, 2.0)
w.add(c2)

w.render()  # This visualizes the world we just constructed.

# PID
pid_controller = PIDSteering(2.0, 0.001, 0.01, dt)
pid_controller2 = PIDSteering(2.0, 0.001, 0.01, dt)
err_list = []
speed_list = []

for k in range(600):
	''' PID Control '''
	v = c1.center - cb.center
	v2 = c2.center - cb.center
	v = np.mod(np.arctan2(v.y, v.x) + np.pi / 2, 2 * np.pi)
	v2 = np.mod(np.arctan2(v2.y, v2.x) + np.pi / 2, 2 * np.pi)
	err = v - c1.heading
	err2 = v2 - c2.heading
	# err_list.append(err)
	heading_new = pid_controller.correct(err)
	heading_new2 = pid_controller2.correct(err2)
	c1.set_control(heading_new, 0.06)  # no acceleration
	c2.set_control(heading_new2, 0.06)

	w.tick()  # This ticks the world for one time step (dt second)
	w.render()
	time.sleep(dt / 4)  # Let's watch it 4x
	speed_list.append(np.sqrt(c1.velocity.x ** 2 + c1.velocity.y ** 2))

	if w.collision_exists():  # We can check if there is any collision at all.
		print('Collision exists somewhere...')

# plt.figure()
# plt.plot(err_list)
# plt.plot(speed_list)
# plt.show()
