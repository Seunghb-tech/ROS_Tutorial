#! /usr/bin/python3
"""ROCKET BOT

A simulated Rocket with a ROS interface.  Just intended to serve as an
alternative to turtlesim for playing with topics and messages.

Publishes to:
  location  (geomtry_msgs/Point)

Subscribes to:
  thrust    (geometry_msgs/Vector3)

(0, 0, 0) is at the lower left edge of the screen.  The x axis points
to the right and the y axis points up.  Thrust along the z axis will
be ignored.  Negative thrusts along the y axis will be ignored.

Author: Nathan Sprague

"""
# Some code taken from : 
#http://www.gpwiki.org/index.php/Python:Pygame_basics



import pygame
import random
import numpy as np
import rospy

from pygame.locals import *
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

ACC_MULTIPLIER = .003
GRAVITY = 9.8

class Rocket(pygame.sprite.Sprite):
    """ This is a rocket sprite. """

    HEIGHT = 30
    WIDTH = 10
    FLAME_HEIGHT = 20
    CONE_HEIGHT = 10
    def __init__(self, screen, pos, vel):
        pygame.sprite.Sprite.__init__(self)
        self._screen = screen
        self.pos = np.array(pos, dtype='float64')
        self.vel = np.array(vel, dtype='float64')


    def update(self, thrust):
        """Update the position and velocity based on the thrust.  Redraw the
        rocket at the new position.

        """

        self.vel[1] += GRAVITY * ACC_MULTIPLIER
        self.vel += thrust * ACC_MULTIPLIER
        self.pos += self.vel

        if self.pos[0] > self._screen.get_width():
            self.pos[0] = self._screen.get_width()
            self.vel[0] = -self.vel[0]
        if self.pos[0] < 0:
            self.pos[0] = 0
            self.vel[0] = -self.vel[0]
        if self.pos[1] > self._screen.get_height()-self.HEIGHT:
            self.pos[1] = self._screen.get_height() - self.HEIGHT
            self.vel[1] = 0
        if self.pos[1] < 0:
            self.pos[1] = 0
            self.vel[1] = 0

        if thrust[0] != 0 or thrust[1] != 0:
            p1 = (self.pos[0], self.pos[1] + self.HEIGHT)
            p2 = (self.pos[0]+ self.WIDTH-1, self.pos[1] + self.HEIGHT)
            p3 = (self.pos[0]+ self.WIDTH/2, 
                  self.pos[1] + self.HEIGHT + self.FLAME_HEIGHT)
            pygame.draw.polygon(self._screen, (255, 0, 0), (p1, p2, p3), 0)

        p1 = self.pos
        p2 = (self.pos[0]+ self.WIDTH-1, self.pos[1])
        p3 = (self.pos[0]+ self.WIDTH/2, self.pos[1] - self.CONE_HEIGHT)
        pygame.draw.polygon(self._screen, (0, 0, 0), (p1, p2, p3), 0)

        pygame.draw.rect(self._screen, (0, 0, 0), Rect(self.pos, (10,30))) 


class RocketNode(object):
    def __init__(self):
        rospy.init_node('rocket_bot')
        rospy.Subscriber('thrust', Vector3, self.thrust_callback)
        self.loc_pub = rospy.Publisher('location', Point, queue_size=10)
        self.cur_thrust = np.array([0.0, 0.0])
        self.thrust_start = 0

        self.back_r = rospy.get_param("~back_r", 255)
        self.back_g = rospy.get_param("~back_g", 255)
        self.back_b = rospy.get_param("~back_b", 255)

    def thrust_callback(self, thrust):
        self.thrust_start = time.time()
        if thrust.y < 0:
            thrust.y = 0.0
        self.cur_thrust = np.array([thrust.x, -thrust.y])

    def run(self):
        pygame.init()
        width = 480
        screen = pygame.display.set_mode((width, width))
        clock = pygame.time.Clock()
        pub_rate = 10.0
        refresh_rate = 100

        pygame.display.set_caption('RocketBot 354')

        rocket = Rocket(screen, (width/2, width), [0.0, 0.])
        last_pub = 0.0
        rate = rospy.Rate(refresh_rate)
        done = False
        while not rospy.is_shutdown() and not done:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            rate.sleep()
            screen.fill((self.back_r, self.back_g, self.back_b))
            if ((self.cur_thrust[0] != 0 or self.cur_thrust[1] != 0) and
                time.time() > self.thrust_start + .6):
                self.cur_thrust = np.array([0, 0])
            rocket.update(self.cur_thrust)

            pygame.display.flip()

            if time.time() > last_pub + 1.0/pub_rate - 1/refresh_rate:
                point = Point(rocket.pos[0],
                              width - rocket.pos[1] - rocket.HEIGHT, 0)
                self.loc_pub.publish(point)
                last_pub = time.time()

if __name__ == "__main__":
    node = RocketNode()
    node.run()

