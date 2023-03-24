from cmath import sqrt
from typing import Dict
from xmlrpc.client import Boolean
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Empty

from pynput import keyboard

import os

def clear_terminal():
    os.system('cls' if os.name=='nt' else 'clear')


class RemoteController(Node):
    keys: Dict[str, Boolean] = {}
    steering: Float64 = Float64()
    lastSpeed = 0.0
    debug = False

    def __init__(self, debug = False):
        super().__init__('remote_controller')
        self.debug = debug
        self.speed = self.create_publisher(Float64, '/remote_controller/speed', 1)
        self.steer = self.create_publisher(Float64, '/remote_controller/steering', 1)
        self.abort = self.create_publisher(Empty, '/remote_controller/abort', 1)
        self.switch_lane = self.create_publisher(Empty, '/remote_controller/switch_lane', 1)
        self.reset = self.create_publisher(Empty, '/remote_controller/reset', 1)
        self.popState = self.create_publisher(Empty, '/remote_controller/pop_state', 1)
        self.recreateStack = self.create_publisher(Empty, '/remote_controller/recreate_stack', 1)
        self.grandmaMode = self.create_publisher(Empty, '/remote_controller/slowmode', 1)

        # t = turbo
        self.keys = {'w': False, 'a': False, 's': False, 'd': False, 
                     'q': False, 't': False, 'f': False, 'r': False,
                     'p': False, 'x': False, 'e': False}
        self.keyboard_listener()

    def on_press(self, key):
        self.handleKey(key, True)

    def on_release(self, key):
        self.handleKey(key, False)
        
    def handleKey(self, key, value: Boolean):
        try:
            if(isinstance(key, keyboard.KeyCode) and (key is not None) and (self.keys.get(key.char.lower()) is not None)):
                self.keys[key.char.lower()] = value
                self.key_watcher()
            elif(isinstance(key, keyboard.Key)):
                if(key == keyboard.Key.shift):
                    self.keys['t'] = value
                    self.key_watcher()
        except:
            pass

    def keyboard_listener(self):
        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release,) as listener:
            listener.join()

    def key_watcher(self):
        msg = Float64()

        # speed
        if (self.keys['w']):
            msg.data = 1.0
            if(self.keys['t']):
                msg.data *= 3
        elif (self.keys['s']):
            msg.data = -1.0
            if(self.keys['t']):
                msg.data *= 3
        else:
            msg.data = 0.0

        self.lastSpeed = msg.data
        self.speed.publish(msg)

        # steering
        if (self.keys['d']):
            if (self.steering.data < 20.0):
                self.steering.data = 20.0
            elif (self.steering.data < 30.0):
                self.steering.data *= 1.05 
            else:
                self.steering.data = 30.0
        elif (self.keys['a']):
            if (self.steering.data > -20.0):
                self.steering.data = -20.0
            elif (self.steering.data > -30.0):
                self.steering.data *= 1.05
            else:
                self.steering.data = -30.0
        else:
            self.steering.data = 0.0

        msg.data = self.steering.data
        self.steer.publish(msg)

        # toggle steering from image processing
        if(self.keys['q']):
            self.abort.publish(Empty())

        # manually start overtaking
        if(self.keys['f']):
            self.switch_lane.publish(Empty())

        # force reset (will stop overtaking, reset pids)
        if(self.keys['r']):
            self.reset.publish(Empty())

        # remove top state 
        if(self.keys['p']):
            self.popState.publish(Empty())

        # create stack as if the vehicle just started driving
        if(self.keys['x']):
            self.recreateStack.publish(Empty())

        # toggle slower driving
        if(self.keys['e']):
            self.grandmaMode.publish(Empty())

        if(self.debug):
            self.printDebug()

    def printDebug(self):
        clear_terminal()
        print("----------------------------------------------------")
        print("speed: " + str(self.lastSpeed))
        print("steering: " + str(self.steering.data))
        print("")
        print("w: " + str(self.keys['w']))
        print("a: " + str(self.keys['a']))
        print("s: " + str(self.keys['s']))
        print("d: " + str(self.keys['d']))
        print("")
        print("[abort]\tq: " + str(self.keys['q']))
        print("[lanes]\tf: " + str(self.keys['f']))
        print("[reset]\tr: " + str(self.keys['r']))
        print("[state]\tp: " + str(self.keys['p']))
        print("[stack]\tx: " + str(self.keys['x']))
        print("")
        print("t (shift): " + str(self.keys['t']))
        print("----------------------------------------------------")

def main(args=None):
    rclpy.init(args=args)

    remoteController = RemoteController()

    rclpy.spin(remoteController)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    remoteController.destroy_node()
    rclpy.shutdown()

def debug(args=None):
    rclpy.init(args=args)

    remoteController = RemoteController(True)

    rclpy.spin(remoteController)

    remoteController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
