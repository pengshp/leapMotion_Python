#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__='pengshp'

import Leap, sys, thread, time
import threading,serial,binascii
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        #self.grab=grab
        print "Initialized" #初始化

    def on_connect(self, controller):
        print "Connected"  #连接

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
    # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
                frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))
        # Get hands
        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"
            print "  %s, id %d, position: %s" % (
                handType, hand.id, hand.palm_position)
            grab=hand.grab_strength
            #global grab
            #grab = hand.pinch_strength
            print "grab=%f" %grab
            value=int(grab*49+50)
            value=str(value)
            ser=serial.Serial('COM6',4800)
            """     if  0.0 <= vaul < 0.3:
                ser.write(vaul)
               print "串口发送成功0"
                time.sleep(0.5)
            elif 0.3 <= vaul <=0.7:
                ser.write('01')
                print "串口发送成功1"
                time.sleep(0.5)
            else:                        
            """
            #binascii_a2b_hex()
            print value
            #hexer=value.decode("hex")
            #print hexer
            ser.write('2')
            ser.write(value)
            #ser.write(hexer)
            #print hexer            
            time.sleep(0.05)              

            
            # Get the hand's normal vector and direction
            #normal = hand.palm_normal
            normal = hand.palm_position
            direction = hand.direction
            numx=int((-normal[0])/7)
            numx=numx+48
            numx=str(numx)
            print numx
            #ser.write('2')
            #ser.write(numx)

            numy=int((-normal[1])/5)
            numy=numy+60
            numy=str(numy)
            print "手掌y向量"
            print numy

            numz=int((-normal[2])/8)
            numz=numz+45
            numz=str(numz)
            print "手掌z向量"
            print numz
            #ser.write('2')
           # ser.write(numz)
            #ser.write('2')
            #ser.write(numx)
            #ser.write('2')
            #ser.write(numx)
            print "palm_normal=手掌向量" 
            print normal[0]
            print normal[1]
            print normal[2]
            time.sleep(0.5)              
            #for i in normal:
                #str_normal=str(int (normal[i]))
                #hex_normal=str_normal.decode("hex")
                #print normal[i]

            print "direction=" 
            print direction
            # Calculate the hand's pitch, roll, and yaw angles
            print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)

            # Get arm bone
            arm = hand.arm
            print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
                arm.direction,
                arm.wrist_position,
                arm.elbow_position)

            # Get fingers
            for finger in hand.fingers:

                print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                    self.finger_names[finger.type],
                    finger.id,
                    finger.length,
                    finger.width)

                # Get bones
                for b in range(0, 4):
                    bone = finger.bone(b)
                    print "      Bone: %s, start: %s, end: %s, direction: %s" % (
                        self.bone_names[bone.type],
                        bone.prev_joint,
                        bone.next_joint,
                        bone.direction)

        # Get tools
        for tool in frame.tools:

            print "  Tool id: %d, position: %s, direction: %s" % (
                tool.id, tool.tip_position, tool.direction)

        # Get gestures
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)

                # Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    clockwiseness = "clockwise"
                else:
                    clockwiseness = "counterclockwise"

                # Calculate the angle swept since the last frame
                swept_angle = 0
                if circle.state != Leap.Gesture.STATE_START:
                    previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                    swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                print "  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                        gesture.id, self.state_names[gesture.state],
                        circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                print "  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                        gesture.id, self.state_names[gesture.state],
                        swipe.position, swipe.direction, swipe.speed)

            if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                keytap = KeyTapGesture(gesture)
                print "  Key Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        keytap.position, keytap.direction )

            if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                screentap = ScreenTapGesture(gesture)
                print "  Screen Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        screentap.position, screentap.direction )
        
        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print "====================================="
        #return grab

    def regrab(self, controller):
        frame = controller.frame()
        frame.on_frame()
        grab=frame.hand.pinch_strength
        print "grab=======%f" %grab
        return self.grab

    def state_string(self, state):
            if state == Leap.Gesture.STATE_START:
                    return "STATE_START" #开始

            if state == Leap.Gesture.STATE_UPDATE:
                    return "STATE_UPDATE" #更新

            if state == Leap.Gesture.STATE_STOP:
                    return "STATE_STOP"  #停止

            if state == Leap.Gesture.STATE_INVALID:
                    return "STATE_INVALID"#无效

#串口发送函数，设置波特率为9600
def serial_send(self,str):
        ser=serial.Serial('COM3',9600)
        ser.write(str)
        #ser.close()

def main():
        listener = SampleListener()
        controller = Leap.Controller()

        # Have the sample listener receive events from the controller
        controller.add_listener(listener)
        #while True:
        #vaul=controller.regrab()
        """vaul=controller.grab
        if  0.0 < vaul <= 0.5:
            serial_send(0)
            print "串口发送成功0"
            time.sleep(1)
        elif 0.5 < vaul <=1.0:
            serial_send(1)
            print "串口发送成功1"
            time.sleep(1)
        """
        # Keep this process running until Enter is pressed
        print "Press Enter to quit..."
        try:
                sys.stdin.readline()
        except KeyboardInterrupt:
                pass
        finally:
        # Remove the sample listener when done
                controller.remove_listener(listener)
if __name__ == "__main__":
    main()