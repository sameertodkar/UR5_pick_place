
# Te code is used to generate a GUI which displays the orientation of the cube based on the
# current position of the box in the scene. The code uses the libraries of OpenGl and pygame.
# The code displays the rotation angles in the format of roll, pitch, yaw which is used to
# determine the orientation of the box


import rospy
from geometry_msgs.msg import Pose
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *


useQuat = True   # set true for using quaternions, false for using y,p,r angles

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Cube orientation visualization")
    resizewin(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    rospy.init_node('Visualize', anonymous=True)

    oldW =0.0
    oldX =0.0
    oldY =0.0
    oldZ =0.0
    count =0
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        try:    
            rospy.Subscriber("box_pose", Pose, callback)
            # print('hello: {}'.format(count))

            if(oldW != ori_w):
                print("NEW VaLUE_________________________________{}".format(count))
                draw(ori_w, ori_x, ori_y,ori_z) #------------------ Enter Quaternions here[W,X,Y,Z]/ or 1,R,P,Y
                pygame.display.flip()
                frames += 1
                count = count+1
            oldW = ori_w
            oldX = ori_x
            oldY = ori_y
            oldZ = ori_z
        except:
            pass


def resizewin(width, height):
    """
    For resizing window based on input width and height
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def draw(w, nx, ny, nz):
    """
    Function to draw a cube based on the input arguments

    Args:
        w (float): the real part of quaternion 
        nx (float): imaginary x of quaternion
        ny (float): imaginary y of quaternion
        nz (float): imaginary z of quaternion
    """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.6, 2), "Module to visualize cuboid", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(-roll, 0.00, 0.00, 1.00)
        glRotatef(pitch, 1.00, 0.00, 0.00)
        glRotatef(yaw, 0.00, 1.00, 0.00)


    ##### To generate the box using vertices and color combinations
    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    """
    Function for defining a text on the output window

    Args:
        position ([int]): Position of the text on the window
        textString ([string]): text to be added
        size ([int]): size of the text
    """
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    """
    Function to convert quaternion form of orientation to yaw pitch roll

    Args:
        q (quaternion): quaternion elements w,x,y,z

    Returns:
        [list]: yaw, roll, pitch angles in a list
    """
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


def callback(data):
    """
    Function for providing a callback for subscriber

    Args:
        data ([PoseStamp]): position and orientation variable
    """

    global ori_x, ori_y, ori_z, ori_w
    ori_x = data.orientation.x
    ori_y =  data.orientation.y
    ori_z =  data.orientation.z
    ori_w = data.orientation.w
    
    return 


if __name__ == '__main__':

    main()
    