# import native libraries
import time
import uuid
from random import randint,  uniform
from math   import sin, cos, sqrt, isnan

# import numpy and quaternion
import numpy as np
import quaternion


# import physics engine
import pybullet as p

# import OpenGL
from OpenGL.GL   import *
from OpenGL.GLU  import *
from OpenGL.GLUT import *

from phenotype import *
from helper import *


# Define screen size.
width  = 1920
height = 1200

# Define globals for keyboard callback
posx = 0.0
posy = 0.0
posz = 0.0
rotz = 0.0
scale = 1.0
angle = 0.0
lx =  0.0
ly = -1.0

# Define yolo attributes
revolute = 0
ortho = 1

# Initialize base physics engine
physicsClient = p.connect(p.GUI)
#physicsClient = p.connect(p.DIRECT)
#p.connect(p.DIRECT)

# Initial testing  world does not involve gravity
p.setGravity(0,0,0)
p.setRealTimeSimulation(0)
p.setTimeStep(1)


# Define plane collision shape and body identifiers
# planeCS = p.createCollisionShape(p.GEOM_PLANE)
# planeBodyID = p.createMultiBody(0, planeCS)
# p.changeDynamics(   planeBodyID,       -1, 
#                     lateralFriction  = 0.001,
#                     spinningFriction = 0.001,
#                     rollingFriction  = 0.001, 
#                     linearDamping    = 0.0     )


# Create a collision shape to use with resource collisions
collision_id = p.createCollisionShape( p.GEOM_SPHERE, .01 )


def keyboard(key, x, y):
    ##  Define the keyboard callback
    ##  for user input navigation

    global posx, posy, posz, rotz, scale, angle, lx, ly

    if( key == b'w' ):
        posx  += lx * .1
        posy  += ly * .1

    if( key == b's' ): 
        posx  -= lx * .1
        posy  -= ly * .1

    if( key == b'a' ):
        angle -= .1
        lx     =  sin( angle )
        ly     = -cos( angle )

    if( key == b'd' ):
        angle += .1
        lx     =  sin( angle )
        ly     = -cos( angle )

    if( key == b'q' ): rotz  += +.1
    if( key == b'e' ): rotz  += -.1
    if( key == b'+' ): scale += +.1
    if( key == b'-' ): scale += -.1
    glutPostRedisplay()



def testRotation():
    glBegin(GL_TRIANGLES)
    glColor3f( 1, 0, 1 ) 
    
    qx,qy,qz,qw = 0,0,0,1 # identity rotation (x,y,z),s notation
    x,y,z = 0,0,0
    dx,dy,dz = 10,10,10

    a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw  )
    glVertex3f( a,b,c)            
    a,b,c = rotate( x - dx, y - dy, z - dz, qx,qy,qz,qw )
    glVertex3f( a,b,c)    
    a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw )
    glVertex3f( a,b,c)
    glEnd()


def glassRender( size, alpha, scene ):
    glDisable ( GL_LIGHTING )
    glEnable  ( GL_BLEND    )

    glColor                                      ( 1, 1, 1, alpha )
    glMaterial( GL_FRONT_AND_BACK, GL_AMBIENT,   ( 1, 1, 1, 1 ) )
    glMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE,   ( 1, 1, 1, 1 ) )
    glMaterial( GL_FRONT_AND_BACK, GL_SPECULAR,  ( 1, 1, 1, 1 ) )
    glMaterial( GL_FRONT_AND_BACK, GL_SHININESS,   32.0         )

    scene( size )

    glColor                                      ( 1, 1, 1, 1 )
    glMaterial( GL_FRONT_AND_BACK, GL_AMBIENT,   ( 0, 0, 0, 0 ) )
    glMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE,   ( 0, 0, 0, 0 ) )
    glMaterial( GL_FRONT_AND_BACK, GL_SPECULAR,  ( 1, 1, 1, 1 ) )
    
    
    

    glEnable   ( GL_LIGHTING )
    glDepthFunc( GL_EQUAL )

    scene( size )

    glDisable    ( GL_BLEND    )
    glDepthFunc  ( GL_LESS     )


def testScene( size ):
    glPushMatrix()
    glutSolidCube( size, 32, 32 )    
    glPopMatrix()

    glPushMatrix()
    glTranslatef(1,0,0)
    glutWireSphere( size, 32, 32 )
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0,1,0)
    glutWireSphere ( size, 32, 32 )
    glPopMatrix()
    
    glPushMatrix()
    glTranslatef(0,0,1)
    glScale( 1, -1, 1)
    glutWireSphere( size, 32, 32 )
    glPopMatrix()
    
    glPushMatrix()
    glTranslatef(0,0,1)
    glutWireSphere( size, 32, 32 )
    glPopMatrix()


def displayTestScene():
    glassRender( .1, .2, testScene  )
    glassRender( .2, .1, testScene  )
    
    
def displayCube(m):
    glBegin(GL_LINES)

    glColor3f(0,0,1)
    glVertex3f(0,0,0)
    glVertex3f(m,0,0)

    glColor3f(0,1,0)
    glVertex3f(0,0,0)
    glVertex3f(0,m,0)

    glColor3f(0,1,1)
    glVertex3f(0,0,0)
    glVertex3f(0,0,m)

    glColor3f(1,0,0)
    glVertex3f(m,0,m)
    glVertex3f(0,0,m)

    glColor3f(1,0,1)
    glVertex3f(m,0,0)
    glVertex3f(m,0,m)

    glColor3f(1,1,0)
    glVertex3f(m,0,0)
    glVertex3f(m,m,0)

    glColor3f(1,1,1)
    glVertex3f(m,m,0)
    glVertex3f(0,m,0)

    glColor3f(1,1,1)
    glVertex3f(0,0,m)
    glVertex3f(0,m,m)

    glColor3f(1,1,1)
    glVertex3f(0,m,m)
    glVertex3f(0,m,0)
    glEnd()


def displayPlane( x, y, m ):
    glBegin(GL_TRIANGLES)

    glNormal3f( 0,0,1)
    glColor3f(  .3, .3, .3 )
    
    glVertex3f( x +  m,  y +  m, 0)
    glVertex3f( x -1*m,  y +  m, 0)
    glVertex3f( x +  m,  y -1*m, 0)

    glVertex3f( x -1*m,  y  + m, 0)
    glVertex3f( x +  m,  y -1*m, 0)
    glVertex3f( x -1*m,  y -1*m, 0)
   
    glEnd()


def displayCreatures():
    for creature in creatures:      
        x,y,z = p.getBasePositionAndOrientation( creature.body.body[0] )[0]
        
        #print p.getBaseVelocity( creature.body.bodyId )[0]
        if isnan(x) or isnan(y) or isnan(z):
            print 'error'
            creatures.remove( creature   )
            creatures.append( Creature() )

        
        creature.moveBody()
        creature.drawBody()


def parseResources():
    for resource in resources:
        contact_points =  p.getContactPoints( resource.body_id ) 
        for cp in contact_points:
            for creature in creatures:
                if creature.body.bodyPartId == cp[2]:
                    creature.score += 1
                    resources.remove (resource )
        resource.render()


class Resource:
    def __init__(self, size, position, color ):
        self.x, self.y, self.z = position
        self.r, self.g, self.b = color
       
        self.size         = size
        
        self.body_id      = p.createMultiBody( 1,
                                               collision_id,
                                               -1,
                                               [ self.x, self.y, self.z ],
                                               [0,0,0,1],
                                               [0,0,0,0],
                                               [0,0,0,1] )

    def render( self ):
        self.x, self.y, self.z = p.getBasePositionAndOrientation( self.body_id ) [0]
        glColor                                      ( self.r, self.g, self.b )
        glMaterial( GL_FRONT_AND_BACK, GL_AMBIENT,   ( self.r, self.g, self.b, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE,   ( self.r, self.g, self.b, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_SPECULAR,  ( 1, 1, 1, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_SHININESS,   32.0         )

        glPushMatrix ()
        glTranslatef ( self.x, self.y, self.z )
        glutWireSphere( self.size, 32, 32 )
        glPopMatrix  ()


class Creature:
    # Each creature is a phenotype of a genotype
    # it containts a function to draw its body parts
    # at each frame
    def __init__(self):

        self.body  = Phenotype(p)

        self.dimensions     = self.body.getDimensions()
        self.rootDimensions = self.body.getRootDimensions()
        
        self.name  = uuid.uuid4()
        self.color = random3f  (.1, 1 )
        
        self.energy = 100.0
        self.health = 100.0
        self.score  = 0




    def moveBody(self):
        # print "\n"
        # print str( self.name ).split('-')[0]
        # print 'energy: {0}, health {1}'.format( self.energy, self.health )
        # print self.body.g.genotype.edges()
        # print list(nx.edge_dfs( self.body.g.genotype, -1 ))
        
        for bodypart in  self.body.body:
            for joint in range (p.getNumJoints( bodypart )):
        	    #print p.getJointInfo( self.body.bodyId, joint)
	            p.setJointMotorControl2( bodypart, 
                                        joint, 
                                        p.VELOCITY_CONTROL,
                                        targetVelocity = .1,
                                        maxVelocity    = 2,
                                        force = 1)


    def drawBody(self):
        #   for each body part get its global position and draw it
        #   as a rectagle based on its dimensions
        for bodypart in  self.body.body:
            for joint in range ( 0, p.getNumJoints( bodypart ) ):
                world_position    = p.getLinkState( bodypart, joint )[4]
                world_orientation = p.getLinkState( bodypart, joint )[5]
                x,  y,  z   = world_position
                qx,qy,qz,qw = world_orientation
                dx, dy, dz  = self.dimensions[ joint ]
                self.drawPart(x,y,z, dx,dy,dz, qx,qy,qz,qw)
            

        #   do the same for the root node 
        bpao        = p.getBasePositionAndOrientation( self.body.body[-1] )
        x,y,z       = bpao[0]
        qx,qy,qz,qw = bpao[1]
        dx,dy,dz    = self.rootDimensions[0]
        
        self.drawPart(x,y,z, dx,dy,dz, qx,qy,qz,qw)

        glColor3f( 1, 1, 1 ) 
        glRasterPos3f(x, y, z + dz + .01 )
        glDisable ( GL_LIGHTING )
        #glDisable(GL_BLEND) 
        for c in ( str( self.name ).split('-') [0] + str(self.score) ) :
            glutBitmapCharacter( GLUT_BITMAP_TIMES_ROMAN_10, ord( c ) )
        glEnable ( GL_LIGHTING )
        # print self.name
        # print 'Genotype Nodes: ', self.body.g.genotype.nodes()
        # print 'Genotype Edges: ', self.body.g.genotype.edges()
        # print 'World Position: ', x,y,z,'\n'
        r,g,b = self.color

        

        # glLightfv( GL_LIGHT1, GL_POSITION, [x,y,z,1])
        # glLightfv( GL_LIGHT1, GL_DIFFUSE,  [r,g,b,.5 ])
            
    
    def drawPart( self, x,y,z, dx,dy,dz, qx,qy,qz,qw ):

        r,g,b = self.color

        glMaterial( GL_FRONT_AND_BACK, GL_AMBIENT,   (  r,  g,  b, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE,   (  r,  g,  b, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_SPECULAR,  ( .5, .5, .5, 1 ) )
        glMaterial( GL_FRONT_AND_BACK, GL_EMISSION,    .1              )
        glMaterial( GL_FRONT_AND_BACK, GL_SHININESS, 32.0              )

        #glPushMatrix()
        # glTranslatef(x,y,z)
        # m  = 180.0 / np.pi
        # rx, ry, rz = p.getEulerFromQuaternion ( [ qx, qy, qz, qw ] )
        # rx = rx * m
        # ry = ry * m
        # rz = rz * m
        # #print rx,ry,rx
        # glRotatef( rx, 1, 0, 0 )
        # glRotatef( ry, 0, 1, 0 )
        # glRotatef( rz, 0, 0, 1 )
        glBegin(GL_TRIANGLES)

        

        #Face A1
        #glColor3f( 1, 0, 0 ) 
        glColor3f( *self.color )
        #glNormal3f( 0,0,1)
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw  )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
       
        
        # Face A2
        #glColor3f( 1, 1, 1 ) 
        #glNormal3f( 0,1,0)
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw  )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z + dz, qx,qy,qz,qw  )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw  )
        glVertex3f( a,b,c)
        # print x - dx, y - dy, z + dz
        # print x + dx, y - dy, z + dz
        # print x + dx, y - dy, z - dz


        # Face B1
        #glColor3f( 0, 0, 1 )
        
        a,b,c = rotate( x - dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # # # Face B2
        #glColor3f( 0, 0, 1)
        a,b,c = rotate( x - dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face C1
        #glColor3f( 0, 0, 1 )
        a,b,c = rotate( x - dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face C2
        #glColor3f( 0, 0, 1 )
        a,b,c = rotate( x - dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face D1
        #glColor3f( 1, 1, 0 )
        a,b,c = rotate( x + dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face D2
        #glColor3f( 1, 1, 0 )
        a,b,c = rotate( x + dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face E1
       # glColor3f( 1, 0, 1 )
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        #Face E2
        #glColor3f( 1, 0, 1 )
        a,b,c = rotate( x - dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z + dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        # Face G1
        #glColor3f( 1, 1, 1 )
        a,b,c = rotate( x - dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x - dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        #Face G2
        #glColor3f( 1, 1, 1 )
        a,b,c = rotate( x - dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y - dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)
        a,b,c = rotate( x + dx, y + dy, z - dz, qx,qy,qz,qw )
        glVertex3f( a,b,c)

        
        glEnd()
       # glPopMatrix()
        
        #


##  Main Loop 
def displayFun():
    global posx, posy, posz, rotz, scale

    p.stepSimulation( physicsClient )
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT| GL_STENCIL_BUFFER_BIT)          
    glLoadIdentity()
    
    ## first display view camera, then display the creatures and then the scene
    gluLookAt( posx,      posy,      0.0,
               posx + lx, posy + ly, 0,
               0,         0,         1   )

        
    #parseResources()
    displayCreatures()
    displayTestScene()

    glutPostRedisplay()
    glutSwapBuffers()          

    glFlush()

    time.sleep(0.03)
    

def initLights():
    ## Setup light sources
    glDisable( GL_LIGHT0 )

    glEnable ( GL_LIGHT1 )
    glLightfv( GL_LIGHT1, GL_POSITION, [ 1, 0, 0,  1 ])
    glLightfv( GL_LIGHT1, GL_AMBIENT,  [.2, .2, .2, 1 ]) 
    glLightfv( GL_LIGHT1, GL_DIFFUSE,  [.5, .5, .5, 1 ]) 
    glLightfv( GL_LIGHT1, GL_SPECULAR, [ 1,  1,  1, 1 ]) 

    glEnable ( GL_LIGHT2 )
    glLightfv( GL_LIGHT2, GL_POSITION, [0, 1, 0, 1 ])
    glLightfv( GL_LIGHT2, GL_AMBIENT,  [.2, .2, .2, 1 ]) 
    glLightfv( GL_LIGHT2, GL_DIFFUSE,  [.5, .5, .5, 1 ]) 
    glLightfv( GL_LIGHT2, GL_SPECULAR, [ 1,  1,  1, 1 ]) 

    glEnable ( GL_LIGHT3 )
    glLightfv( GL_LIGHT3, GL_POSITION, [0, 0,  1, 1 ])
    glLightfv( GL_LIGHT3, GL_AMBIENT,  [.2, .2, .2, 1 ]) 
    glLightfv( GL_LIGHT3, GL_DIFFUSE,  [.5, .5, .5, 1 ]) 
    glLightfv( GL_LIGHT3, GL_SPECULAR, [ 1,  1,  1, 1 ]) 

   
    # glEnable ( GL_LIGHT4 )   
    # glLightfv( GL_LIGHT4, GL_POSITION, [ 0,  0,  0, 1 ])
    # glLightfv( GL_LIGHT4, GL_AMBIENT,  [.2, .2, .2, 1 ]) 
    # glLightfv( GL_LIGHT4, GL_DIFFUSE,  [.5, .5, .5, 1 ]) 
    # glLightfv( GL_LIGHT4, GL_SPECULAR, [ 1,  1,  1, 1 ]) 


def initFun():
    # Initialise more OpenGL parameters
    glViewport   ( 0,0, width, height )
    glClearColor ( 0.0, 0.0, 0.0, 0.0 )
    glColor3f    ( 0.0, 0.0, 0.0 )
    glPointSize  ( 1.0 )

    # Enable depth to draw front faces last
    glEnable     ( GL_DEPTH_TEST )
    glDepthFunc  ( GL_LESS       )

    ## Face culling is supposed to reduce number of faces draws
    # glEnable     ( GL_CULL_FACE )
    # glCullFace   ( GL_FRONT     )
    # glFrontFace  ( GL_CCW       )


    ## Stencil buffer buffers things i do not know
    # glEnable     ( GL_STENCIL_TEST   )
    # glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE)
    # glStencilFunc( GL_ALWAYS, 1, 255 )
    # glStencilMask( 255 )
    

    # Using the projection matrix is done once during init
    # to set the projection parameters 
    glMatrixMode ( GL_PROJECTION )
    glLoadIdentity()
    gluPerspective ( 50, float(width) / float(height), 0.1, 100) 

    # Switch to Modelview matrix to use from now on
    # also always load the identity matrix after changing modes
    glMatrixMode( GL_MODELVIEW )
    glLoadIdentity()
    
    # Opengl Light Source Configuration   
    glEnable     ( GL_LIGHTING )
    #glShadeModel ( GL_SMOOTH ) # GL_FLAT  or  GL_SMOOTH 
    glShadeModel ( GL_FLAT ) # GL_FLAT  or  GL_SMOOTH 
    glBlendFunc  ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA )
    
    initLights()
    

if __name__ == '__main__':
    # Init GLUT and create Window
    glutInit()
    
    # Initialise creature and resource lists
    creatures = []
    resources = []

    # generate a predefined number of resources at random points
    # for i in range(20):
    #     x,y,z   = random3f( -1.0, 1.0 )
    #     resources.append  ( Resource( .01, ( x,y,z ), ( 1,1,0 ) ) ) 

    # generate a predefined number of creatures
    for i in range( 10 ):
        creatures.append( Creature() )
    
    
    # Initialise and create a window
    glutInitWindowSize ( width, height )
    # glutInitWindowPosition (100, 100)
    glutCreateWindow("Creatures")
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GL_STENCIL | GLUT_DEPTH )

    # register callbacks
    glutDisplayFunc  ( displayFun )   
    glutKeyboardFunc ( keyboard   )

    initFun()
    glutMainLoop()


##  TODO LIST
##  implement camera             ( init implementation ok)
##  integrate keyboard and mouse ( used glu keyboard callback, but no mouse yet )
##  design fitness functions
##  assign attributes to creatures (health, energy, etc)
##  assign functions to creatures (, energy, etc)
##  read shadows.zip and implement shadows (shaders?)
##  generate terrain mesh 
##  add textures
##  implement neural network for sensory i/o
##  implement VBOs
##  implement rotations from quaternions    ( still on it)
##  ditch glu for glm as glu appears .on the forums. to be deprecated
##  use softbody dynamics
##  instead of rectagles for creature body parts use meshes


##  NOTES numpy-quaternion, and pybullet have the representation
##  reversed:   [w,x,y,z]   and [x,y,z,w]
##
##  While the ambient, diffuse, specular and emission material parameters
##  all have alpha components, only the diffuse alpha component is used in
##  the lighting computation.
        

class Consciousness:

    def __init__(self):
        self.exteroceptive = None
        self.interoceptive = None
        self.affective     = None

