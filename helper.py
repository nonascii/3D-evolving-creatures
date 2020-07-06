# import numpy and quaternion
import numpy as np
import quaternion

from random import randint,  uniform
logging = 1
def log( text ):
    if logging == 1:
        print text


def random3f(min, max):
    return uniform(min, max) ,uniform(min, max) ,uniform(min, max) 


def random3i(min, max):
    return randint(min, max), randint(min, max), randint(min, max)


def random6f(min, max, m, M):
    a,b,c =  random3f(min,max)
    d,e,f =  random3f(m,M)
    return a,b,c,d,e,f


def rotate( x,y,z, qx,qy,qz,qw ):
    vec0         = np.quaternion(  0, x, y, z )
    quat         = np.quaternion( qw, qx, qy, qz )
    quatc        = quat.conjugate()
    vec1         = quat  * vec0  * quatc
    a,b,c,d        = vec1.x, vec1.y, vec1.z,  vec1.w
     
    text = '{0}\n{1}\n{2}\n'.format( vec0, quat, vec1 )
    #log( text )
    return x,y,z
    #return a,b,c
    #return x+a,y+b,z+c

