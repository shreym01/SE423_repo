import numpy as np


class EulerAngles: #{{{1
    def __init__(self,phi=0,theta=0,psi=0):
        self.phi = phi
        self.theta = theta
        self.psi = psi

    def set(self,phi,theta,psi):
        self.phi = phi
        self.theta = theta
        self.psi = psi

    def vec(self):
        return np.array([self.phi, self.theta, self.psi])


class Quaternion: #{{{1
    def __init__(self,w=0,x=0,y=0,z=0):
        self.set(w,x,y,z)

    def set(self,w,x,y,z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def vec(self):
        return np.array([self.w, self.x, self.y, self.z])

def Lmatrix(q): #{{{1
    return np.array([[q.w, q.z, -q.y, q.x],
                     [-q.z, q.w, q.x, q.y],
                     [q.y, -q.x, q.w, q.z],
                     [-q.x, -q.y, -q.z, q.w]])

def Rmatrix(q): #{{{1
    return np.array([[q.w, -q.z, q.y, q.x],
                     [q.z, q.w, -q.x, q.y],
                     [-q.y, q.x, q.w, q.z],
                     [-q.x, -q.y, -q.z, q.w]])


def quaternionConjugate(q): #{{{1
    return Quaternion(q.w, -q.x, -q.y, -q.z)


def quaternionMultiply(q1, q2): #{{{1
    qout = Quaternion()
    qout.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z*q2.z
    tmp1 = np.array([q1.x, q1.y, q1.z])
    tmp2 = np.array([q2.x, q2.y, q2.z])
    tmp = q1.w * tmp2 + q2.w * tmp1 + np.cross(tmp1,tmp2)
    qout.x = tmp[0]
    qout.y = tmp[1]
    qout.z = tmp[2]
    return qout

def quaternionNormalize(q): #{{{1
    tolerance = 1e-6
    mag2 = q.w * q.w + q.x*q.x + q.y*q.y + q.z*q.z
    if (abs(mag2) > tolerance) and (abs(mag2 - 1.0) > tolerance):
        mag = np.sqrt(mag2)
        q.x = q.x / mag
        q.y = q.y / mag
        q.z = q.z / mag
        q.w = q.w / mag
    return q

def skew(x): #{{{1 turn 3-vector into skew symmetric matrix
    return np.array([[0,-x[2], x[1]],[x[2], 0, -x[0]], [-x[1], x[0], 0]])

def wedge(v): #{{{1
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                     [-v[1],  v[0],    0]])

def unwedge(M): #{{{1
    return np.array([-M[1,2], M[0,2], -M[0,1]])


def euler2quat(angles): #{{{1
    phi2 = angles[0]/2.0
    theta2 = angles[1]/2.0
    psi2 = angles[2]/2.0

    sinphi = np.sin(phi2)
    sintheta = np.sin(theta2)
    sinpsi = np.sin(psi2)
    cosphi = np.cos(phi2)
    costheta = np.cos(theta2)
    cospsi = np.cos(psi2)

    q = Quaternion(cosphi*costheta*sinpsi - sinphi*sintheta*cospsi,
                   sinphi*costheta*cospsi - cosphi*sintheta*sinpsi,
                   cosphi*sintheta*cospsi + sinphi*costheta*sinpsi,
                   cosphi*costheta*cospsi + sinphi*sintheta*sinpsi)
                   
    return q

def axisangle2quat(axis, angle): #{{{1
    angle2 = angle/2.0
    # make sure to normalize axis
    axis = axis / np.linalg.norm(axis)

    sinangle2 = np.sin(angle2)

    q = Quaternion()
    q.w = np.cos(angle2)
    q.x = axis[0] * sinangle2
    q.y = axis[1] * sinangle2
    q.z = axis[2] * sinangle2

    return q

def quat2euler(q): #{{{1
    angles = EulerAngles()
    angles.phi = np.arctan2( 2*(q.w * q.x + q.y*q.z), 1-2*(q.x**2 + q.z**2))
    angles.theta = np.arcsin(2*(q.w*q.y - q.x*q.z))
    angles.psi = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
    return angles

def quat2rot(q): #{{{1
    R = np.zeros((3,3))
    R[0,0] = 1 - 2*(q.y**2 + q.z**2)
    R[0,1] = 2*(q.x*q.y - q.w*q.z)
    R[0,2] = 2*(q.w*q.y + q.x*q.z)
    R[1,0] = 2*(q.x*q.y + q.w*q.z)
    R[1,1] = 1 - 2*(q.x**2 + q.z**2)
    R[1,2] = 2*(q.y*q.z - q.w*q.x)
    R[2,0] = 2*(q.x*q.z - q.w*q.y)
    R[2,1] = 2*(q.w*q.x + q.y*q.z)
    R[2,2] = 1 - 2*(q.x**2 + q.y**2)
    return R



def CHOP(x,llim,ulim): #{{{1 chop between lower and upper limit 
    """ x is a scalar, llim and ulim are lower and upper limits """
    if (x < llim):
        ret = llim
    elif (x > ulim):
        ret = ulim
    else:
        ret = x
    return ret

def angle_diff(a1,a2): #{{{1 angle difference
   return (((a1 - a2) + 3*np.pi) % (2*np.pi)) - np.pi

def rungekutta(f, x, t, dt, *args): #{{{1
    """ fourth-order Runge-Kutta integration.
        xk: current state
        uk: input to hold constant over interval [t, t+dt]
        f: system dynamics   xdot = f(x,u) 
    """
    #uk = args[0]

    xp = f(x + (dt/2) * f(x, t, *args), t, *args)
    xpp = f( x + (dt/2)*xp, t, *args)
    xppp = f(x + (dt/2)*xpp, t, *args)
    xnext = x + (dt/6) * (f(x, t, *args) + 2*xp + 2*xpp + xppp)
    return xnext

def euler(f, x, t, dt, *args): #{{{1
    """ Euler method (one step).
        f: system dynamics:  xdot = f(x,u)
        args = arguments for the function f, e.g.:
            xk: current state
            uk: input to hold constant over interval [t, t+dt]
    """
    xnext = x + f(x, t, *args)*dt
    return xnext

def list2mat(v): #{{{1
    N = len(v)
    n = len(v[0])
    V = np.empty((n,N))
    for i in range(N):
        V[:,i] = v[i]
    return V

def mat2list(V): #{{{1
    N = V.shape[1]
    n = V.shape[2]
    v = []
    for i in range(N):
        v.append( V[:,i] )
    #return np.array(v)
    return v


def Rbw(angles): #{{{1 rotation matrix rotating world to body
    phi, theta, psi = angles
    c,s = np.cos, np.sin
    R = np.zeros((3,3))
    R[0,0] = c(theta)*c(psi)
    R[0,1] = c(theta)*s(psi)
    R[0,2] = -s(theta)
    R[1,0] = s(phi)*s(theta)*c(psi) - c(phi)*s(psi)
    R[1,1] = s(phi)*s(theta)*s(psi) + c(phi)*c(psi)
    R[1,2] = s(phi)*c(theta)
    R[2,0] = c(phi)*s(theta)*c(psi) + s(phi)*s(psi)
    R[2,1] = c(phi)*s(theta)*s(psi) - s(phi)*c(psi)
    R[2,2] = c(phi)*c(theta)
    return R

def Rwb(angles): #{{{1 rotation matrix rotating body to world
    return np.transpose(Rbw(angles))

#[r1 r2 r3] = threeaxisrot( dcm(1,2,:), dcm(1,1,:), -dcm(1,3,:), ...
def rot2euler(dcm): #{{{1
    anglevec = threeaxisrot(dcm[0,1], dcm[0,0], -dcm[0,2], 
                                   dcm[1,2], dcm[2,2], 
                                  -dcm[1,0], dcm[1,1])
    return anglevec
 

def optiquat2euler(qin): #{{{1
    #case 'yzx'      
        #[r1 r2 r3] = threeaxisrot( -2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3)), ...
        anglevec = threeaxisrot( -2.*(qin[1]*qin[3] - qin[0]*qin[2]), 
                                    qin[0]**2 + qin[1]**2 - qin[2]**2 - qin[3]**2,
                                    2.*(qin[1]*qin[2] + qin[0]*qin[3]),
                                   -2.*(qin[2]*qin[3] - qin[0]*qin[1]), 
                                    qin[0]**2 - qin[1]**2 + qin[2]**2 - qin[3]**2)
        anglevec2 = [anglevec[0], anglevec[2], anglevec[1]]
        return EulerAngles(*anglevec2)

 
#[r1 r2 r3] = threeaxisrot(r11, r12, r21, r31, r32, r11a, r12a)
def threeaxisrot(r11, r12, r21, r31, r32, r11a=None, r12a=None):
    # find angles for rotations about X, Y, and Z axes
    r1 = np.arctan2( r11, r12 );
    r2 = np.arcsin( r21 );
    r3 = np.arctan2( r31, r32 );
    #if strcmpi( lim, 'zeror3')
        #for i = find(abs( r21 ) >= 1.0)
            #r1(i) = atan2( r11a(i), r12a(i) );
            #r2(i) = asin( r21(i) );
            #r3(i) = 0;
        #end
    #end
    return np.array([r3,r2,r1])


#def DRbw(angles): #{{{1 jacobian of rotation matrix w.r.t euler angles
    #phi, theta, psi = angles
    #c,s = np.cos, np.sin
    #sphi = np.sin(phi)
    #cphi = np.cos(phi)
    #stheta = np.sin(theta)
    #ctheta = np.cos(theta)
    #spsi = np.sin(psi)
    #cpsi = np.cos(psi)
    #DR = np.zeros((3,9))
#
    ##DR w.r.t phi
    #DR[0:3,0:3] = [[0,-stheta*cpsi,-ctheta*spsi],
                    #[cphi*stheta*cpsi + sphi*spsi, sphi*ctheta*cpsi, -sphi*stheta*spsi - cphi*cpsi],
                    #[-sphi*stheta*cpsi + cphi*spsi, cphi*ctheta*cpsi, -cphi*stheta*spsi+sphi*cpsi]]
#
    #
    #return DR

def S(angles): #{{{1 transformation mapping body angular velocities to euler angle rates
    """ euler rates = S * [p,q,r]' """
    phi, theta, psi = angles
    c,s,t = np.cos, np.sin, np.tan
    Sm = np.zeros((3,3))
    Sm[0,0] = 1.0
    Sm[0,1] = s(phi)*t(theta)
    Sm[0,2] = c(phi)*t(theta)
    #Sm[1,0] = 0.
    Sm[1,1] = c(phi)
    Sm[1,2] = -s(phi)
    #Sm[2,0] = 0.
    Sm[2,1] = s(phi)/c(theta)
    Sm[2,2] = c(phi)/c(theta)
    return Sm

def Sinv(angles):   #{{{1 transformation mapping euler angle rates to body angular velocities
    """ p,q,r = S^-1 (phidot, thetadot, psidot) """
    phi, theta, psi = angles
    c,s,t = np.cos, np.sin, np.tan
    Sm = np.zeros((3,3))
    Sm[0,0] = 1.0
    #Sm[0,1] = 0
    Sm[0,2] = -s(theta)
    #Sm[1,0] = 0.
    Sm[1,1] = c(phi)
    Sm[1,2] = s(phi)*c(theta)
    #Sm[2,0] = 0.
    Sm[2,1] = -s(phi)
    Sm[2,2] = c(phi)*c(theta)
    return Sm

   

def DS1(angles): #{{{1 partial deriv of S w.r.t. phi
    phi, theta, psi = angles
    c,s,t = np.cos, np.sin, np.tan
    DS = np.zeros((3,3))
    #DS[0,0] = 0.0
    DS[0,1] = c(phi)*t(theta)
    DS[0,2] = -s(phi)*t(theta)
    #DS[1,0] = 0.
    DS[1,1] = -s(phi)
    DS[1,2] = -c(phi)
    #DS[2,0] = 0.
    DS[2,1] = c(phi)/c(theta)
    DS[2,2] = -s(phi)/c(theta)
    return DS

    #DS[0,0] 

def DS2(angles): #{{{1 partial deriv of S w.r.t. theta
    phi, theta, psi = angles
    c,s,t = np.cos, np.sin, np.tan
    DS = np.zeros((3,3))
    # DS[0,0] = 0.0
    DS[0,1] = s(phi) / (c(theta)**2)
    DS[0,2] = c(phi) / (c(theta)**2)
    #DS[1,0] = 0.
    #DS[1,1] = c(phi)
    #DS[1,2] = -s(phi)
    #DS[2,0] = 0.
    DS[2,1] = s(phi) * t(theta)/c(theta)
    DS[2,2] = c(phi) * t(theta)/c(theta)
    return DS



