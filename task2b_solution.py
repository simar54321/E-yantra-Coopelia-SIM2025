############################
#  BALANCING + MANIPULATOR #
#     FINAL FAST VERSION   #
############################

import math

def get_pitch(sim, obj):
    m = sim.getObjectMatrix(obj, -1)
    r20 = m[8]
    r21 = m[9]
    r22 = m[10]
    return math.atan2(-r20, (r21*r21 + r22*r22)**0.5)


def sysCall_init():
    sim = require('sim')

    # === HANDLES ===
    self.body = sim.getObject('/body')
    self.left = sim.getObject('/left_joint')
    self.right = sim.getObject('/right_joint')

    self.prismatic = sim.getObject('/Prismatic_joint')
    self.arm_joint = sim.getObject('/arm_joint')

    # === POSITION REFERENCE ===
    pos = sim.getObjectPosition(self.body, -1)
    self.x_ref = pos[0]
    self.step = 0.03

    # === STARTUP DELAY ===
    self.ready = False
    self.t0 = sim.getSimulationTime()
    self.startup_wait = 0.6

    # === PID GAINS (FAST RESPONSE) ===
    self.Kp_t = 480
    self.Kd_t = 8
    self.Ki_t = 0

    # FASTER OUTER LOOP (POSITION)
    self.Kp_p = 0.60
    self.Kd_p = 0.12
    self.Ki_p = 0

    # === LIMITS ===
    self.max_tilt = 0.10
    self.max_speed = 7.0
    self.turn = 0

    # manipulator speeds
    self.v_arm = 1
    self.v_prism = 1

    # SPACEBAR STOP FLAG
    self.stop_requested = False

    self.prev_pos = pos[0]

    sim.addStatusbarMessage("FAST CONTROLLER INIT OK")


def sysCall_sensing():
    sim = require('sim')
    msg, data, data2 = sim.getSimulatorMessage()

    if msg == sim.message_keypress:

        # =============================
        #  BASE KEYS (ARROWS ONLY)
        #  (REVERSED DIRECTION)
        # =============================

        # FORWARD/BACKWARD reversed
        if data[0] == 2007:          # UP ARROW
            self.x_ref += self.step  # was -=

        if data[0] == 2008:          # DOWN ARROW
            self.x_ref -= self.step  # was +=

        # LEFT/RIGHT reversed
        if data[0] == 2009:          # LEFT ARROW
            self.turn = +3.5         # was -3.5

        if data[0] == 2010:          # RIGHT ARROW
            self.turn = -3.5         # was +3.5

        # =============================
        #  SPACEBAR = FULL STOP
        # =============================
        if data[0] == 32:
            pos = sim.getObjectPosition(self.body, -1)
            self.x_ref = pos[0]
            self.turn = 0
            self.stop_requested = True

        # ==========================================
        #  MANIPULATOR KEYS
        # ==========================================

        # PRISMATIC (j/l)
        if data[0] == ord('j'):
            sim.setJointTargetVelocity(self.prismatic, -self.v_prism)
        if data[0] == ord('l'):
            sim.setJointTargetVelocity(self.prismatic, +self.v_prism)

        # ARM JOINT (i/k)
        if data[0] == ord('i'):
            sim.setJointTargetVelocity(self.arm_joint, +self.v_arm)
        if data[0] == ord('k'):
            sim.setJointTargetVelocity(self.arm_joint, -self.v_arm)

    else:
        # stop manipulator movement when no key is pressed
        sim.setJointTargetVelocity(self.prismatic, 0)
        sim.setJointTargetVelocity(self.arm_joint, 0)



def sysCall_actuation():
    sim = require('sim')
    t = sim.getSimulationTime()
    dt = sim.getSimulationTimeStep()
    if dt <= 0: dt = 0.01

    # === STARTUP BALANCING ===
    if not self.ready:
        if t - self.t0 > self.startup_wait:
            self.ready = True
        else:
            tilt = get_pitch(sim, self.body)
            c = max(min(-300 * tilt, 4), -4)
            sim.setJointTargetVelocity(self.left, c)
            sim.setJointTargetVelocity(self.right, c)
            return

    # === SPACEBAR STOP ===
    if self.stop_requested:
        sim.setJointTargetVelocity(self.left, 0)
        sim.setJointTargetVelocity(self.right, 0)
        self.stop_requested = False
        return

    # === READ STATE ===
    tilt = get_pitch(sim, self.body)
    pos = sim.getObjectPosition(self.body, -1)[0]
    linV, angV = sim.getObjectVelocity(self.body)
    tilt_rate = angV[1]
    pos_d = (pos - self.prev_pos) / dt
    self.prev_pos = pos

    # === OUTER LOOP (POSITION) ===
    pos_err = self.x_ref - pos
    desired_tilt = self.Kp_p * pos_err - self.Kd_p * pos_d
    desired_tilt = max(min(desired_tilt, self.max_tilt), -self.max_tilt)

    if abs(pos_err) < 0.01:
        desired_tilt = 0

    # === INNER LOOP (TILT) ===
    tilt_err = desired_tilt - tilt
    wheel_cmd = self.Kp_t * tilt_err - self.Kd_t * tilt_rate
    wheel_cmd = max(min(wheel_cmd, self.max_speed), -self.max_speed)

    # === TURNING ===
    L = wheel_cmd - self.turn
    R = wheel_cmd + self.turn

    L = max(min(L, self.max_speed), -self.max_speed)
    R = max(min(R, self.max_speed), -self.max_speed)

    # decay turning
    self.turn *= 0.9
    if abs(self.turn) < 0.05:
        self.turn = 0

    # apply motors
    sim.setJointTargetVelocity(self.left, L)
    sim.setJointTargetVelocity(self.right, R)



def sysCall_cleanup():
    sim = require('sim')
    sim.setJointTargetVelocity(self.left, 0)
    sim.setJointTargetVelocity(self.right, 0)
    sim.setJointTargetVelocity(self.prismatic, 0)
    sim.setJointTargetVelocity(self.arm_joint, 0)
