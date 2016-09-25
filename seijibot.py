from bot_interface import *
import math

class SeijiBot(BotBase):
    def __init__(self):
        self.initialized = False
    
    def initialize(self, gamestate):
        gamestate.log("Initializing...")
        #Getting UID
        self.uid = gamestate.bot.uid
        gamestate.log("This ship has uid " + str(self.uid))
        #Getting time step
        self.step = gamestate.timestep
        gamestate.log("Initialized with timestep of " + str(self.step) + "s")
        gamestate.log("Ships have a " + str(gamestate.ships[self.uid].radius) + "m radius")
        #Setting Global constants
        self.mass = 1
        self.main_thrust = 30
        self.side_thrust = 15
        self.side_thrust_offset = 2
        self.laser_charge_time = .5
        
        self.initialized = True
    #From here are some useful functions
    
    #Side functions
    def solveQuad(self, a, b, c):
        if a == 0:
            return None
        delta = b**2 - 4*a*c
        if delta < 0:
            return None
        if delta == 0:
            return (-b)/(2*a), (-b)/(2*a)
        delta = math.sqrt(delta)
        return (((-b)-delta)/(2*a), ((-b)+delta)/(2*a))
     
    def dist(self, obj1, obj2):
        return math.sqrt((obj1.posx - obj2.posx)**2 + (obj1.posy - obj2.posy)**2)

    def toRad(self, angle):
        return (float(angle)/180)*math.pi
    
    def sign(self, n):
        if n == 0:
            return 0
        return n/abs(n)
        
    def fmod(self, n, k):
        d = math.floor(n/k)
        return n - k*d

    def glob_loc(self, x1, y1, angle, x2, y2):
        rx, ry = x2 - x1, y2 - y1
        tx, ty = math.cos(-angle) * rx - math.sin(-angle) * ry, math.cos(-angle) * ry + math.sin(-angle) * rx
        return tx, ty

    def normalize(self, vec):
        sqrl = 0
        for i in vec:
            sqrl += i**2

        l = math.sqrt(sqrl)

        if l == 0.0:
            return vec

        res = []
        for i in vec:
            res.append(i/l)
        return res

    def invert(self, vec):
        return [-i for i in vec]

    #Movement functions
    
    #Change angular speed - It doesn't change linear velocity
    #Returns -> thruster value
    def angularSpeed(self, ship, final_speed):
        k = .1
        vel = self.toRad(ship.velang)
        delta = final_speed - vel
        ret = delta*k
        if ret > 1:
            ret = 1
        elif ret < -1:
            ret = -1
        return -ret

    def angDelta(self, ship, angle):
        delta = self.fmod(angle + 2*math.pi, 2*math.pi) - self.fmod(self.fmod(self.toRad(ship.ang), 2*math.pi) + 2*math.pi, 2*math.pi)
        if abs(delta) > math.pi:
            delta = (2*math.pi - abs(delta))*self.sign(-delta)
        return delta
        
    #Control ship rotation to certain angle - It doesn't change linear velocity
    #Returns -> thruster value
    def lookAt(self, ship, final_ang):
        kP, kD = .6, 3.5

        out = -kP*self.angDelta(ship, final_ang) + kD*self.toRad(ship.velang)*self.step
        if out > 1:
            out = 1
        elif out < -1:
            out = -1
        return out
        
    #Accelerate ship towards certain coordinate - It doesn't change velang
    #Returns -> main thruster value, frontal thruster value, back thruster value
    def accelerateTo(self, ship, towx, towy, pot = 1):
        tstep = self.step
        fmax = self.main_thrust/self.mass

        angles = self.toRad(ship.ang)
        
        x, y = self.glob_loc(ship.posx, ship.posy, angles, towx, towy)

        res = [0, 0, 0]

        cx, cy = self.normalize([x, y])

        res[0] = -cy*pot
        res[1] = cx*pot
        res[2] = cx*pot

        return res
    
    #Estimating objects
    def estimateObj(self, obj, time = None):
        if time == None:
            time = self.step
            
        objest = obj
        objest.posx += objest.velx*time
        objest.posy += objest.vely*time
        return objest
        
    def estimateRock(self, obj, time = None):
        if time == None:
            time = self.step
            
        objest = obj
        objest.posx += objest.velx*time
        objest.posy += objest.vely*time
        return objest
        
    def estimateShip(self, obj, time = None):
        if time == None:
            time = self.step
            
        objest = obj
        objest.posx += objest.velx*time
        objest.posy += objest.vely*time
        objest.ang += objest.velang*time
        return objest
        
    def estimateLaser(self, obj, time = None):
        if time == None:
            time = self.step
            
        objest = obj
        objest.posx += objest.velx*time
        objest.posy += objest.vely*time
        objest.lifetime -= time
        return objest
        
    #Estimating Time of Collision
    #Returns -> Time(seconds) for collision of obj1 and obj2: MIN, MAX
    def toC(self, obj1, obj2, error_margin):
        A = obj1.posx
        a = obj1.velx
        B = obj2.posx
        b = obj2.velx
        C = obj1.posy
        c = obj1.vely
        D = obj2.posy
        d = obj2.vely
        R = obj1.radius + error_margin/2
        r = obj2.radius + error_margin/2
        
        Asq = A**2
        asq = a**2
        Bsq = B**2
        bsq = b**2
        Csq = C**2
        csq = c**2
        Dsq = D**2
        dsq = d**2
        Rsq = R**2
        rsq = r**2

        div = asq - 2*a*b + bsq + csq - 2*c*d + dsq
        delta = (-Asq*csq + 2*Asq*c*d - Asq*dsq + 2*A*B*csq - 4*A*B*c*d + 2*A*B*dsq + 2*A*C*a*c - 2*A*C*a*d - 2*A*C*b*c + 2*A*C*b*d - 2*A*D*a*c + 2*A*D*a*d + 2*A*D*b*c - 2*A*D*b*d - Bsq*csq + 2*Bsq*c*d - Bsq*dsq - 2*B*C*a*c + 2*B*C*a*d + 2*B*C*b*c - 2*B*C*b*d + 2*B*D*a*c - 2*B*D*a*d - 2*B*D*b*c + 2*B*D*b*d - Csq*asq + 2*Csq*a*b - Csq*bsq + 2*C*D*asq - 4*C*D*a*b + 2*C*D*bsq - Dsq*asq + 2*Dsq*a*b - Dsq*bsq + Rsq*asq - 2*Rsq*a*b + Rsq*bsq + Rsq*csq - 2*Rsq*c*d + Rsq*dsq + 2*R*asq*r - 4*R*a*b*r + 2*R*bsq*r + 2*R*csq*r - 4*R*c*d*r + 2*R*dsq*r + asq*rsq - 2*a*b*rsq + bsq*rsq + csq*rsq - 2*c*d*rsq + dsq*rsq)
        minusb = (-A*a + A*b + B*a - B*b - C*c + C*d + D*c - D*d)
        if div == 0 or delta < 0:
            return None
        else:
            res0 = (minusb - math.sqrt(delta))/(div)
            res1 = (minusb + math.sqrt(delta))/(div)
            return res0, res1
    
    #Predictive shooting of moving target
    #Returns -> Time(seconds) for shoot to reach target on line, coordinates x and y for the shoot to be 'centered'
    def predShoot(self, ship, target, speed, gamestate):
        tx = target.posx - ship.posx
        ty = target.posy - ship.posy
        tvx = target.velx - ship.velx
        tvy = target.vely - ship.vely
        a = tvx**2 + tvy**2 - speed**2
        b = 2*(tvx*tx + tvy * ty)
        c = tx**2 + ty**2

        
        r = self.solveQuad(a, b, c)
        
        if r == None:
            return None
        else:
            r0, r1 = r
        if r1 < 0 and r0 < 0:
            return None
        elif r0 < 0:
            coords = (target.posx + tvx*r1, target.posy + tvy*r1)
            return r1, coords
        else:
            coords = (target.posx + tvx*r0, target.posy + tvy*r0)
            return r0, coords

    target = None

    ok = False

    ltick = 0

    def process(self, gamestate):
        if not self.initialized:
            self.initialize(gamestate)
            return Action(0, .1, .1, 0)

        try:
            sgargs = gamestate.ships[self.target]
        except:
            self.target = None
            self.ok = False

        if len(gamestate.ships) > 1 and not self.ok:
            for i in gamestate.ships:
                if i is not self.uid:
                    self.ok = True
                    self.target = i
                    gamestate.log("Following ship " + str(i))
                    break
        
        s_ship = gamestate.ships[self.uid]

        zero = 0

        out = [0, 0, 0]
        avoid = [0, 0, 0]
        rotation_out = 0
        rot_mean = 0
        out_s = 0
        self.ltick = gamestate.tick

        #Targeting and shooting

        for ship_uid in gamestate.ships:
            if self.uid == ship_uid:
                continue
            ship = gamestate.ships[ship_uid]
            if self.dist(ship, s_ship) < self.dist(gamestate.ships[self.target], s_ship):
                self.target = ship_uid



        if(self.target is not None):
            targetp = self.estimateShip(gamestate.ships[self.target], self.step)
            shipp = self.estimateShip(s_ship, self.step)
            prediction0 = None
            prediction1 = None
            prediction2 = None
            shoot_type = 0
            min_time = 9999
            if shipp.charge >= 3:
                predictiont = self.predShoot(shipp, targetp, 75, gamestate)
                if predictiont is not None:
                    time, coords = predictiont
                    time += self.step
                    if time < .8:
                        prediction2 = predictiont

            if shipp.charge >= 2:
                predictiont = self.predShoot(shipp, targetp, 50, gamestate)
                if predictiont is not None:
                    time, coords = predictiont
                    time += self.step
                    if time < .6:
                        prediction1 = predictiont

            if shipp.charge >= 1:
                predictiont = self.predShoot(shipp, targetp, 25, gamestate)
                if predictiont is not None:
                    time, coords = predictiont
                    time += self.step
                    if time < .4:
                        prediction0 = predictiont
            time, coords = None, None

            if prediction2 is not None:
                time, coords = prediction2
                time += self.step

                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                    out_s = 3

            if prediction1 is not None:
                time, coords = prediction1
                time += self.step
                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                    out_s = 2

            if prediction0 is not None:
                time, coords = prediction0
                time += self.step
                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                    out_s = 1
            if time is not None:
                rotation_out += self.lookAt(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy ))
                rot_mean += 1
            else:
                rotation_out += self.lookAt(shipp, math.atan2(shipp.posx - targetp.posx,targetp.posy - shipp.posy ))

        #Avoidance code
        #Avoid rocks

        rock_repel_r = 15
        rock_repel_t = 5

        rock_less = 9999
        rock_less_uid = None

        for rock_uid in gamestate.rocks:
            rock = gamestate.rocks[rock_uid]
            dist = self.dist(s_ship, rock)
            final = [0, 0, 0]
            if dist <= rock_repel_r:
                tmp = self.accelerateTo(s_ship, 2*s_ship.posx - rock.posx, 2*s_ship.posy - rock.posy, math.sqrt((rock_repel_r-dist)/rock_repel_r))

                avoid[0] += tmp[0]
                avoid[1] += tmp[1]
                avoid[2] += tmp[2]
            toc = self.toC(rock, s_ship, .1)
            if not toc == None:
                if toc[0] > 0:
                    gamestate.log("Rock of uid " + str(rock_uid) + ": Will collide in " + ('%.2f' % toc[0]) + " seconds")
                    shp = self.estimateShip(s_ship, toc[0])
                    rck = self.estimateRock(rock, toc[0])
                    
                    if toc[0] <= rock_repel_t:
                        tmp = self.accelerateTo(shp, 2*shp.posx - rck.posx, 2*shp.posy - rck.posy, math.sqrt((rock_repel_t-toc[0])/rock_repel_t))
                        final[0] += tmp[0]
                        final[1] += tmp[1]
                        final[2] += tmp[2]
                        if rock_less > toc[0]:
                            rock_less = toc[0]
                            rock_less_uid = rock_uid
            out[0] += final[0]
            out[1] += final[1]
            out[2] += final[2]

        #Avoid lasers

        laser_repel_r = 15
        laser_repel_t = 3

        laser_less = 9999
        laser_less_uid = None

        for laser_uid in gamestate.lasers:
            laser = gamestate.lasers[laser_uid]
            dist = self.dist(s_ship, laser)
            final = [0, 0, 0]
            if dist <= laser_repel_r:
                tmp = self.accelerateTo(s_ship, 2*s_ship.posx - laser.posx, 2*s_ship.posy - laser.posy, math.sqrt((laser_repel_r-dist)/laser_repel_r))

                avoid[0] += tmp[0]
                avoid[1] += tmp[1]
                avoid[2] += tmp[2]
            toc = self.toC(laser, s_ship, .1)
            if not toc == None:
                if toc[0] > 0:
                    if toc[0] <= laser.lifetime:
                        gamestate.log("Shot of uid " + str(laser_uid) + " from " + str(laser.owner) + ": Will hit in " + ('%.2f' % toc[0]) + " seconds")
                        shp = self.estimateShip(s_ship, toc[0])
                        lsr = self.estimateLaser(laser, toc[0])
                        shipp = self.estimateShip(s_ship, self.step)
                        las = self.estimateLaser(laser, self.step)
                        prediction = self.predShoot(shipp, las, 75, gamestate)

                        if prediction is not None:
                            time, coords = prediction
                            time += self.step
                            gamestate.log(str())
                            if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                                out_s = 3
                        prediction = self.predShoot(shipp, las, 50, gamestate)

                        if prediction is not None:
                            time, coords = prediction
                            time += self.step

                            if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                                out_s = 2

                        prediction = self.predShoot(shipp, las, 25, gamestate)

                        if prediction is not None:
                            time, coords = prediction
                            time += self.step

                            if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < .1:
                                out_s = 1

                        if toc[0] <= laser_repel_t:
                            tmp = self.accelerateTo(s_ship, 2*shp.posx - lsr.posx, 2*shp.posy - lsr.posy, math.sqrt((laser_repel_t-toc[0])/laser_repel_t))

                            final[0] += tmp[0]
                            final[1] += tmp[1]
                            final[2] += tmp[2]
                            if laser_less > toc[0]:
                                laser_less = toc[0]
                                laser_less_uid = laser_uid
                    else:
                        gamestate.log("Shot of uid " + str(laser_uid) + " from " + str(laser.owner) + ": Will not hit. Just " + ('%.2f' % laser.lifetime) + " seconds remaining.")
            out[0] += final[0]
            out[1] += final[1]
            out[2] += final[2]

        #Try not to collide with the arena

        arenac = 1

        if math.sqrt(s_ship.posx**2 + s_ship.posy**2) > gamestate.arenaRadius - 5:
            tmp = self.accelerateTo(s_ship, 0, 0, (math.sqrt(s_ship.posx**2 + s_ship.posy**2) - (gamestate.arenaRadius - 5))/5)
            out[0] += tmp[0]*arenac
            out[1] += tmp[1]*arenac
            out[2] += tmp[2]*arenac
        #Stay at a distance from target
        attrcnt = .3

        if self.target is not None:
            target_r = 30
            dist = self.dist(s_ship, gamestate.ships[self.target])
            linpot = 0
            if target_r-dist is not zero:
                linpot = target_r/(dist - target_r)
            tmp = self.accelerateTo(s_ship, gamestate.ships[self.target].posx, gamestate.ships[self.target].posy, (linpot**8)*self.sign(linpot))
            tmp = self.normalize(tmp)
            mx = max(abs(tmp[0]), abs(tmp[1]), abs(tmp[2]))
            if mx != 0:
                mx = 1/mx
            avoid[0] += tmp[0]*mx*attrcnt
            avoid[1] += tmp[1]*mx*attrcnt
            avoid[2] += tmp[2]*mx*attrcnt

        #Keep track of ship headings/ships targeting self
        
        predeyesight = .5

        for ship_uid in gamestate.ships:
            if ship_uid is self.uid:
                continue
            ship = gamestate.ships[ship_uid]
            targetp = self.estimateShip(s_ship, self.step)
            shipp = self.estimateShip(ship, self.step)
            prediction = None
            shoot_type = 0
            if shipp.charge < 2 and shipp.charge >= 1:
                prediction0 = self.predShoot(shipp, targetp, 25, gamestate)
                prediction1 = None
                prediction2 = None
            elif shipp.charge < 3:
                prediction0 = self.predShoot(shipp, targetp, 25, gamestate)
                prediction1 = self.predShoot(shipp, targetp, 50, gamestate)
                prediction2 = None
            else:
                prediction0 = self.predShoot(shipp, targetp, 25, gamestate)
                prediction1 = self.predShoot(shipp, targetp, 50, gamestate)
                prediction2 = self.predShoot(shipp, targetp, 75, gamestate)
            if prediction2 is not None:
                time, coords = prediction2
                time += self.step
                laser = Laser(0)
                laser.lifetime = 3
                laser.owner = ship_uid
                laser.posx = shipp.posx
                laser.posy = shipp.posy
                laser.velx = shipp.velx + 75*math.sin(self.toRad(shipp.ang))
                laser.vely = shipp.posy + 75*math.cos(self.toRad(shipp.ang))
                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < 2:
                    if time < 1:
                        shp = self.estimateShip(s_ship, time)
                        lsr = self.estimateLaser(laser, time)
                        tmp = self.accelerateTo(s_ship, 2*shp.posx - lsr.posx, 2*shp.posy - lsr.posy, math.sqrt((laser_repel_t-time)/laser_repel_t))
                        avoid[0] += tmp[0]*predeyesight
                        avoid[1] += tmp[1]*predeyesight
                        avoid[2] += tmp[2]*predeyesight
                        gamestate.log("Ship " + str(ship_uid) + " is targeting at 75m/s...")
            elif prediction1 is not None:
                time, coords = prediction1
                time += self.step
                laser = Laser(0)
                laser.lifetime = 3
                laser.owner = ship_uid
                laser.posx = shipp.posx
                laser.posy = shipp.posy
                laser.velx = shipp.velx + 50*math.sin(self.toRad(shipp.ang))
                laser.vely = shipp.posy + 50*math.cos(self.toRad(shipp.ang))
                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < 2:
                    if time < 1:
                        shp = self.estimateShip(s_ship, time)
                        lsr = self.estimateLaser(laser, time)
                        tmp = self.accelerateTo(s_ship, 2*shp.posx - lsr.posx, 2*shp.posy - lsr.posy, math.sqrt((laser_repel_t-time)/laser_repel_t))
                        avoid[0] += tmp[0]*predeyesight
                        avoid[1] += tmp[1]*predeyesight
                        avoid[2] += tmp[2]*predeyesight
                        gamestate.log("Ship " + str(ship_uid) + " is targeting at 50m/s...")
            if prediction0 is not None:
                time, coords = prediction0
                time += self.step
                laser = Laser(0)
                laser.lifetime = 3
                laser.owner = ship_uid
                laser.posx = shipp.posx
                laser.posy = shipp.posy
                laser.velx = shipp.velx + 25*math.sin(self.toRad(shipp.ang))
                laser.vely = shipp.posy + 25*math.cos(self.toRad(shipp.ang))
                if abs(self.angDelta(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy))) < 2:
                    if time < 1:
                        shp = self.estimateShip(s_ship, time)
                        lsr = self.estimateLaser(laser, time)
                        tmp = self.accelerateTo(s_ship, 2*shp.posx - lsr.posx, 2*shp.posy - lsr.posy, math.sqrt((laser_repel_t-time)/laser_repel_t))
                        avoid[0] += tmp[0]*predeyesight
                        avoid[1] += tmp[1]*predeyesight
                        avoid[2] += tmp[2]*predeyesight
                        gamestate.log("Ship " + str(ship_uid) + " is targeting at 25m/s...")
        
        #apply rotations and final weight calculation

        peravd = 2
        out[0] += avoid[0]*peravd
        out[1] += avoid[1]*peravd
        out[2] += avoid[2]*peravd

        mx = 1

        #out = self.normalize(out)
        #mx = max(abs(out[0]), abs(out[1]), abs(out[2]))
        #if mx != 0:
        #    mx = 1/mx
        #mx = 1

        rotmulti = 1
        #out[0] = 0
        out[1] += rotation_out*rotmulti
        out[2] += -rotation_out*rotmulti

        #out_s = 0
        #out = [0, 0, 0]


        #virtual 'friction'

        '''kF = .5

        vel = [s_ship.posx-s_ship.velx, s_ship.posy-s_ship.vely]
        mvel = math.sqrt(s_ship.velx**2 + s_ship.vely**2)
        vel = self.normalize(vel)
        tmp = self.accelerateTo(s_ship, vel[0], vel[1], kF)
        out[0] += tmp[0]*(mvel/30)
        out[1] += tmp[1]*(mvel/30)
        out[2] += tmp[2]*(mvel/30)'''

        #Emergency overwrite - in case of iminent danger
        rotation_out = 0 
        if rock_less <= 1:
            out_s = 1
            gamestate.log("Overwriting controls: rock 1s of ID " + str(laser_less_uid))

            shipp = self.estimateShip(s_ship, self.step)
            targetp = self.estimateRock(gamestate.rocks[rock_less_uid], self.step)

            prediction = self.predShoot(shipp, targetp, 25, gamestate)
            if prediction is not None:
                time, coords = prediction
                rotation_out = self.lookAt(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy ))

        if rock_less <= .5:
            gamestate.log("Overwriting controls: rock .5 of ID " + str(rock_less_uid))
            shp = self.estimateShip(s_ship, rock_less)
            rck = self.estimateRock(gamestate.rocks[rock_less_uid], rock_less)

            out = self.accelerateTo(shp, 2*shp.posx - rck.posx, 2*shp.posy - rck.posy)
            out = self.normalize(out)
            out = self.invert(out)

            out[1] += rotation_out*rotmulti
            out[2] += -rotation_out*rotmulti

            mx = max(abs(out[0]), abs(out[1]), abs(out[2]))
            if mx != 0:
                mx = 1/mx
        if laser_less <= 1.5:
            out_s = 1
            gamestate.log("Overwriting controls: laser 1s of ID " + str(laser_less_uid))

            shipp = self.estimateShip(s_ship, self.step)
            targetp = self.estimateLaser(gamestate.lasers[laser_less_uid], self.step)

            prediction = self.predShoot(shipp, targetp, 25, gamestate)
            if prediction is not None:
                time, coords = prediction
                rotation_out = self.lookAt(shipp, math.atan2(shipp.posx - coords[0],coords[1] - shipp.posy ))

        if laser_less <= .5:
            gamestate.log("Overwriting controls: laser .5 of ID " + str(laser_less_uid))
            shp = self.estimateShip(s_ship, laser_less)
            lsr = self.estimateLaser(gamestate.lasers[laser_less_uid], laser_less)

            out = self.accelerateTo(s_ship, 2*shp.posx - lsr.posx, 2*shp.posy - lsr.posy)
            out = self.normalize(out)
            out = self.invert(out)

            #@out[0] = -out[0]

            out[1] += rotation_out*rotmulti
            out[2] += -rotation_out*rotmulti

            mx = max(abs(out[0]), abs(out[1]), abs(out[2]))
            if mx != 0:
                mx = 1/mx

        return Action(-out[0]*mx, out[1]*mx, out[2]*mx, out_s)
        gamestate.log(str(s_ship.vely))
        return Action(1, 0, 0, 0)

GameState(SeijiBot()).connect()
