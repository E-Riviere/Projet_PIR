import math


class Command():
    def __init__(self):
        pass
    #Action de la commande
    def FollowTheGap(self, gap_analysis, gap_sel):

        forward = 0.0
        lateral = 0.0
        rotation = 0.0 

        forward = 0.5

        if gap_analysis[1][gap_sel] > 0:
            rotation = (abs(gap_analysis[1][gap_sel])/math.pi) 
            lateral = 0.1

        if gap_analysis[1][gap_sel] < 0:
            rotation = -(abs(gap_analysis[1][gap_sel])/math.pi)
            lateral = -0.1

        else:
            pass

        return((forward, lateral, rotation))


    def CenterInIntersection(self, NegativeDetection):
        Obst_dist_ray = NegativeDetection[1]
        (forward, lateral, rotation) = (0.0, 0.0, 0.0)
        if Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] < -0.5:
            forward = 0.1
        elif Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] > 0.5:
            forward = -0.1

        if Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] < - 1:
            lateral = 0.1
        elif Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] < 1:
            lateral = -0.1

        return((forward, lateral, rotation))
        

    def AlignWithTheGap(self, gap_analysis, gap_selected):
        direction = gap_analysis[1][gap_selected]

        forward = 0.0
        lateral = 0.0
        rotation = 0.0

        if direction > 0.05:
            rotation = 0.2

        if direction < -0.05:
            rotation = - 0.2
        
        return((forward, lateral, rotation))
    

    def RushInTheGap(self):

        forward = 0.5
        lateral = 0.0
        rotation = 0.0

        return ((forward, lateral, rotation))
    
    def FollowTheExplorator(self, SEMANTICProcess, setpoint, follower):

        forward = 0.0
        lateral = 0.0
        rotation = 0.0
        if setpoint - SEMANTICProcess[0][0].distance > 0:
            self.speed_control = -follower(SEMANTICProcess[0][0].distance)
                
        elif setpoint - SEMANTICProcess[0][0].distance < 0:
            self.speed_control = -follower(SEMANTICProcess[0][0].distance)

        (forward, lateral, rotation) = (self.speed_control, 0.0, 0.0)
        return ((forward, lateral, rotation))