import numpy as np

from pyrevolve.SDF.math import Vector3
from pyrevolve.util import Time
import math
import sys
import os
import errno


def _make_path(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

class BehaviouralMeasurements:
    """
        Calculates all the measurements and saves them in one object
    """
    def __init__(self, robot_manager = None, robot = None):
        """
        :param robot_manager: Revolve Manager that holds the life of the robot
        :param robot: Revolve Bot for measurements relative to the robot morphology and brain
        :type robot: RevolveBot
        """
        if robot_manager is not None and robot is not None:
            self.velocity = velocity(robot_manager)
            self.displacement = displacement(robot_manager)
            self.displacement_velocity = displacement_velocity(robot_manager)
            self.displacement_velocity_hill = displacement_velocity_hill(robot_manager)
            self.head_balance = head_balance(robot_manager)
            self.contacts = contacts(robot_manager, robot)
            self.GaitAngleErr = GaitAngleError(robot_manager)
            self.Avg_step_size = step_size_avg(robot_manager, robot)
            self.Sum_area = traj_area_measure(robot_manager)
            self.Mean_head_deviation = mean_head_deviation(robot_manager)
            self.Disp_sum = disp_sum(robot_manager)
            self.x_axis_displacement = X_axis_displacement(robot_manager)
            self.Disp_sum_avg = disp_sum_avg(robot_manager)
            self.gaitAngleErrorCumulative = GaitAngleErrorCumulative(robot_manager)
            self.effectiveMovement = EffectiveMovement(robot_manager)
            self.difFromIdealMovement = DifFromIdealMovement(robot_manager)
            self.difFromIdealMovementX = DifFromIdealMovementX(robot_manager)
            self.difFromIdealMovementY = DifFromIdealMovementY(robot_manager)
            self.standHeight = standingHeight(robot_manager)


            logs_position_orientation(robot_manager, robot_manager.conf.evaluation_time, robot.id,('experiments/'+robot_manager.conf.experiment_name))

        else:
            self.velocity = None
            self.displacement = None
            self.displacement_velocity = None
            self.displacement_velocity_hill = None
            self.head_balance = None
            self.contacts = None
            self.GaitAngleErr = None
            self.Avg_step_size = None
            self.Sum_area = None
            self.Mean_head_deviation = None
            self.Disp_sum = None
            self.x_axis_displacement = None
            self.Disp_sum_avg = None
            self.gaitAngleErrorCumulative = None
            self.effectiveMovement = None
            self.difFromIdealMovement = None
            self.difFromIdealMovementX = None
            self.difFromIdealMovementY = None
            self.standHeight = None

    def items(self):
        return {
            'velocity': self.velocity,
            #'displacement': self.displacement,
            'displacement_velocity': self.displacement_velocity,
            'displacement_velocity_hill': self.displacement_velocity_hill,
            'head_balance': self.head_balance,
            'contacts': self.contacts,
            'gaitAngleErr': self.GaitAngleErr,
            'Avgstepsize': self.Avg_step_size,
            'SumArea': self.Sum_area,
            'MeanHeadDeviation': self.Mean_head_deviation,
            'DisplacementSum': self.Disp_sum,
            'x_axis_displacement' : self.x_axis_displacement,
            'displacement_full_avg': self.Disp_sum_avg,
            'gaitAngleErrorCumulative': self.gaitAngleErrorCumulative,
            'EffectiveMovement': self.effectiveMovement,
            'DifFromIdealMovement' : self.difFromIdealMovement,
            'DifFromIdealMovementX': self.difFromIdealMovementX,
            'DifFromIdealMovementY': self.difFromIdealMovementY,
            'StandingHeight': self.standHeight

        }.items()



def velocity(robot_manager):
    """
    Returns the velocity over the maintained window
    :return:
    """
    return robot_manager._dist / robot_manager._time if robot_manager._time > 0 else 0


def displacement(robot_manager):
    """
    Returns a tuple of the displacement in both time and space
    between the first and last registered element in the speed
    window.
    :return: Tuple where the first item is a displacement vector
             and the second a `Time` instance.
    :rtype: tuple(Vector3, Time)
    """
    if len(robot_manager._positions) == 0:
        return Vector3(0, 0, 0), Time()
    return (
        robot_manager._positions[-1] - robot_manager._positions[0],
        robot_manager._times[-1] - robot_manager._times[0]
    )


def path_length(robot_manager):
    return robot_manager._dist


def displacement_velocity(robot_manager):
    """
    Returns the displacement velocity, i.e. the velocity
    between the first and last recorded position of the
    robot in the speed window over a straight line,
    ignoring the path that was taken.
    :return:
    """
    dist, time = displacement(robot_manager)
    if time.is_zero():
        return 0.0
    return np.sqrt(dist.x ** 2 + dist.y ** 2) / float(time)


def displacement_velocity_hill(robot_manager):
    dist, time = displacement(robot_manager)
    if time.is_zero():
        return 0.0
    return dist.y / float(time)


def head_balance(robot_manager):
    """
    Returns the average rotation of teh head in the roll and pitch dimensions.
    :return:
    """
    roll = 0
    pitch = 0
    instants = len(robot_manager._orientations)
    for o in robot_manager._orientations:
        roll = roll + abs(o[0]) * 180 / math.pi
        pitch = pitch + abs(o[1]) * 180 / math.pi
    #  accumulated angles for each type of rotation
    #  divided by iterations * maximum angle * each type of rotation
    if instants == 0:
        balance = None
    else:
        balance = (roll + pitch) / (instants * 180 * 2)
        # turns imbalance to balance
        balance = 1 - balance
    return balance


def contacts(robot_manager, robot):
    """
    Measures the average number of contacts with the floor relative to the body size

    WARN: this measurement could be faulty, several robots were
    found to have 0 contacts if simulation is too fast

    :param robot_manager: reference to the robot in simulation
    :param robot: reference to the robot for size measurement
    :return: average number of contacts per block in the lifetime
    """
    avg_contacts = 0
    for c in robot_manager._contacts:
        cc = c[1]
        avg_contacts += cc
    avg_contacts = (avg_contacts/(robot_manager._contacts.__len__()+1)) / robot.phenotype._morphological_measurements.measurements_to_dict()['absolute_size']
    if avg_contacts==0:
        print("fuck")
    return avg_contacts


def logs_position_orientation(robot_manager, evaluation_time, robotid, path):
    if robot_manager.second <= evaluation_time:
        _path = os.path.join(path, 'data_fullevolution/descriptors')
        _make_path(_path)
        _file_path = os.path.join(_path, f'positions_{robotid}.txt')
        with open(_file_path, 'a') as f:
            for o in range(len(robot_manager._positions)):
                f.write(str(robot_manager.second) + ' '
                        + str(o) + ' '
                        + str(robot_manager._orientations[o][0]) + ' '
                        + str(robot_manager._orientations[o][1]) + ' '
                        + str(robot_manager._orientations[o][2]) + ' '
                        + str(robot_manager._positions[o].x) + ' '
                        + str(robot_manager._positions[o].y) + ' '
                        + str(robot_manager._positions[o].z) + '\n')



def  GaitAngleError(robot_manager):
    """
    Returns the the angle between moving directly and what happened during evaluation
    :return:
    """
    dist, time = displacement(robot_manager)
    if time.is_zero():
        return 0.0
    distz = math.sqrt(((dist.y)**2)+((dist.x)**2))
    tetha = math.degrees(math.acos(((dist.y**2)+(distz**2)-(dist.x**2))/(2*dist.y*distz)))
    return tetha

def step_size_avg(robot_manager, robot):
    dist = 0
    counter = 0
    for c in robot_manager._contacts_positions:
        y = c.pop()
        if(len(y)>1):
            for i in range(len(y)):
                if(i != 0):
                    dist += calc_dist(y[i], y[i-1])
                    counter += 1
    if(counter == 0):
        return 0
    else:
        return float(dist/counter)

def calc_dist(a,b):
    if (a==None or b == None):
        return 0
    else:
        dist = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        return float(dist)

def proj_point_line(p1,p2,p3):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    # distance between p1 and p2
    l2 = np.sum((p1 - p2) ** 2)
    if l2 == 0:
        print('p1 and p2 are the same points')

    # The line extending the segment is parameterized as p1 + t (p2 - p1).
    # The projection falls where t = [(p3-p1) . (p2-p1)] / |p2-p1|^2

    # if you need the point to project on line extention connecting p1 and p2
    t = np.sum((p3 - p1) * (p2 - p1)) / l2

    # if you need to ignore if p3 does not project onto line segment
    if t > 1 or t < 0:
        print('p3 does not project onto p1-p2 line segment')

    # if you need the point to project on line segment between p1 and p2 or closest point of the line segment
    t = max(0, min(1, np.sum((p3 - p1) * (p2 - p1)) / l2))

    projection = p1 + t * (p2 - p1)
    return projection

def calc_area(a,b,c,d):#has bug - when line between two position has a intersect with base line
    a = np.array(a)
    b = np.array(b)
    if find_intersection(a, b, c, d) is None:
        ac = math.sqrt((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2)
        bd = math.sqrt((b[0] - d[0]) ** 2 + (b[1] - d[1]) ** 2)
        cd = math.sqrt((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2)
        area = ((ac+bd)*cd)/2
        sign = choose_side(c,d,a)
        #print("First")
    else:
        intersect = find_intersection(a, b, c, d)
        ac = math.sqrt((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2)
        bd = math.sqrt((b[0] - d[0]) ** 2 + (b[1] - d[1]) ** 2)
        c_intersect = math.sqrt((c[0] - intersect[0]) ** 2 + (c[1] - intersect[1]) ** 2)
        intersect_d = math.sqrt((intersect[0] - d[0]) ** 2 + (intersect[1] - d[1]) ** 2)
        area_1 = (ac * c_intersect)/2
        area_2 = (bd * intersect_d) / 2
        area = abs(area_2-area_1)
        sign = "match"
        #print(ac,"---",bd,"---",area_1,"---",area_2,"---","Second")

    return [area,sign]

def traj_area_measure(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c!= 0):
            a = robot_manager._positions[c-1]
            b = robot_manager._positions[c]
            c = proj_point_line(np.array(robot_manager._positions[0]),np.array(robot_manager._positions[-1]),a)
            d = proj_point_line(np.array(robot_manager._positions[0]), np.array(robot_manager._positions[-1]), b)
            area = calc_area(a,b,c,d)
            #if(area[1]=="side1" or area[1]=="match"):
            sum = sum + area[0]
            #else:
            #    sum = sum - area[0]
    dist = calc_dist(robot_manager._positions[0],robot_manager._positions[-1])
    return sum/dist

def mean_head_deviation(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c!= 0):
            a = robot_manager._positions[c-1][2]
            b = robot_manager._positions[c][2]

            sum = sum + abs(b-a)
    return sum/(robot_manager._positions.__len__())

def find_intersection( p0, p1, p2, p3 ) :

    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y

    if denom == 0 : return None # collinear

    denom_is_positive = denom > 0

    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]

    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision

    t_numer = s32_x * s02_y - s32_y * s02_x

    if (t_numer < 0) == denom_is_positive : return None # no collision

    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision


    # collision detected

    t = t_numer / denom

    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]
    return intersection_point

def choose_side(a,b,c):
    if (b[0]-a[0] == 0):
        if c[0]<a[0]:
            return "side1"
        elif c[0]==a[0]:
            return "match"
        else:
            return "match"

    else:
        tilt = (b[1]-a[1])/(b[0]-a[0])
        bias = a[1]-tilt*a[0]
        y = tilt*c[0]+bias
        if c[1]>y:
            #print(c[1],y)
            return "side1"
        elif c[1]<y:
            #print(c[1],y)
            return "side2"
        else:
            #print(c[1],y)
            return "match"

def disp_sum(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c != 0):
            xa = robot_manager._positions[c - 1][0]
            xb = robot_manager._positions[c][0]
            ya = robot_manager._positions[c - 1][1]
            yb = robot_manager._positions[c][1]

            dis = calc_dist(robot_manager._positions[c],robot_manager._positions[c - 1])

            sum = sum + dis
    return sum


def X_axis_displacement(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c!= 0):
            a = robot_manager._positions[c-1][0]
            b = robot_manager._positions[c][0]

            sum = sum + (b-a)
    return sum

def disp_sum_avg(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c != 0):
            xa = robot_manager._positions[c - 1][0]
            xb = robot_manager._positions[c][0]
            ya = robot_manager._positions[c - 1][1]
            yb = robot_manager._positions[c][1]

            dis = calc_dist(robot_manager._positions[c],robot_manager._positions[c - 1])

            sum = sum + dis
    return sum/robot_manager._positions.__len__()

def  GaitAngleErrorCumulative(robot_manager):
    """
    Returns the the angle between moving directly and what happened during evaluation
    :return:
    """
    sum = 0
    for c in range(robot_manager._positions.__len__()):
        if (c != 0):
            dist = robot_manager._positions[c]-robot_manager._positions[c-1]
            distz = math.sqrt(((dist.y)**2)+((dist.x)**2))
            tetha = math.degrees(math.acos(((dist.y**2)+(distz**2)-(dist.x**2))/(2*dist.y*distz)))
            sum = sum + tetha
    return tetha

def EffectiveMovement(robot_manager):
    x = math.sqrt((displacement(robot_manager)[0][0]**2)+(displacement(robot_manager)[0][1]**2)+(displacement(robot_manager)[0][2]**2))
    return x/disp_sum(robot_manager)

def DifFromIdealMovement(robot_manager):
    sum = 0
    disp = displacement(robot_manager)
    disp_size = math.sqrt((disp[0][0]**2)+disp[0][1]**2)
    NumOfstepSizeIdealParts = robot_manager._positions.__len__()
    for i in range(NumOfstepSizeIdealParts):
        m = i
        n = NumOfstepSizeIdealParts - i
        idealPos = (m/(m+n))*(robot_manager._positions[0]) + (n/(m+n))*(robot_manager._positions[-1])
        actualPos = robot_manager._positions[i]
        difference = actualPos - idealPos
        difference_dist = math.sqrt(difference[0]**2+difference[1]**2)
        sum = sum+difference_dist
    return sum

def DifFromIdealMovementX(robot_manager):
    sum = 0
    disp = displacement(robot_manager)
    disp_size = math.sqrt((disp[0][0]**2)+disp[0][1]**2)
    NumOfstepSizeIdealParts = robot_manager._positions.__len__()
    for i in range(NumOfstepSizeIdealParts):
        m = i
        n = NumOfstepSizeIdealParts - i
        idealPos = (m/(m+n))*(robot_manager._positions[0]) + (n/(m+n))*(robot_manager._positions[-1])
        actualPos = robot_manager._positions[i]
        difference = actualPos - idealPos
        difference_dist = abs(difference[0])
        sum = sum+difference_dist
    return sum

def DifFromIdealMovementY(robot_manager):
    sum = 0
    disp = displacement(robot_manager)
    disp_size = math.sqrt((disp[0][0]**2)+disp[0][1]**2)
    NumOfstepSizeIdealParts = robot_manager._positions.__len__()
    for i in range(NumOfstepSizeIdealParts):
        m = i
        n = NumOfstepSizeIdealParts - i
        idealPos = (m/(m+n))*(robot_manager._positions[0]) + (n/(m+n))*(robot_manager._positions[-1])
        actualPos = robot_manager._positions[i]
        difference = actualPos - idealPos
        difference_dist = abs(difference[1])
        sum = sum+difference_dist
    return sum
def standingHeight(robot_manager):
    sum = 0
    for c in range(robot_manager._positions.__len__()):
            a = robot_manager._positions[c][2]
            sum = sum + a
    return sum / (robot_manager._positions.__len__())
