from robot import *

def main():
    #----User Changes-------------

    #--------Main param-----------
    DH_parameters =[
        [0,0,0,0],
        [0,0,0,1],
        [0,1,np.pi/2,1],
        [0,0,np.pi/2,0]
    ]
    
    conf = 'rrrp'

    ##---Secondary param----------
    prismaticLenght = 1 
    lim_display = 2

    #---------------------------------
    robot = ArmManipulator(DH_parameters, conf, prismaticLenght)
    ArmVisualizer(robot, lim_display)


if __name__ == '__main__':
    main()

