import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider,RadioButtons,CheckButtons, TextBox
import sys
import gradient_descent as inv_kin

class ArmManipulator():
    def __init__(self, DH_parameters, configuration=None,prismaticLenght=None):
        self.DH_parameters = DH_parameters 
        self.last_solution = None #last solution of inverse kinematics 
        #---Configuration
        if configuration is None:
            self.configuration = 'r' * len(self.DH_parameters) #only rotative
        else:
            self.configuration = configuration
           
        
        #----Prismatic Lenght
        if prismaticLenght is None:
            self.prismaticLenght = 1 
        else:
            self.prismaticLenght = prismaticLenght
            
        #error detection 
        for conf in self.configuration:
            if (conf != 'r' and conf != 'p'):
                print("Invalid configuration, can only contain 'p' and 'r' !! :'( ")
        
        if (len(self.configuration) < len(self.DH_parameters)):
            print("Error : more link than configuration")
            sys.exit()
        elif (len(self.configuration) > len(self.DH_parameters)):
            print("Error : more configuration than link")
            sys.exit()
        
    def DH_matrix(self,theta,d,alpha,a):
        return np.array([
            [np.cos(theta)               , -np.sin(theta)               , 0              , a],
            [np.cos(alpha)*np.sin(theta) , np.cos(alpha)*np.cos(theta)  , -np.sin(alpha) , 0],
            [np.sin(alpha)*np.sin(theta) , np.sin(alpha)*np.cos(theta)  , np.cos(alpha)  , d],
            [0            , 0                           , 0                            , 1]
        ])
    
    
    def calculate_origin_all_frame(self,DH_parameters):
        origins_points = [np.zeros(3)] #origin fix frame
        x_axes = [np.array([1,0,0])] #fix frame
        y_axes = [np.array([0,1,0])] #fix frame
        z_axes = [np.array([0,0,1])] #fix frame
        
        T = np.eye(4) # I 
     
        for parameters in DH_parameters:
            T = T @ self.DH_matrix(*parameters)
            x_axes.append(T[:3,0])
            y_axes.append(T[:3,1])
            z_axes.append(T[:3,2])
            origins_points.append(T[:3,3])
    

        origins_points.append((T@ np.array([[1,0,0,1]]).transpose()).flatten()[:3])
        return np.array(origins_points),np.array(x_axes), np.array(y_axes), np.array(z_axes)

class ArmVisualizer():
    def __init__(self,robot,lim=2):
        self.robot = robot 
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111,projection="3d")
        #ax limit
        self.lim=lim
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-lim, lim)
        self.ax.set_title('Bras Robotique 3R')
        #----
        self.line, = self.ax.plot([], [], [], 'o-', color='blue')
        self.quiver_list = []
        
        self.sliders = []

        
        self.mode = 'Directe'
        #Initialisation des sliders et boutons 
        self.init_sliders()
        #self.init_sliders_i()
        self.radio = self.init_radio_buttons()
  
        

        self.update()
        plt.show()
        
    
    def init_sliders(self):
        DH_parameters = self.robot.DH_parameters
        #Ajout des sliders
        axcolor = 'lightgoldenrodyellow'
        for i,conf in enumerate(self.robot.configuration):
            slider_ax = plt.axes([0.2, 0.01 + i* 0.03, 0.65, 0.02], facecolor=axcolor)
            if (conf == 'r'):
                theta = self.robot.DH_parameters[i][0]
                slider = Slider(slider_ax, f'Theta{i}', -180, 180, valinit=theta*180/np.pi)
            else:
                a = self.robot.DH_parameters[i][3]
                lenght = self.robot.prismaticLenght
                slider = Slider(slider_ax, f'a{i}' , -lenght, lenght, valinit=a)
            
            slider.on_changed(self.update)
            self.sliders.append(slider)
            
            
            
        if len(self.robot.configuration) < 3:
            for i in range(len(self.robot.configuration),3):
                slider_ax = plt.axes([0.2, 0.01 + i* 0.03, 0.65, 0.02], facecolor=axcolor)
                slider =  Slider(slider_ax, '', -180, 180, valinit=0)
                slider.ax.set_visible(False)
                slider.on_changed(self.update)
                self.sliders.append(slider)
   
        
    def init_radio_buttons(self): 
        axcolor = 'lightgoldenrodyellow'
        radio_ax = plt.axes([0.02, 0.7, 0.15, 0.15], facecolor=axcolor)
        radio = RadioButtons(radio_ax, ('Directe', 'Inverse'))
        radio.on_clicked(self.update_mode)
        
        return radio
    
    def add_quiver(self,points, x_axes,y_axes, z_axes):
    
        for quiver in self.quiver_list:
            quiver.remove()
        
        self.quiver_list=[]
        for i in range(len(points)-1):
            origin = points[i]
            x_axis = x_axes[i]
            y_axis = y_axes[i]
            z_axis = z_axes[i]
            quiver_z = self.ax.quiver(
                origin[0], origin[1], origin[2],  # Origine
                z_axis[0], z_axis[1], z_axis[2],  # Composantes du vecteur
                length=0.5, color='r', normalize=True
            )
            quiver_x = self.ax.quiver(
                origin[0], origin[1], origin[2],  # Origine
                x_axis[0], x_axis[1], x_axis[2],  # Composantes du vecteur
                length=0.5, color='y', normalize=True
            )
            quiver_y = self.ax.quiver(
                origin[0], origin[1], origin[2],  # Origine
                y_axis[0], y_axis[1], y_axis[2],  # Composantes du vecteur
                length=0.5, color='g', normalize=True
            )
            self.quiver_list.append(quiver_x)
            self.quiver_list.append(quiver_y)
            self.quiver_list.append(quiver_z)
    
    def update(self, val=None):
        if self.mode=='Directe':
            param = [slider.val for slider in self.sliders]
        
        else:
            target = [self.sliders[i].val for i in range(3)]
            param =  inv_kin.gradient_descent( inv_kin.f, len(self.robot.DH_parameters), robot = self.robot, target = target)
            
            if param is None:
                return
                
            mask = np.array([char=='r' for char in self.robot.configuration])
            param[mask] =  param[mask] % (2*np.pi)
            param[((param>np.pi) & mask)] = param[((param>np.pi) & mask)] - 2*np.pi
            
        configuration = self.robot.configuration    
        for i,DH_link in enumerate(self.robot.DH_parameters):
            if configuration[i] == 'r':
                if self.mode == 'Directe':
                    DH_link[0] = np.radians(param[i])
                else:
                    DH_link[0] = param[i]    
            else:
                DH_link[3] = param[i]
        
        points,x_axes, y_axes, z_axes = self.robot.calculate_origin_all_frame(self.robot.DH_parameters)  
        self.add_quiver(points, x_axes, y_axes, z_axes)
        self.line.set_data(points[:, 0], points[:, 1])
        self.line.set_3d_properties(points[:, 2])
        self.fig.canvas.draw_idle()
        
    def update_mode(self,label):
        
        self.mode = label
        
       
            
        if label == "Directe":
            for i,slider in enumerate(self.sliders):
                if i < len(self.robot.configuration):
                    slider.ax.set_visible(True)
                    if self.robot.configuration[i] == 'r':
                        slider.label.set_text(f'Theta{i}')
                        slider.set_val(0)
                        slider.ax.set_xlim(-180,180)
                        slider.valmin, slider.valmax = -180,180
                    else:
                        slider.label.set_text(f'a{i}')
                        slider.set_val(0)
                        l = self.robot.prismaticLenght
                        slider.ax.set_xlim(-l,l)
                        slider.valmin, slider.valmax = -l,l
                
            if len(self.robot.configuration) < 3:
                for i in range(len(self.robot.configuration),3):
                    slider = self.sliders[i]
                    slider.ax.set_visible(False)
                

        else:
            s =['x :','y :','z :']
            for i in range(3):
                slider = self.sliders[i]
                slider.label.set_text(s[i])
                slider.set_val(0)
                slider.ax.set_visible(True)
                slider.ax.set_xlim(-self.lim,self.lim)
                slider.valmin, slider.valmax = -self.lim,self.lim
            
            for i in range(3,len(self.robot.configuration)):
                slider = self.sliders[i]
                slider.ax.set_visible(False)
        self.update()





