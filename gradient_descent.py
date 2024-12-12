import numpy as np

    
def f(x, robot):   
    
    DH_parameters = list(robot.DH_parameters) #copy 
    configuration = list(robot.configuration) #copy
    for i,DH_link in enumerate(DH_parameters):
        if configuration[i] == 'r':
            DH_link[0] = x[i] 
        else:
            DH_link[3] = x[i]
    T = np.eye(4) # I 
    for parameters in DH_parameters:  
        T = T @ robot.DH_matrix(*parameters)
    result = T @ np.array([[1,0,0,1]]).transpose()
    return result.flatten()[:3]
   

def jacobian(function, point, robot, epsilon=1e-6):

    n = len(point)
    m = len(function(point, robot))
    
    J = np.zeros((m,n))
    
    for i in range(n):
        #little perturbation
        h = np.zeros(n)
        h[i] = epsilon
        
        #evaluate
        f_plus  = function(point+h, robot)
        f_minus = function(point-h, robot)
        J[:,i] = (f_plus-f_minus)/(2*epsilon)
    
    return J
   

def gradient_descent(f, nbrParam, target=0, robot=None, tol=1e-2,max_iter=1000,alpha=0.1):
    
    conf_mask = np.array([char=='p' for char in robot.configuration])
    if robot.last_solution is None:
        xi = 5*(np.random.rand(nbrParam) - 0.5)
    else:
        xi = robot.last_solution #search a solution near our actual solution

    for i in range(max_iter):
        #copy of xi 
        yi = xi.copy()
        
        yi[conf_mask] = robot.prismaticLenght * np.tanh(xi[conf_mask]) #tanh transformation
        #jacobian of tanh transformation 
        diag_elements = np.ones_like(xi)  # Par défaut, dérivée = 1
        diag_elements[conf_mask] = robot.prismaticLenght * (1 - np.tanh(xi[conf_mask])**2)
        grad_t  = np.diag(diag_elements)
        
        #jacobian of f
        J_transpose = jacobian(f, yi, robot).T
        g = f(yi, robot) - target 
        grad_g = 2 * J_transpose @ g
        
        #grad 
        grad = grad_t @ grad_g 
        
        #mise a jour gradient
        xi -= alpha*grad
        #xi[conf_mask] = np.clip(xi[conf_mask],-robot.prismaticLenght,+robot.prismaticLenght)
        #print(np.linalg.norm(g))
        if (np.linalg.norm(g) < tol):
            robot.last_solution = yi
            return yi

    print(f"No solution found for {target} near start point {xi}")
    return None
