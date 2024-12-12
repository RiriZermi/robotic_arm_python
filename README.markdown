# Robotic Arm Project - Visualization and Kinematics

This project simulates and visualizes a 3-axis robotic arm with rotational joints. It provides an interactive interface to explore forward and inverse kinematics concepts.

## Key Features

- **3D Visualization**: Displays the robotic arm with local axes for each joint.
- **Forward Kinematics**: Adjust joint angles to observe the arm's movement.
- **Inverse Kinematics**: Define target coordinates to calculate joint positions.
- **Interactive Interface**: Use sliders and radio buttons to manipulate the arm in real-time.

## Project Structure

1. **`ArmManipulator`**: Handles calculations and transformations related to the robotic arm using DH (Denavit-Hartenberg) parameters.
2. **`ArmVisualizer`**: Graphical interface for visualizing and controlling the robotic arm.
3. **External Modules**:
   - `numpy`: For matrix computations.
   - `matplotlib`: For 3D visualization and user interface.
## DH parameters revisited
### Custom DH Convention with x-axis Pointing to Link (i+1)

This document outlines a custom Denavit-Hartenberg (DH) convention where the \(x_i\) axis points toward the next link \(i+1\). This approach differs from the standard DH convention and requires specific adjustments to the transformation matrix.

---

#### **1. Axis Definitions:**
- **\(z_i\) Axis:** Defined as the axis of rotation or translation for joint \(i\).
- **\(x_i\) Axis:** Points toward the next link \(i+1\), connecting the \(z_i\) axis of joint \(i\) to the \(z_{i+1}\) axis of joint \(i+1\).
- **\(y_i\) Axis:** Defined by the right-hand rule to form an orthonormal coordinate system.

---

#### **2. Parameters:**
The parameters in this convention are defined as:
- **\(a_i\):** The distance between the \(z_i\) and \(z_{i+1}\) axes along the \(x_i\) axis.
- **\(\alpha_i\):** The angle between the \(z_i\) and \(z_{i+1}\) axes, measured about the \(x_i\) axis.
- **\(d_i\):** The offset along the \(z_i\) axis between the origin of frame \(i\) and its intersection with the \(x_i\) axis.
- **\(\theta_i\):** The angle of rotation around the \(z_i\) axis, defining the relative orientation of frame \(i+1\).

---

#### **3. Transformation Matrix:**
Given this convention, the homogeneous transformation matrix from frame \(i\) to frame \(i+1\) is defined as:

\[
T_i^{i+1} =
\begin{bmatrix}
\cos(\theta) & -\sin(\theta) & 0 & a \\
\cos(\alpha)\sin(\theta) & \cos(\alpha)\cos(\theta) & -\sin(\alpha) & 0 \\
\sin(\alpha)\sin(\theta) & \sin(\alpha)\cos(\theta) & \cos(\alpha) & d \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

## Parameters to modify
You need to modify two parameters in the main file (there is already a default setting) : 

### DH Parameters


Each row in the `DH_parameters` array represents the Denavit-Hartenberg parameters for a joint in the manipulator. These parameters define the relative transformations between consecutive links and joints. 

The parameters in each row are:
- `theta` (rotation angle around the z-axis)
- `d` (translation along the z-axis)
- `a` (translation along the x-axis)
- `alpha` (rotation around the x-axis)

### Configuration String

conf = 'rrrp'

The string `conf` defines the configuration of the joints:
- `'r'` stands for a **rotational joint** (rotates around the z-axis).
- `'p'` stands for a **prismatic joint** (translates along the z-axis).


### Example

The code uses Denavit-Hartenberg revisited (DH) parameters to define the kinematic structure of a robotic manipulator. 

#### DH Parameters
Here the code :
```python
DH_parameters =[
        [0,0,0,0],
        [0,0,0,1],
        [0,1,np.pi/2,1],
        [0,0,0,0]
    ]
```
The table below shows the current Denavit-Hartenberg parameters for each joint:

| Joint | `theta` (Rotation around z-axis) | `d` (Translation along z-axis) | `alpha` (Rotation around x-axis) |`a` (Translation along x-axis) |
|-------|----------------------------------|--------------------------------|--------------------------------|----------------------------------|
| 0     | 0  (variable)                               | 0                              | 0                              | 0                                |
| 1     | 0  (variable)                              | 0                              | 0                              | 1                                |
| 2     | 0  (variable)                              | 1                              | π/2                             | 1                                |
| 3     | 0                                | 0  (variable)                            | 0                              | 0                                 |

Each row in the `DH_parameters` array represents the Denavit-Hartenberg parameters for a joint in the manipulator. These parameters define the relative transformations between consecutive links and joints.

- `theta`: Rotation angle around the z-axis.
- `d`: Translation along the z-axis.
- `a`: Translation along the x-axis.
- `alpha`: Rotation around the x-axis.

#### Configuration String

```python
conf = 'rrrp'
```
It means that the first we join are rotative joint, and the last is a prismatic joint.
#### Example picture
![Example](https://i.imgur.com/h2Ii9ad.png)
## Forward and Inverse Kinematics
You can use forward kinematics to control parameters directly or use the inverse kinematics to control the robot with the coordinates. The inverse kinematics is compute with a gradient descent who sometimes don't converge.

## Secondary Param
- prismaticLenght : fix the min-max value where the prismatic join can moove
- lim_display : show the coordinates from -lim_display to +lim_display
## Run 
```bash
python main.py
```