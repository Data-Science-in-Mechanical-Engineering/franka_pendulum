#!/usr/bin/freecadcmd
import FreeCAD
import numpy as np
import rospkg
import sys

class Part:
    def __init__(self):
        self.mass = 0
        self.position = np.zeos(3)
        self.inertia = np.zeos((3,3))

    def __str__(self):
        position = np.copy(self.position)
        position[np.where(np.abs(position) < 1e-9)] = 0.0
        inertia = np.copy(self.inertia)
        inertia[np.where(np.abs(inertia) < 1e-9)] = 0.0

        return (
            "<inertial>\n" +
            "  <origin xyz=\"" + str(position[0]) + " " + str(position[1]) + " " + str(position[2]) + "\" rpy=\"0 0 0\"/>\n" +
            "  <mass value=\"" + str(self.mass) + "\"/>\n" +
            "  <inertia\n" +
            "  ixx=\"" + str(inertia[0,0]) + "\" ixy=\"" + str(inertia[0,1]) + "\" ixz=\"" + str(inertia[0,2]) + "\"\n" +
            "  iyy=\"" + str(inertia[1,1]) + "\" iyz=\"" + str(inertia[1,2]) + "\"\n" +
            "  izz=\"" + str(inertia[2,2]) + "\"/>\n" +
            "</inertial>"
        )

    def inertia_at(self, position):
        inertia = np.copy(self.inertia)
        inertia[0,0] += self.mass * ((position[1] - self.position[1])**2 + (position[2] - self.position[2])**2)
        inertia[1,1] += self.mass * ((position[0] - self.position[0])**2 + (position[2] - self.position[2])**2)
        inertia[2,2] += self.mass * ((position[0] - self.position[0])**2 + (position[1] - self.position[1])**2)
        inertia[0,1] -= self.mass * (position[0] - self.position[0]) * (position[1] - self.position[1])
        inertia[1,0] -= self.mass * (position[0] - self.position[0]) * (position[1] - self.position[1])
        inertia[0,2] -= self.mass * (position[0] - self.position[0]) * (position[2] - self.position[2])
        inertia[2,0] -= self.mass * (position[0] - self.position[0]) * (position[2] - self.position[2])
        inertia[1,2] -= self.mass * (position[1] - self.position[1]) * (position[2] - self.position[2])
        inertia[2,1] -= self.mass * (position[1] - self.position[1]) * (position[2] - self.position[2])
        return inertia

class CADPart(Part):
    def __init__(self, document, name, density):
        object = document.getObject(name)
        if object is None: raise Exception("Object " + name + " not found")
        self.mass = 1e-9 * object.Shape.Mass * density
        self.inertia = 1e-12 * np.matrix(object.Shape.MatrixOfInertia.A).reshape(4,4)[0:3,0:3]
        self.position = 1e-3 * np.array(object.Shape.CenterOfMass)

class BeamPart(Part):
    def __init__(self, length, radius, hollow, axis, density, position):
        if hollow:
            self.mass = length * density
            self.inertia = 1.0/12.0 * self.mass * length**2 * np.eye(3)
            self.inertia[axis,axis] = self.mass * radius**2
            self.position = position
        else:
            self.mass = np.pi * radius**2 * length * density
            self.inertia = 1.0/12.0 * self.mass * length**2 * np.eye(3)
            self.inertia[axis,axis] = 0.5 * self.mass * radius**2
            self.position = position

class BallPart(Part):
    def __init__(self, radius, hollow, density, position):
        if hollow:
            self.mass = 4.0 * np.pi * radius**2 * density
            self.inertia = 2.0/3.0 * self.mass * radius**2 * np.eye(3)
            self.position = position
        else:
            self.mass = 4.0/3.0 * np.pi * radius**3 * density
            self.inertia = 2.0/3.0 * self.mass * radius**2 * np.eye(3)
            self.position = position
        
class PartGroup(Part):
    def __init__(self, parts):
        self.mass = 0.0
        for part in parts: self.mass += part.mass

        self.position = np.zeros(3)
        for part in parts: self.position += part.position * part.mass / self.mass

        self.inertia = np.zeros((3,3))
        for part in parts: self.inertia += part.inertia_at(self.position)

def get_parts():
    density = 1250.0 #kg/m^3
    beam_length = 0.65 #m
    beam_radius = 0.005 #m
    beam_density = 0.0042 #kg/m
    ball_radius = 0.0275 #m
    ball_position = 0.41 #m
    beam_position = 0.06 #m
    document = FreeCAD.open(rospkg.RosPack().get_path("franka_pole") + "/meshes/1D.FCStd")

    lower = PartGroup([
        CADPart(document, "b_part8_001_", density),
        CADPart(document, "b_part11_001_", density)
        ])
    
    upper = PartGroup([
        CADPart(document, "b_part9_001_", density),
        CADPart(document, "b_part10_001_", density),
        CADPart(document, "b_part10_001_001", density),
        BeamPart(beam_length, beam_radius, True, 2, beam_density, np.array([0,0,beam_position])),
        BallPart(ball_radius, False, density, np.array([0,0,ball_position]))
        ])

    return lower, upper

def get_parts_2d():
    density = 0.2 * 1250.0 #kg/m^3
    beam_length = 0.65 #m
    beam_radius = 0.005 #m
    beam_density = 0.0042 #kg/m
    ball_radius = 0.0275 #m
    ball_position = 0.43+0.012 #m
    beam_position = 0.06+0.012 #m
    document = FreeCAD.open(rospkg.RosPack().get_path("franka_pole") + "/meshes/2D.FCStd")

    lower = PartGroup([
        CADPart(document, "b_part8_001_", density),
        CADPart(document, "b_part6_001_", density),
        CADPart(document, "b_part3_001_", density),
        CADPart(document, "b_part3_001_001", density),
        CADPart(document, "b_part7_001_", density),
        CADPart(document, "b_part13_001_", density)
        ])

    middle = PartGroup([
        CADPart(document, "b_part4_001_", density)
        ])
    
    upper = PartGroup([
        CADPart(document, "b_part1_001_", density),
        CADPart(document, "b_part3_001_002", density),
        CADPart(document, "b_part3_001_003", density),
        CADPart(document, "b_part12_001_", density),
        CADPart(document, "b_part5_001_", density),
        CADPart(document, "b_part2_001_", density),
        CADPart(document, "b_part2_001_001", density),
        CADPart(document, "b_part14_001_", density),
        CADPart(document, "b_part14_001_001", density),
        BeamPart(beam_length, beam_radius, True, 2, beam_density, np.array([0,0,beam_position])),
        BallPart(ball_radius, False, density, np.array([0,0,ball_position]))
        ])

    return lower, middle, upper

if __name__ == "__main__":
    if sys.argv[-1] == '2D':
        lower, middle, upper = get_parts_2d()
        print("Lower:")
        print(lower)
        print("Middle:")
        print(middle)
        print("Upper:")
        print(upper)
        #Warning: all coordinates are in CAD space / effector space. In reality upper link is 1.2cm higher
    else:
        lower, upper = get_parts()
        print("Lower:")
        print(lower)
        print("Upper:")
        print(upper)