import mathutils
import yaml


def read_txt(file_path):
    file = open(file_path)
    lines = file.readlines()[1:]
    rows = []
    for line in lines:
        row = line.split()
        row = [float(item) for item in row]
        rows.append(row)
        # print(row)

    print(rows)
    file.close()
    return rows


def conv_to_rpy_xyz(list):
    rpy = []
    xyz = []
    for line in list:
        a, d, alfa, theta = line
        rotX = mathutils.Matrix.Rotation(alfa, 4,  'X')
        transX = mathutils.Matrix.Translation((a, 0, 0))
        rotZ = mathutils.Matrix.Rotation(theta, 4, 'Z')
        transZ = mathutils.Matrix.Translation((0, 0, d))
        T = rotX @ transX @ rotZ @ transZ
        xyz.append(T.to_translation())
        rpy.append(T.to_euler())
    return rpy, xyz


def write_yaml(dh_file):  # , yaml_file):
    rows = read_txt(dh_file)
    rpy, xyz = conv_to_rpy_xyz(rows)
    text = ""
    for i in range(len(rows)):
        text += "line"+str(i+1)+":\n"
        text += "  joint_rpy: " + \
            str(rpy[i][0])+" "+str(rpy[i][1])+" "+str(rpy[i][2])+"\n"
        text += "  joint_xyz: " + \
            str(xyz[i][0])+" "+str(xyz[i][1])+" "+str(xyz[i][2])+"\n"
        if i == 0:
            text += "  link_xyz: 0 0 "+str(0.5*rows[i][1])+"\n"
            text += "  size: "+str(rows[i][1])+"\n"
        else:
            text += "  link_xyz: "+str(0.5*rows[i][0])+" 0 0\n"
            text += "  size: "+str(rows[i][0])+"\n"
    with open(r'./param.yaml', 'w') as file:
        file.write(text)

        
write_yaml("./dh_matrix.txt")
