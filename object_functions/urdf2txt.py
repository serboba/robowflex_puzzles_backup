import xml.etree.ElementTree as ET
import shutil
import os

def convert_files_to_xml():
    src_dir = os.getcwd()  # copy the urdf into xml_files and convert
    target_dir = src_dir + "/xml_files"
    for urdf_file in os.listdir(os.getcwd()):
        if urdf_file.endswith(".urdf"):
            temp = urdf_file.split(".")
            new_name = temp[0] + '.xml'
            src_file = os.path.join(src_dir, urdf_file)
            target_file = os.path.join(target_dir, new_name)
            shutil.copy2(src_file, target_file)
        if urdf_file.endswith(".srdf"):
            temp = urdf_file.split(".")
            new_name = temp[0] + '_srdf.xml'
            src_file = os.path.join(src_dir, urdf_file)
            target_file = os.path.join(target_dir, new_name)
            shutil.copy2(src_file, target_file)
    translate_into_txt(target_dir)
    delete_xml(target_dir)

def translate_into_txt(target_dir):

    for f in os.listdir(target_dir):
        f_new = f.split('.')[0]
        f_new = f_new+'.txt'
        print(f_new)
        txt_file = []
        root = ET.parse(target_dir+'/'+f).getroot()
        if(f.endswith('_srdf.xml')):
            txt_file = parse_srdf_file(root)
        elif(f.endswith('.xml')):
            txt_file = parse_urdf_file(root)
        else:
            continue
        if(len(txt_file)>=1):
            write_into_txt(txt_file,(target_dir+'/'+f_new))


def parse_urdf_file(root):
    links = []
    for link in root.findall('link'):
        link_name = link.get('name')
        if (link.find('visual') != None and link.find('visual/geometry/mesh') == None):  # if object different type ?
            link_size = link.find('visual/geometry/box').get('size')
            link_xyz = link.find('visual/origin').get('xyz')
            link_rpy = link.find('visual/origin').get('rpy')
            line1 = link_name + ',' + link_size + ',' + link_rpy + ',' + link_xyz
            links.append(line1)

    joints = []
    indices =[]

    for joint in root.findall('joint'):  # inside robowflex getACM (unnecessary)
        j_child_name = joint.find('child').get('link')
        if(joint.get('type') == 'fixed'):
            continue
        index_j = [i for i, s in enumerate(links) if j_child_name in s]
        if (len(index_j)>0):
            indices.append(index_j[0])
            print(index_j, j_child_name)
            print(links)
        if any(j_child_name in s for s in links):
            origin_xyz = joint.find('origin').get('xyz')
            origin_rpy = joint.find('origin').get('rpy')
            j_name = joint.get('name')
            j_type = joint.get('type')
            j_axis = joint.find('axis').get('xyz')
            j_lower_limit = joint.find('limit').get('lower')
            j_upper_limit = joint.find('limit').get('upper')
            j_line = j_name + ',' + origin_rpy + ',' + origin_xyz + '\n' + j_type + ',' + j_axis + '\n' + j_lower_limit+ ',' +j_upper_limit
            joints.append(j_line)

    merge = []
    for i in range(len(indices)):
        merge.append(links[indices[i]])
        merge.append(joints[i])
    return merge


def parse_srdf_file(root):
    groups = []
    for group in root.findall('group'):
        groups_joint = group.get('name')
        joint_name = group.find('joint').get('name')
        g_line = joint_name + ',' + groups_joint
        groups.append(g_line)
    return groups
    # print(g_line)

def write_into_txt(file,name):
    with open(name, 'w') as f:
        for item in file:
            f.write("%s\n" % item)

def delete_xml(target_dir):
    for f in os.listdir(target_dir):
        if(f.endswith('.xml')):
            os.remove(target_dir+'/'+f)

convert_files_to_xml()
