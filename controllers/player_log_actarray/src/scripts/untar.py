#!/usr/bin/env python

import sys, os, time


def untar(path=None):
    basename_list = []
    if path == None:
        path = '.'
    list = os.listdir(path)
    for i in list:
        basename_list.append(FileNames(i, i.split('.')[0]))
    for j in basename_list:
        command = 'mkdir ' + j.bn + '; cd ' + j.bn + '; tar xvvf ../' + j.fn + '; cd ..'
        print command;
        os.system(command)
        
def tar(path=None, keys = []):
    pcdlist = []
    if path == None:
        path = '.'
    list = os.listdir(path)
    for file in list:
        append = True
        for k in keys:
            if file.find(k) == -1:
                append = False
                break;
        if append == True:
            pcdlist.append(file)
    objects_list = []
    for file in pcdlist:
        if file.split('_')[0] not in objects_list:
            objects_list.append(file.split('_')[0])
    print 'objects_list: ', objects_list, '\n'
    tar_str = ''
    for obj in objects_list:
        for file in pcdlist:
            if file.find(obj) != -1:
                tar_str += ' ' + file
        if 'delimited'in keys and not 'denoised' in keys and not 'rotated' in keys :
            command = 'tar cvvf ' + obj +  '.delimited.pcd.tar.bz2' + tar_str
        elif  'delimited' in keys and 'denoised' in keys and not 'rotated' in keys:
            command = 'tar cvvf ' + obj +  '.delimited.denoised.pcd.tar.bz2' + tar_str
        elif 'delimited' in keys and 'denoised' in keys and 'rotated' in keys:
            command = 'tar cvvf ' + obj +  '.delimited.denoised.rotated.pcd.tar.bz2' + tar_str
        else:
            print "Unknown arguments!!"
            quit(0)
        os.system(command)
        tar_str = ''
        print 'obj: ', obj, 'command: ', command, '\n'

def rename_to_fixed_prec(path=None):
    pcdlist = []
    if path == None:
        path = '.'
    list = os.listdir(path)
    for file in list:
        nr_old = file.split('_')[1]
        nr_new = '%04.d' %int(nr_old)
        file_renamed=file.replace(nr_old, nr_new)
        command = 'mv ' + file + ' ' + file_renamed
        print "moving: ", command, "\n"
        os.system(command)

def find_missing (key, path=None):
    angles = ['-180', '0180', '-150', '0150', '-120', '0120',  '-090', '0090', '-060', '0060', '-030', '0030', '0000']
    s_a = set(angles)
    if path == None:
        path = '.'
    dir_list = os.listdir(path)
    f=open('missing.log', 'a+')
    print 'dir list', dir_list
    for dir in dir_list:
        file_list = os.listdir(dir)
        angle_list = []
        for file in file_list:
            if file.find(key) != -1:
                #print "file", file
                angle_list.append (str(file.split('_')[1]))
                #print 'angle', angle
        if s_a.difference(set(angle_list)).__len__() != 0:
           # print s_a, 'del\n', set(angle_list)
           # print 'diff', s_a.difference(set(angle_list))
            f.write(file.split('_')[0] + ' ')
            f.write(str(s_a.difference(set(angle_list))) + '\n')
    f.close()

class FileNames:
    def __init__(self, fullname, basename):
        self.fn = fullname
        self.bn = basename


if __name__ == "__main__":
    if sys.argv[1] == '0':
        untar()
    elif  sys.argv[1] == 'r':
        rename_to_fixed_prec()
    elif  sys.argv[1] == 'f':
        find_missing(sys.argv[2], None)
    else:
        keys = sys.argv[1:]
        tar(None, keys)
