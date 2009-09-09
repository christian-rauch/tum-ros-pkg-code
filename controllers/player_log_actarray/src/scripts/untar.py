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
        if 'delimited' in keys:
            command = 'tar cvvf ' + obj +  '.delimited.pcd.tar.bz2' + tar_str
        else:
            command = 'tar cvvf ' + obj +  '.pcd.tar.bz2' + tar_str
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

class FileNames:
    def __init__(self, fullname, basename):
        self.fn = fullname
        self.bn = basename


if __name__ == "__main__":
    if sys.argv[1] == '0':
        untar()
    elif  sys.argv[1] == 'r':
        rename_to_fixed_prec()
    else:
        keys = sys.argv[1:]
        tar(None, keys)
