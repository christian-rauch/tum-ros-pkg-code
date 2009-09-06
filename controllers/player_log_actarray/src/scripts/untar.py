#!/usr/bin/env python

import sys, os


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
        #os.system()
        
    

class FileNames:
    def __init__(self, fullname, basename):
        self.fn = fullname
        self.bn = basename



if __name__ == "__main__":
    untar()
