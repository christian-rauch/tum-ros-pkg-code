#!/usr/bin/env python

import sys, os
#, os, time


def to_xml(file, path):
    f = open(file, 'w')
    if path == None:
        path = '.'
    list = os.listdir(path)
    f.write('<?xml version="1.0" ?><imgdataset>')
    for file in list:
        if file[-4:] == '.png':
            angle = file.split('_')[1]
            f.write('<category>\n')
            f.write('\t<title>' + angle + '</title>' + ' <filename>' + './' + path + '/' + file + '</filename>\n')
            f.write('</category>\n')
    f.write('</imgdataset>')
    f.close





        
if __name__ == "__main__":
    if sys.argv.__len__() <= 1:
        print 'Provide arguments'
        exit(0)
    elif sys.argv[1] == '-toxml':
        to_xml(sys.argv[2], sys.argv[3])
