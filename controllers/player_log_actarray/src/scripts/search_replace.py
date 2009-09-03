import os, sys

def search_replace(path, out):
    stext = '<depend package="player_bender_dummy" />'
    rtext = '<!--<depend package="player_bender_dummy" />-->'   
    print 'Sought after text: ', stext
    print 'Replace text: ', rtext
    #path = 'manifest.xml'
    #out = 'tmp'
    input = open(path)
    output = open(out, 'w')
    for s in input.xreadlines():
        output.write(s.replace(stext, rtext))
    input.close()
    output.close()
    command = 'mv ' + out + ' ' + path
    os.system(command)

def undo_search_replace(path, out):
    rtext = '<depend package="player_bender_dummy" />'
    stext = '<!--<depend package="player_bender_dummy" />-->'   
    print 'Sought after text: ', stext
    print 'Replace text: ', rtext
   # path = 'manifest.xml'
   # out = 'tmp'
    input = open(path)
    output = open(out, 'w')
    for s in input.xreadlines():
        output.write(s.replace(stext, rtext))
    input.close()
    output.close()
    command = 'mv ' + out + ' ' + path
    os.system(command)
    

if __name__ == "__main__":
    if sys.argv[3] == '0':
        search_replace(sys.argv[1],sys.argv[2])
    elif sys.argv[3] == '1':
        undo_search_replace(sys.argv[1],sys.argv[2])
    else:
        print 'Do nothing!!'
