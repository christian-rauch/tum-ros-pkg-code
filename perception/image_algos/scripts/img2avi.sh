#encode movies from still images using mencoder
mencoder "mf://*.jpg" -mf w=1608:h=1236:fps=1 -o output.avi -ovc lavc -lavcopts vcodec=mpeg4
#encode for windows compatible
#mencoder "mf://*.png" -mf w=320:h=240:fps=5 -o output.avi -ovc lavc -lavcopts vc
#odec=mpeg1video
