#################################################################
##Use if camera and projector are driven by 2 different computers
#################################################################
#!/bin/bash
ssh -CX leha@192.168.150.108 "roscd projected_light && bash control_camera.sh"