set terminal x11 size 1000 400 title "hand limits"

set xlabel "Joint 6"
set ylabel "Joint 7"
set ylabel offset 2,0

set multiplot
set size 0.5,1.0;

set origin 0.0,0.0;
set title "Left Hand Limits"
plot "./limits_recording_left" using ($1/pi*180):($2/pi*180) with lines title "recorded movement", \
     "limits_extracted_left" using ($1/pi*180):($2/pi*180) with lines title "extracted limits", \
     "limits_interpolated_left" using ($1/pi*180):($2/pi*180) with lines title "extracted limits (i)", \
     "limit_left" using ($1/pi*180):($2/pi*180) with lines title "current limits";

set origin 0.5,0.0;
set title "Right Hand Limits"
plot "./limits_recording_right" using ($1/pi*180):($2/pi*180) with lines title "recorded movement", \
     "limits_extracted_right" using ($1/pi*180):($2/pi*180) with lines title "extracted limits", \
     "limits_interpolated_right" using ($1/pi*180):($2/pi*180) with lines title "extracted limits (i)", \
     "limit_right" using ($1/pi*180):($2/pi*180) with lines title "current limits";

unset multiplot

pause -1
