all: joint_limits hand_limits

joint_limits:
	g++ -o test_checker test_checker.cc FRICheck_legacy.cc ../src/FRICheck.cc
	./test_checker

hand_limits:
	g++ -o verify_limits verify_limits.cc ../src/FRICheck.cc
	./verify_limits
	gnuplot verify_limits.gnuplot

clean:
	-rm limit_left limit_right verify_limits test_checker limits_extracted_* limits_interpolated_*
