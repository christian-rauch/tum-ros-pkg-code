all: joint_limits hand_limits

FRI_HEADERS= $(shell rospack export --lang=cpp --attrib=cflags kuka_fri_header)

joint_limits:
	g++ -o test_checker test_checker.cc FRICheck_legacy.cc ../src/FRICheck.cc
	./test_checker

hand_limits:
	g++ -o verify_limits verify_limits.cc ../src/FRICheck.cc
	./verify_limits
	gnuplot verify_limits.gnuplot

communication:
	gcc -o test_structs ${FRI_HEADERS} test_structs.c
	./test_structs

arm_sim:
	g++ -o fri_dummy_arm ${FRI_HEADERS} fri_dummy_arm.cc

clean:
	-rm limit_left limit_right verify_limits test_checker limits_extracted_* limits_interpolated_*

