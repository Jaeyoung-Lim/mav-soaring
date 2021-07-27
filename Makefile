
.PHONY: test

format:
	Tools/fix_code_style.sh .

build:
	catkin build thermal_soaring ergodic_soaring

build-test:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=True
	catkin build thermal_soaring ergodic_soaring --no-deps -v -i --catkin-make-args tests

test:
	Tools/run_test.sh
