#include <stdio.h>
#include <stdlib.h>

struct PID {
	double proportional, integral, derivative;
	double reset, oldError;
};

double ProportionalControl(double error, struct PID* restrict controller) {
	return error * controller->proportional;
}

double IntegralControl(double error, struct PID* restrict controller) {
	controller->reset += error;
	return controller->integral * controller->reset;
}

double DerivativeControl(double error, struct PID* restrict controller) {
	double returnValue = error - controller->oldError;
	controller->oldError = error;
	return controller->derivative * returnValue;
}

double PID(double input, double setPoint, struct PID* restrict controller) {
	
	double error = setPoint - input;

	double p = ProportionalControl(error, controller);
	double i = IntegralControl(error, controller);
	double d = DerivativeControl(error, controller);

	printf("P: %3.3f I: %3.3f D: %3.3f\n", p, i, d);

	return p+i+d;
}

int main(int argc, char** argv) {

	double 	setPoint = 20,
		measurement = 2;

	struct PID controller = { 
		.proportional = .5,
		.integral = .5,
		.derivative = .5,

		.reset = 0,
		.oldError = 0,
	};

	if (argc >= 4) {
		controller.proportional = ((double)atoi(argv[1]) / 10000.0);
		controller.integral = ((double)atoi(argv[2]) / 10000.0);
		controller.derivative= ((double)atoi(argv[3]) / 10000.0);

		printf("Kp: %f, Ki: %f Kd: %f\n", controller.proportional, controller.integral, controller.derivative);
	}

	if (argc >= 5) {
		setPoint = (double)atoi(argv[4]);
	}

	int itterations = 10;
	if (argc >= 6) {
		itterations = atoi(argv[5]);
	}

	for (int i = 0; i < itterations; i++) {
		printf("%3i: Measurement: %3.3f ", i, measurement);
		measurement += 0.3 * PID(measurement, setPoint, &controller) - 0.3;
	}

	return EXIT_SUCCESS;
}
