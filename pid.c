#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <getopt.h>

struct PID {
	double proportional, integral, derivative;
	double upperLimit, lowerLimit;
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

	printf("P: %3.3f I: %3.3f D: %3.3f ", p, i, d);

	double pid = p + i + d;

	pid = (pid < controller->lowerLimit) ? controller->lowerLimit : pid;
	pid = (pid > controller->upperLimit) ? controller->upperLimit : pid;

	printf("PID: %3.3f ", pid);

	return pid;
}

double drag(double environment, double system, double scalar) {
	double diff = system - environment;

	double sign = (system < environment) ? -1.0 : 1.0;

	diff /= 10;
	return diff*diff*sign*scalar;
}

void usage() {
	printf("Usage\n\n");

	printf("\t-p [integer]\tProportional factor * 1000\n");
	printf("\t-i [integer]\tIntegral factor * 1000\n");
	printf("\t-d [integer]\tDifferential factor * 1000\n");
	printf("\t-s [integer]\tInitial set point * 1000\n");
	printf("\t-m [integer]\tInitial measurement * 1000\n");
	printf("\t-r [integer]\tDrag scalar * 1000\n");
	printf("\t-e [integer]\tEnvironment value * 1000\n");
	printf("\t-f [integer]\tResponse factor* 1000\n");
	printf("\t-u [integer]\tUpper PID limit * 1000\n");
	printf("\t-l [integer]\tLower PID limit * 1000\n");
	printf("\t-n [integer]\tNumber of samples\n");
}

int main(int argc, char** argv) {

	double 	setPoint = 20.0,
		measurement = 2.0,
		response = 1.0,
		environment = 2.0,
		upperLimit = 5.0,
		lowerLimit = 5.0,
		dragScalar = 0.2;

	struct PID controller = { 
		.proportional = .5,
		.integral = .5,
		.derivative = .5,

		.upperLimit = 5.0,
		.lowerLimit = 5.0,

		.reset = 0,
		.oldError = 0,
	};

	int c = 0, iterations = 20;
	while ((c = getopt (argc, argv, "p:i:d:n:s:m:r:e:f:u:l:")) != -1) {
		switch (c) {
		case '?':
			usage();
			return 0;
		case 'p':
			controller.proportional = (double)(atoi(optarg))/1000;
			break;
		case 'i':
			controller.integral = (double)(atoi(optarg))/1000;
			break;
		case 'd':
			controller.derivative = (double)(atoi(optarg))/1000;
			break;
		case 'n':
			iterations = atoi(optarg);
			break;
		case 's':
			setPoint = (double)(atoi(optarg))/1000;
			break;
		case 'm':
			measurement = (double)(atoi(optarg))/1000;
			break;
		case 'r':
			dragScalar = (double)(atoi(optarg))/1000;
			break;
		case 'e':
			environment = (double)(atoi(optarg))/1000;
			break;
		case 'u':
			controller.upperLimit = (double)(atoi(optarg))/1000;
			break;
		case 'l':
			controller.lowerLimit = (double)(atoi(optarg))/1000;
			break;
		case 'f':
			response = (double)(atoi(optarg))/1000;
			break;
		}
	}

	for (int i = 0; i < iterations; i++) {
		printf("%3i: Measurement: %3.3f ", i, measurement);
		measurement += response*PID(measurement, setPoint, &controller);
		double dragValue= drag(environment, measurement, dragScalar); 
		measurement -= dragValue;
		printf("Drag: %3.3f\n", dragValue);
	}

	return EXIT_SUCCESS;
}
