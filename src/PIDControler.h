#include <Arduino.h>

class PIDController {
public:
	double kp;
	double ki;
	double kd;

	PIDController(double kp, double ki, double kd,double sortieMin, double sortieMax)
		: kp(kp), ki(ki), kd(kd), sortieMin(sortieMin), sortieMax(sortieMax), integral(sortieMin) {
	}

	double compute(float lecture, float consigne) {
		return compute(consigne - lecture);
	}

	double compute(double erreur) {
		proportionnel = kp * erreur;

		integral += ki*erreur;
		integral = constrain(integral, sortieMin , sortieMax );

		derivee = kd * (erreur - erreurPrec);
		erreurPrec = erreur;

		sortiePID = constrain(proportionnel + integral + derivee, sortieMin, sortieMax);
		return sortiePID;
	}

	double getP() const { return proportionnel; }
	double getI() const { return integral; }
	double getD() const { return derivee; }
	double getPID() const { return sortiePID; }
	double getDelta() {return erreurPrec; }

private:
	double erreurPrec = 0;
	double proportionnel;
	double integral;
	double derivee;
	double sortiePID;
	double sortieMax;
	double sortieMin;
};