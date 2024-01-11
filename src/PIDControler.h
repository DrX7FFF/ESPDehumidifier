#include <Arduino.h>

class PIDController {
public:
	double kp;
	double ki;
	double kd;

	PIDController(double kp, double ki, double kd)
		: kp(kp), ki(ki), kd(kd) {
	}

	double compute(float lecture, float consigne) {
		return compute(consigne - lecture);
	}

	double compute(double erreur) {
		// Calcul des termes PID
		proportionnel = kp * erreur;

		// Limiter la somme d'erreur pour éviter le "windup"
		sommeErreur += erreur;
		sommeErreur = constrain(sommeErreur, -100, 100);
		integral = ki * sommeErreur;

		derivee = kd * (erreur - erreurPrec);

		// Mise à jour de la variable d'erreur précédente
		erreurPrec = erreur;

		// Calcul de la sortie PID
		sortiePID = proportionnel + integral + derivee;
		return sortiePID;
	}

	double getP() const { return proportionnel; }
	double getI() const { return integral; }
	double getD() const { return derivee; }
	double getPID() const { return sortiePID; }

private:
	double erreurPrec;
	double sommeErreur;
	double proportionnel;
	double integral;
	double derivee;
	double sortiePID;
};