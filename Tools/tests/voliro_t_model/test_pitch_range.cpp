#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <cassert>
#include <cmath>
#include <cstdio>

int main(int argc, char **argv)
{
	assert(argc == 3);
	JSBSim::FGFDMExec fdm;
	fdm.SetDebugLevel(0);
	fdm.Setdt(.0025);
	assert(fdm.LoadModel(SGPath(argv[1]), SGPath(argv[1]), SGPath(argv[2]), "voliro_t", false));
	fdm.GetIC()->SetAltitudeASLFtIC(10000.);
	assert(fdm.RunIC());
	fdm.SuspendIntegration();
	const double limit = 160. * M_PI / 180.;

	for (double input : {-2., -1., -.5, 0., .5, 1., 2.}) {
		fdm.SetPropertyValue("fcs/voliro_t/pitch-cmd-norm", input);

		for (int i = 0; i < 2000; ++i) { assert(fdm.Run()); }

		const double expected = fmax(-1., fmin(1., input)) * limit;
		assert(fabs(fdm.GetPropertyValue("fcs/voliro_t/pitch-cmd-rad") - expected) < 1e-9);
		assert(fabs(fdm.GetPropertyValue("fcs/voliro_t/pitch-rad") - expected) < 1e-8);
	}

	fdm.ResumeIntegration();

	for (double angle : {-160., -100., -90., -85., 0., 85., 90., 100., 160.}) {
		const double theta = angle * M_PI / 180., bank = 5. * M_PI / 180.;
		fdm.GetIC()->SetPhiRadIC(atan2(sin(bank), cos(bank) * cos(theta)));
		fdm.GetIC()->SetThetaRadIC(asin(cos(bank) * sin(theta)));
		fdm.GetIC()->SetPsiRadIC(atan2(sin(bank) * sin(theta), cos(theta)));
		assert(fdm.RunIC());
		fdm.SuspendIntegration();
		assert(fdm.Run());
		const double shown = fdm.GetPropertyValue("fcs/voliro_t/independent-pitch-deg");
		assert(fabs(shown - angle) < 1e-5);
		fdm.ResumeIntegration();
	}

	puts("PASS: model normalized servo +/-160, clipping, and independent pitch display through +/-90");
}
