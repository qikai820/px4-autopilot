#include <FGFDMExec.h>
#include <models/FGExternalReactions.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

static void check(bool ok, const char *message)
{
	if (!ok) {
		std::fprintf(stderr, "FAIL: %s\n", message);
		std::exit(1);
	}
}

int main(int argc, char **argv)
{
	check(argc == 3, "model directory and systems directory required");
	JSBSim::FGFDMExec fdm;
	fdm.SetDebugLevel(0);
	check(fdm.LoadModel(SGPath(argv[1]), SGPath(argv[1]), SGPath(argv[2]), "voliro_t", false), "load model and dependencies");
	fdm.Setdt(.004);
	fdm.SetPropertyValue("ic/h-sl-ft", 10000.);
	check(fdm.RunIC(), "initialize model");
	auto run = [&]() {
		for (int i = 0; i < 1000; ++i) {
			check(fdm.Run(), "run model");
		}
	};
	auto reactions = fdm.GetExternalReactions();

	fdm.SetPropertyValue("fcs/voliro_t/motor-cmd[4]", .6);
	run();
	double fz = reactions->GetForces(3);
	double my = reactions->GetMoments(2);
	check(std::isfinite(fz) && std::isfinite(my), "finite positive-tail wrench");
	check(std::fabs(fz + 3.6 / 4.4482216152605) < 1e-5, "existing 10 N positive tail curve");
	check(std::fabs(my - (.64 / .3048) * fz) < 1e-5, "0.64 m tail moment arm");

	fdm.SetPropertyValue("fcs/voliro_t/motor-cmd[4]", -.6);
	run();
	fz = reactions->GetForces(3);
	my = reactions->GetMoments(2);
	check(std::fabs(fz - 2.16 / 4.4482216152605) < 1e-5, "existing estimated 6 N reverse tail curve");
	check(std::fabs(my - (.64 / .3048) * fz) < 1e-5, "reverse tail arm/sign");

	fdm.SetPropertyValue("fcs/voliro_t/motor-cmd[4]", 0.);

	for (int i = 0; i < 4; ++i) {
		char name[80];
		std::snprintf(name, sizeof(name), "fcs/voliro_t/motor-cmd[%d]", i);
		fdm.SetPropertyValue(name, .2);
	}

	run();
	fz = reactions->GetForces(3);
	my = reactions->GetMoments(2);
	check(std::isfinite(fz) && fz < 0., "finite upward front thrust");
	check(std::fabs(my + (.14 / .3048) * fz) < 1e-5, "0.14 m front moment arm");
	check(std::fabs(reactions->GetMoments(1)) < 1e-5, "equal front thrust has no roll moment");
	check(std::fabs(reactions->GetMoments(3)) < 1e-5, "coaxial front reaction torques cancel");
	std::puts("PASS: JSBSim model load, positive/reverse tail curves, 0.14/0.64 m moment arms and reaction signs");
}
