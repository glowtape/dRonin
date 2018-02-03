#include "pios.h"
#if defined(PIOS_INCLUDE_LQG)

#include "pios_thread.h"
#include "pios_modules.h"
#include "taskmonitor.h"
#include "taskinfo.h"

#include "lqg.h"

#include "lqgsolution.h"

#define STACK_SIZE_BYTES 600		// Reevaluate.
#define TASK_PRIORITY				PIOS_THREAD_PRIO_LOW

/* Hack things into pieces to give other threads more time. Seems like USB telemetry goes
   down the crapper for whatever reason when we try to solve everything in one go. Even
   though it just takes a fraction of a second. */
#define LQR_ITERATIONS_PER_BATCH	100
#define RTKF_ITERATIONS_PER_BATCH	30

// Private variables
static struct pios_thread *lqgSolverTaskHandle;
static bool module_enabled;

static void lqgSolverTask(void *parameters);

/**
 * start the module
 * \return -1 if start failed
 * \return 0 on success
 */
static int32_t lqgSolverStart(void)
{
	if (module_enabled) {
		// Start task
		lqgSolverTaskHandle = PIOS_Thread_Create(
				lqgSolverTask, "lqgSolver", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
		TaskMonitorAdd(TASKINFO_RUNNING_LQGSOLVER,
				lqgSolverTaskHandle);
		return 0;
	}
	return -1;
}

static int32_t lqgSolverInitialize(void)
{
	module_enabled = LQGSolutionInitialize() != -1;
	return 0;
}
MODULE_INITCALL(lqgSolverInitialize, lqgSolverStart)

static void lqgSolverTask(void *parameters)
{
	// Wait a bit before going.
	PIOS_Thread_Sleep(1000);

	lqg_t axis[3] = { 0, 0, 0 };
	int cnt = 0;
	LQGSolutionData lqrsol;

	while (1) {
		lqg_get_axes(axis);

		if (!(axis[0] || axis[1] || axis[2])) {
			PIOS_Thread_Sleep(100);
		} else {
			LQGSolutionGet(&lqrsol);

			for (int i = 0; i < 3; i++) {
				lqg_t lqg = axis[i];
				if (!lqg) continue;

				rtkf_t rtkf = lqg_get_rtkf(lqg);
				lqr_t lqr = lqg_get_lqr(lqg);

				if (!rtkf_is_solved(rtkf)) {
					rtkf_stabilize_covariance(rtkf, RTKF_ITERATIONS_PER_BATCH);
				}
				if (!lqr_is_solved(lqr)) {
					lqr_stabilize_covariance(lqr, LQR_ITERATIONS_PER_BATCH);
				} else {
					/* Keep going at it for live tuning via TxPID. */
					lqr_stabilize_covariance(lqr, 1);
				}

				if (cnt % 1000 == 0 && lqr_is_solved(lqr)) {
					float K[2];
					lqr_get_gains(lqr, K);

					switch (i) {
						case 0:
							lqrsol.RollK[0] = K[0];
							lqrsol.RollK[1] = K[1];
							break;
						case 1:
							lqrsol.PitchK[0] = K[0];
							lqrsol.PitchK[1] = K[1];
							break;
						case 2:
							lqrsol.YawK[0] = K[0];
							lqrsol.YawK[1] = K[1];
							break;
					}
				}
			}
			LQGSolutionSet(&lqrsol);

			/* Iterating one cycle per millisecond after initial solving should reconverge
			   the LQR within ~100-150ms. */
			cnt++;
			PIOS_Thread_Sleep(1);
		}
	}
}

#endif // PIOS_INCLUDE_LQGSOLVER