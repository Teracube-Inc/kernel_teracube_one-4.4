#define PSAM_DEBUG_CODE

#define PSAM_DEVICE "PSAM"
#ifdef PSAM_DEBUG_CODE
#undef PSAM_DEBUG
#define PSAM_DEBUG(a,arg...) pr_err(PSAM_DEVICE ": " a, ##arg)
#define PSAM_FUNC()	pr_err(PSAM_DEVICE ": %s line=%d\n", __func__, __LINE__)
#else
#define PSAM_DEBUG(arg...)
#define PSAM_FUNC()
#endif

enum psam_report_state
{
	PSAM_IN =0,
	PSAM_OUT = 1,
};
