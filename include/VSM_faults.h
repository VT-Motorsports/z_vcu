#pragma once

enum class VSM_FAULTS : int
{
    NO_FAULT = 0,
    FAULTED = 4, // used when fault type is unknown but we know that a fault has occured
    INVERTER_VOLTAGE_SKEW = 12,
    PRECHARGING_TOOK_TOO_LONG = 13,
    BUS_VOLTAGE_DROPPED_AFTER_PRECHARGING = 14,
    IMD_FAULT = 1,
    AMS_FAULT = 2,
    BSPD_FAULT = 3,
    CONTACTOR_ATTEMPT_CLOSE_BEFORE_ARM = 21,
    CONTACTOR_FAULTED = 22,
    CONTACTOR_ATTEMPT_ARM_WHILE_FAULTED = 23,
};