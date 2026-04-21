/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifndef USE_WING

#ifdef USE_POSITION_HOLD

#include "math.h"
#include "build/debug.h"
#include "common/maths.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/compass.h"

#include "pg/pos_hold.h"
#include "pg/autopilot_multirotor.h"
#include "pos_hold.h"

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
    bool useStickAdjustment;
} posHoldState_t;

static posHoldState_t posHold;

#ifdef USE_POSHOLD_CHIRP
#include "common/chirp.h"
chirp_t posChirp;
static bool posChirpAxisY = false;
static float posChirpBasePositionCm[2] = {0.0f, 0.0f};
#endif

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
    posHold.useStickAdjustment = posHoldConfig()->deadband;

#ifdef USE_POSHOLD_CHIRP
    chirpInit(&posChirp,
              autopilotConfig()->posChirpStartFreqHzDeci / 10.0f,
              autopilotConfig()->posChirpEndFreqHzDeci / 10.0f,
              autopilotConfig()->posChirpSweepTimeSec,
              HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ) * 1000000);
    posChirpAxisY = false;
#endif

}

static void posHoldCheckSticks(void)
{
    // if failsafe is active, eg landing mode, don't update the original start point
    if (!failsafeIsActive() && posHold.useStickAdjustment) {
        const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
        setSticksActiveStatus(sticksDeflected);
    }
}

static bool sensorsOk(void)
{
    if (!STATE(GPS_FIX)) {
        return false;
    }
    if (
#ifdef USE_MAG
        !compassIsHealthy() &&
#endif
        (!posHoldConfig()->posHoldWithoutMag || !canUseGPSHeading)) {
        return false;
    }
    return true;
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            resetPositionControl(&gpsSol.llh, POSHOLD_TASK_RATE_HZ); // sets target location to current location
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled && posHold.isControlOk) {
        posHold.areSensorsOk = sensorsOk();
        if (posHold.areSensorsOk) {
            posHoldCheckSticks();
            posHold.isControlOk = positionControl(); // false only on sanity check failure
        }
    }

#ifdef USE_POSHOLD_CHIRP
    static bool wasPosChirpActive = false;
    bool isPosChirpActive = FLIGHT_MODE(POSHOLD_CHIRP_MODE);

    if (isPosChirpActive) {
        // Toggle axis if we re-activate the switch
        if (!wasPosChirpActive) {
            if (posChirp.isFinished) {
                posChirpAxisY = !posChirpAxisY; // Switch to Y axis on next toggle
            }
            chirpReset(&posChirp);
            posChirpBasePositionCm[X] = posHold.targetPositionCm[X]; // Store base
            posChirpBasePositionCm[Y] = posHold.targetPositionCm[Y];
        }

        posChirpUpdate(&posChirp);

        float currentExcitation = autopilotConfig()->posChirpAmpl * posChirp.exc;

        if (!posChirpAxisY) {
            posHold.targetPositionCm[X] = posChirpBasePositionCm[X] + currentExcitation;
        } else {
            posHold.targetPositionCm[Y] = posChirpBasePositionCm[Y] + currentExcitation;
        }

        DEBUG_SET(DEBUG_POSHOLD_CHIRP, 0, posChirpAxisY ? 1 : 0); // 0 = X, 1 = Y
        DEBUG_SET(DEBUG_POSHOLD_CHIRP, 1, lrintf(currentExcitation));
        DEBUG_SET(DEBUG_POSHOLD_CHIRP, 2, lrintf(posChirp.fchirp * 100));
    } else {
        if (wasPosChirpActive) {
            chirpReset(&posChirp);
        }
    }
    wasPosChirpActive = isPosChirpActive;
#endif
}

bool posHoldFailure(void) {
    // used only to display warning in OSD if requested but failing
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

#ifdef USE_POSHOLD_CHIRP
bool posHoldChirpIsFinished(void) {
    return posChirp.isFinished;
}
#endif

#endif // USE_POSITION_HOLD

#endif // !USE_WING
