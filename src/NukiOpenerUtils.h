#pragma once

/**
 * @file NukiUtills.h
 * Implementation of generic/helper functions
 *
 * Created on: 2022
 * License: GNU GENERAL PUBLIC LICENSE (see LICENSE)
 *
 * This library implements the communication from an ESP32 via BLE to a Nuki smart lock.
 * Based on the Nuki Smart Lock API V2.2.1
 * https://developer.nuki.io/page/nuki-smart-lock-api-2/2/
 *
 */

#include "Arduino.h"
#include "NukiDataTypes.h"
#include "NukiOpenerConstants.h"
#include <bitset>

namespace NukiOpener {

void cmdResultToString(const CmdResult state, char* str);

/**
 * @brief Translate a bitset<N> into Nuki weekdays int
 *
 * @tparam N
 * @param bitset with bitset[0] = Monday ... bitset[7] = Sunday
 * @return uint8_t with bit6 = Monday ... bit0 = Sunday
 */
template<std::size_t N>
uint8_t getWeekdaysIntFromBitset(const std::bitset<N> bits);

void logOpenerErrorCode(uint8_t errorCode, bool debug = false);
void logConfig(Config config, bool debug = false);
void logNewConfig(NewConfig newConfig, bool debug = false);
void logNewKeypadEntry(NewKeypadEntry newKeypadEntry, bool debug = false);
void logKeypadEntry(KeypadEntry keypadEntry, bool debug = false);
void logUpdatedKeypadEntry(UpdatedKeypadEntry updatedKeypadEntry, bool debug = false);
void logAuthorizationEntry(AuthorizationEntry authorizationEntry, bool debug = false);
void logNewAuthorizationEntry(NewAuthorizationEntry newAuthorizationEntry, bool debug = false);
void logUpdatedAuthorizationEntry(UpdatedAuthorizationEntry updatedAuthorizationEntry, bool debug = false);
void logNewTimeControlEntry(NewTimeControlEntry newTimeControlEntry, bool debug = false);
void logTimeControlEntry(TimeControlEntry timeControlEntry, bool debug = false);
void logCompletionStatus(CompletionStatus completionStatus, bool debug = false);
void logNukiTrigger(Trigger nukiTrigger, bool debug = false);
void logLockAction(LockAction lockAction, bool debug = false);
void logKeyturnerState(OpenerState keyTurnerState, bool debug = false);
void logBatteryReport(BatteryReport batteryReport, bool debug = false);
void logLogEntry(LogEntry logEntry, bool debug = false);
void logAdvancedConfig(AdvancedConfig advancedConfig, bool debug = false);
void logNewAdvancedConfig(NewAdvancedConfig newAdvancedConfig, bool debug = false);

} // namespace Nuki