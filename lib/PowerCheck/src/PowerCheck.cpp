#include "PowerCheck.h"

PowerCheck::PowerCheck() {
}

PowerCheck::~PowerCheck() {
}

void PowerCheck::loop() {
	if (millis() - lastFastCheck >= FAST_CHECK_INTERVAL_MS) {
		// For reasons explained in greater detail we need to check the system status register frequently.
		// But not too frequently, because it does use some system resources to communicate over I2C.
		// An interval of 50 milliseconds, or 20 times per second, seems about right.
		// Setting the interval to 100 milliseconds causes it to miss the condition looked for below sometimes.

		lastFastCheck = millis();

		byte systemStatus = pmic.getSystemStatus();
		if ((systemStatus & 0x04) != 0) {
			// Bit 2 (mask 0x4) == PG_STAT. If non-zero, power is good
			// This means we're powered off USB or VIN, so we don't know for sure if there's a battery

			// Having power and no battery is probably not what you'd expect:
			// if you're connected to a computer, it just alternates between these two states very quickly:
			// 0x64 Computer connected (VBUS=USB Host, Fast charging, Power Good)
			// 0x74 Computer connected (VBUS=USB Host, Charge Termination Done, Power Good)
			// (It works similarly for a USB charger, except it's 0x24 and 0x34).

			// Bit 5 CHRG_STAT[1] R
			// Bit 4 CHRG_STAT[0] R
			// 00 – Not Charging, 01 – Pre-charge (<VBATLOWV), 10 – Fast Charging, 11 – Charge Termination Done
			byte chrgStat = (systemStatus >> 4) & 0x3;

			if (chrgStat != lastChrgStat) {
				// Here's where we check to see if the charging status changes. When there's no battery present
				// it changes a lot between fast charging and charge termination done, but since we're polling
				// here instead of using interrupts, we have to poll frequently enough to see states.
				// (The BQ24195 power manager controller is connected to BATT_INT_PC13, but accessing that
				// interrupt from user code is a pain, so I just poll here.)
				changeCount++;
				lastChrgStat = chrgStat;
			}

			// We have power (USB or VIN)
			hasPower = true;

			// Other Common System Status Values:
			// 0x00 No USB or VIN power connected (running off battery)
			// 0x04 Power Good only - seems to happen in transition when it's deciding whether to charge or not
			// 0x34 USB Charger connected (Charge Termination Done, Power Good)
			// 0x74 Computer connected (VBUS=USB Host, Charge Termination Done, Power Good)
			//
			// Battery that needs charging:
			// 0x64 Computer connected (VBUS=USB Host, Fast charging, Power Good)
			//
			// Powered by VIN
			// 0x34 (Charge Termination Done, Power Good)
			// It does not seem to be possible to tell the difference between VIN and a USB charger
		}
		else {
			// Power good is false, so that means we must be running off battery
			hasPower = false;
		}

		checkCount++;
	}
	if (millis() - lastSlowCheck > SLOW_CHECK_INTERVAL_MS) {
		lastSlowCheck = millis();

		// Normally we get a checkCount of 36 or so, and a changeCount of 18 when there's no battery
		hasBattery = ! ((checkCount > 10) && (changeCount > (checkCount / 4)));

		// The battery is charging if in pre-charge or fast charge mode
		isCharging = hasBattery && (lastChrgStat == 1 || lastChrgStat == 2);

		checkCount = 0;
		changeCount = 0;
	}

}
