#ifndef POWERCHECK_H
#define POWERCHECK_H

#include "application.h"
/**
 * Simple class to monitor for has power (USB or VIN), has a battery, and is charging
 *
 * Just instantiate one of these as a global variable and call loop() out of loop. You
 * should call it on every loop.
 */
class PowerCheck {
public:
	const unsigned long FAST_CHECK_INTERVAL_MS = 50;
	const unsigned long SLOW_CHECK_INTERVAL_MS = 2000;

	PowerCheck();
	virtual ~PowerCheck();

	/**
	 * Call out of loop(), perferably on every loop
	 */
	void loop();

	/**
	 * Returns true if the Electron has power, either a USB host (computer), USB charger, or VIN power.
	 */
	bool getHasPower() const { return hasPower; }

	/**
	 * Returns true if the Electron has a battery. This is only updated every 2 seconds.
	 */
	bool getHasBattery() const { return hasBattery; }

	/**
	 * Returns true if the Electron is currently charging (red light on)
	 */
	bool getIsCharging() const { return isCharging; }

private:
	unsigned long lastFastCheck = 0;
	unsigned long lastSlowCheck = 0;
	PMIC pmic;
	int checkCount = 0;
	int changeCount = 0;
	byte lastChrgStat = 0;
	bool hasPower = false;
	bool hasBattery = true;
	bool isCharging = false;
};

#endif
