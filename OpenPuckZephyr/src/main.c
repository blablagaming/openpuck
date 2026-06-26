/*
 * OpenPuck — Zephyr port entry point (bring-up stub).
 *
 * This will grow into the cooperative pump that the Arduino loop() was. For now
 * it only proves the toolchain + board build and links.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(openpuck, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("OpenPuck Zephyr port: bring-up stub up");
	while (1) {
		k_sleep(K_SECONDS(1));
	}
	return 0;
}
