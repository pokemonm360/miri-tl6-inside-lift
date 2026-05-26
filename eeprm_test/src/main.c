#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define EEPROM_NODE DT_ALIAS(eeprom_0)

#if !DT_NODE_HAS_STATUS(EEPROM_NODE, okay)
#error "EEPROM alias eeprom-0 is not enabled in devicetree"
#endif

#define TEST_OFFSET 0x0100U
#define TEST_LEN    32U

static const struct device *const eeprom = DEVICE_DT_GET(EEPROM_NODE);

static void dump_block(const char *label, const uint8_t *buf, size_t len)
{
	printk("%s", label);
	for (size_t i = 0; i < len; i++) {
		printk(" %02x", buf[i]);
	}
	printk("\n");
}

static bool write_and_verify(uint32_t offset, const uint8_t *wr, uint8_t *rd,
			     size_t len)
{
	int ret;

	ret = eeprom_write(eeprom, offset, wr, len);
	if (ret != 0) {
		printk("EEPROM write failed at 0x%04x: %d\n", offset, ret);
		return false;
	}

	ret = eeprom_read(eeprom, offset, rd, len);
	if (ret != 0) {
		printk("EEPROM read failed at 0x%04x: %d\n", offset, ret);
		return false;
	}

	if (memcmp(wr, rd, len) != 0) {
		printk("EEPROM verify failed at 0x%04x\n", offset);
		dump_block("Wrote:", wr, len);
		dump_block("Read: ", rd, len);
		return false;
	}

	return true;
}

int main(void)
{
	uint8_t original[TEST_LEN];
	uint8_t pattern_a[TEST_LEN];
	uint8_t pattern_b[TEST_LEN];
	uint8_t readback[TEST_LEN];
	int ret;

	printk("24LC32AT EEPROM test start\n");
	printk("I2C 7-bit address: 0x50 (binary 1010000)\n");

	if (!device_is_ready(eeprom)) {
		printk("EEPROM device is not ready\n");
		return 0;
	}

	printk("EEPROM size: %zu bytes\n", eeprom_get_size(eeprom));
	printk("Test offset: 0x%04x, length: %u bytes\n", TEST_OFFSET, TEST_LEN);

	for (size_t i = 0; i < TEST_LEN; i++) {
		pattern_a[i] = (uint8_t)(0xA0U + i);
		pattern_b[i] = (uint8_t)(0x5FU - i);
	}

	ret = eeprom_read(eeprom, TEST_OFFSET, original, sizeof(original));
	if (ret != 0) {
		printk("Initial EEPROM read failed: %d\n", ret);
		return 0;
	}

	dump_block("Original:", original, sizeof(original));

	if (!write_and_verify(TEST_OFFSET, pattern_a, readback, sizeof(pattern_a))) {
		return 0;
	}
	printk("Pattern A OK\n");

	if (!write_and_verify(TEST_OFFSET, pattern_b, readback, sizeof(pattern_b))) {
		return 0;
	}
	printk("Pattern B OK\n");

	ret = eeprom_write(eeprom, TEST_OFFSET, original, sizeof(original));
	if (ret != 0) {
		printk("Original restore failed: %d\n", ret);
		return 0;
	}

	printk("Original block restored\n");
	printk("24LC32AT EEPROM test PASS\n");

	return 0;
}
