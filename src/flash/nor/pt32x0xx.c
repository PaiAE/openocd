#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>
#include <target/target_type.h>

#define PT32X0XX_FLASH_BASE                			0
#define PT32X0XX_FLASH_PAGE_SIZE					512

#define PT32X0XX_FLASH_REG_BASE             		0x40000000
#define PT32X0XX_FLASH_REG_CR1             			0x40000000
#define PT32X0XX_FLASH_REG_CR2             			0x40000004
#define PT32X0XX_FLASH_REG_KR1             			0x4000000C
#define PT32X0XX_FLASH_REG_KR2             			0x40000010
#define PT32X0XX_FLASH_REG_SR1             			0x40000018
#define PT32X0XX_FLASH_REG_AR             			0x40000020
#define PT32X0XX_FLASH_REG_DR1             			0x40000024
#define PT32X0XX_FLASH_REG_OPTCR					0x40000030

#define PT32X0XX_RCC_REG_ASRCR						0x40020028

#define PT32X0XX_ES_REG_CIR							0x40020800

#define PT32X0XX_FLASH_CR2_PG                   	0x00000001
#define PT32X0XX_FLASH_CR2_PER                   	0x00000002
#define PT32X0XX_FLASH_CR2_MER                   	0x00000004

#define PT32X0XX_FLASH_KR1_UKEY_UNLOCK          	0x3B6A0000
#define PT32X0XX_FLASH_KR1_UKEY_LOCK          		0xEA2D0000
#define PT32X0XX_FLASH_KR1_KEY_MAINCODE				0x0000ADEB
#define PT32X0XX_FLASH_KR1_KEY_DATAAREA				0x000063D2

#define PT32X0XX_FLASH_KR2_UKEY_UNLOCK          	0xB75C0000
#define PT32X0XX_FLASH_KR2_UKEY_LOCK          		0xEA2D0000
#define PT32X0XX_FLASH_KR2_KEY_UNLOCK				0x0000D3A5

#define PT32X0XX_FLASH_SR1_PROGRAM_DONE				0x00000001
#define PT32X0XX_FLASH_SR1_PAGE_ERASE_DONE			0x00000002
#define PT32X0XX_FLASH_SR1_MASS_ERASE_DONE			0x00000004

#define PT32X0XX_FLASH_SR1_CMD_ERR    				0x00000010
#define PT32X0XX_FLASH_SR1_KEY_ERR   				0x00000020
#define PT32X0XX_FLASH_SR1_ADDR_ERR   				0x00000040

#define PT32X0XX_FLASH_REG_OPTCR_RDP_MASK			0x000000FF
#define PT32X0XX_FLASH_OPTCR_RDP_UNPROTECTED		0x000000AA
#define PT32X0XX_FLASH_OPTCR_RDP_PROTECTED			0x000000CC
#define PT32X0XX_FLASH_OPTCR_KEY_VALUE				0x15EC1CCA

#define PT32X0XX_RCC_ASFCR_RELOAD					0x0000CD23

#define PT32X0XX_ES_REG_CIR_FSIZE_MASK				0x0000000F

struct pt32x0xx_flash_bank {
	bool probed;
};

static int pt32x0xx_erase_page(struct flash_bank *bank, uint32_t addr)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* if not called, GDB errors will be reported during large writes */
	keep_alive();

	uint32_t status = 0;

	int retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR1, PT32X0XX_FLASH_KR1_UKEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	if(addr >= 0x1C000000)
	{
		retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR1, PT32X0XX_FLASH_KR1_KEY_DATAAREA);
	}
	else
	{
		retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR1, PT32X0XX_FLASH_KR1_KEY_MAINCODE);
	}
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_AR, addr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR2, PT32X0XX_FLASH_KR2_UKEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;
	
	retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR2, PT32X0XX_FLASH_KR2_KEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_CR2, PT32X0XX_FLASH_CR2_PER);
	if (retval != ERROR_OK)
		return retval;
	
	do
	{
		retval = target_read_u32(target, PT32X0XX_FLASH_REG_SR1, &status);
		if (retval != ERROR_OK)
			return retval;
	}while(!(status & PT32X0XX_FLASH_SR1_PAGE_ERASE_DONE));

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_SR1, PT32X0XX_FLASH_SR1_PAGE_ERASE_DONE);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int pt32x0xx_write_block(struct flash_bank *bank, const uint8_t *buffer, uint32_t address, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	static const uint8_t pt32x0xx_flash_write_code[] = {
#include "../../../contrib/loaders/flash/pt32/pt32x0xx.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(pt32x0xx_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(pt32x0xx_flash_write_code), pt32x0xx_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(count * 4 + 8, MAX(buffer_size, 256));
	/* Normally we allocate all available working area.
	 * MIN shrinks buffer_size if the size of the written block is smaller.
	 * MAX prevents using async algo if the available working area is smaller
	 * than 256, the following allocation fails with
	 * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
	 */

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always 32-bit word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, PT32X0XX_FLASH_REG_BASE);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 4,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32, buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & PT32X0XX_FLASH_SR1_CMD_ERR) {
			LOG_ERROR("flash program cmd error");
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & PT32X0XX_FLASH_SR1_KEY_ERR) {
			LOG_ERROR("flash program key error");
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & PT32X0XX_FLASH_SR1_ADDR_ERR) {
			LOG_ERROR("invalid flash memory write address");
		}
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

static int pt32x0xx_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* if not called, GDB errors will be reported during large writes */
	keep_alive();

	int retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR1, PT32X0XX_FLASH_KR1_UKEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR1, PT32X0XX_FLASH_KR1_KEY_MAINCODE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_AR, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR2, PT32X0XX_FLASH_KR2_UKEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;
	
	retval = target_write_u32(target, PT32X0XX_FLASH_REG_KR2, PT32X0XX_FLASH_KR2_KEY_UNLOCK);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_CR2, PT32X0XX_FLASH_CR2_MER);
	if (retval != ERROR_OK)
		return retval;
	
	do
	{
		retval = target_read_u32(target, PT32X0XX_FLASH_REG_SR1, &status);
		if (retval != ERROR_OK)
			return retval;
	}while(!(status & PT32X0XX_FLASH_SR1_MASS_ERASE_DONE));

	retval = target_write_u32(target, PT32X0XX_FLASH_REG_SR1, PT32X0XX_FLASH_SR1_MASS_ERASE_DONE);
	if (retval != ERROR_OK)
		return retval;	
	
	return retval;
}

static int pt32x0xx_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	//uint32_t opt_value;
	
	//retval = target_read_u32(target, PT32X0XX_FLASH_REG_OPTCR, &opt_value);
	//if (retval != ERROR_OK)
	//	return retval;

	//if ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_PROTECTED)
	//{
	//	LOG_ERROR("Flash is in read-protect state, please unlock it first.");
	//	return ERROR_FLASH_PROTECTED;
	//}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return pt32x0xx_mass_erase(bank);

	uint32_t addr = bank->base + bank->sectors[first].offset;
	uint32_t endAddr = bank->base + bank->sectors[last].offset;

	addr = addr & ~(PT32X0XX_FLASH_PAGE_SIZE - 1);

	while(addr <= endAddr)
	{
		retval = pt32x0xx_erase_page(bank, addr);
		if (retval != ERROR_OK)
			LOG_ERROR("Failed to erase page at address %08x", addr);
		
		addr += PT32X0XX_FLASH_PAGE_SIZE;
	}

	return retval;
}

static int pt32x0xx_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	(void)first;
	(void)last;
	
	struct target *target = bank->target;
	int ret = 0;
	uint32_t opt_value;

	ret = target_read_u32(target, PT32X0XX_FLASH_REG_OPTCR, &opt_value);
	if (ret != ERROR_OK)
		return ret;

	if (((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_PROTECTED && set)
			|| ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_UNPROTECTED && set == 0))
	{
		return ERROR_OK;
	}

	ret = target_write_u32(target, PT32X0XX_FLASH_REG_OPTCR, PT32X0XX_FLASH_OPTCR_KEY_VALUE);
	if (ret != ERROR_OK)
		return ret;
	
	if (set)
	{
		ret = target_write_u32(target, PT32X0XX_FLASH_REG_OPTCR, PT32X0XX_FLASH_OPTCR_RDP_PROTECTED);
	}
	else
	{
		ret = target_write_u32(target, PT32X0XX_FLASH_REG_OPTCR, PT32X0XX_FLASH_OPTCR_RDP_UNPROTECTED);
	}
	
	if (ret != ERROR_OK)
		return ret;

	target_write_u32(target, PT32X0XX_RCC_REG_ASRCR, PT32X0XX_RCC_ASFCR_RELOAD);

	target->reset_halt = true;
	target->type->assert_reset(target);
	target->type->deassert_reset(target);

	if (target->state != TARGET_HALTED)
	{
		target_halt(target);
		target_wait_state(target, TARGET_HALTED, 5000);
	}

	ret = target_read_u32(target, PT32X0XX_FLASH_REG_OPTCR, &opt_value);
	if (ret != ERROR_OK)
		return ret;

	if ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_PROTECTED && (!set))
	{
		LOG_ERROR("Flash read protection unlock fail.");
		return ERROR_FLASH_UNPROTECTED;
	}
	else if ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_UNPROTECTED && set)
	{
		LOG_ERROR("Flash read protection lock fail.");
		return ERROR_FLASH_PROTECTED;
	}
	else
	{
		return ERROR_OK;
	}
}

static int pt32x0xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	uint8_t *new_buffer = NULL;
	// uint32_t opt_value;

	//int retval = target_read_u32(target, PT32X0XX_FLASH_REG_OPTCR, &opt_value);
	//if (retval != ERROR_OK)
	//	return retval;

	//if ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_PROTECTED)
	//{
	//	LOG_ERROR("Flash is in read-protect state, please unlock it first.");
	//	return ERROR_FLASH_PROTECTED;
	//}
	
	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	if (count & 0x3) {
		uint32_t old_count = count;
		count = (old_count | 3) + 1;
		new_buffer = malloc(count);
		if (!new_buffer) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32 " and padding with 0xff", old_count, count);
		memset(new_buffer, 0xff, count);
		buffer = memcpy(new_buffer, buffer, old_count);
	}
		
	uint32_t words_remaining = count / 4;

	/* try using a block write */
	retval = pt32x0xx_write_block(bank, buffer, bank->base + offset, words_remaining);

	free(new_buffer);
	return retval;
}

static int pt32x0xx_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	assert(bank->sectors);

	uint32_t opt_value;

	int retval = target_read_u32(target, PT32X0XX_FLASH_REG_OPTCR, &opt_value);
	if (retval != ERROR_OK)
		return retval;

	int is_protected = 0;
	if ((opt_value & PT32X0XX_FLASH_REG_OPTCR_RDP_MASK) == PT32X0XX_FLASH_OPTCR_RDP_PROTECTED)
	{
		is_protected = 1;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = is_protected;

	return ERROR_OK;
}

static int pt32x0xx_probe(struct flash_bank *bank)
{
	struct pt32x0xx_flash_bank *pt32x0xx_info = bank->driver_priv;
	uint16_t flash_size_in_kb = 32;
	int page_size = PT32X0XX_FLASH_PAGE_SIZE;
	uint32_t base_address = PT32X0XX_FLASH_BASE;

	pt32x0xx_info->probed = false;

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / page_size;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	pt32x0xx_info->probed = true;

	return ERROR_OK;
}

static int leo01_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	struct pt32x0xx_flash_bank *pt32x0xx_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	int page_size = PT32X0XX_FLASH_PAGE_SIZE;
	uint32_t base_address = PT32X0XX_FLASH_BASE;

	pt32x0xx_info->probed = false;

	uint32_t value;

	int retval = target_read_u32(target, PT32X0XX_ES_REG_CIR, &value);
	if (retval != ERROR_OK)
		return retval;

	value &= PT32X0XX_ES_REG_CIR_FSIZE_MASK;
	if (value == 0) {
		flash_size_in_kb = 32;
	} else if (value == 1) {
		flash_size_in_kb = 64;
	} else {
		flash_size_in_kb = 120;
	}

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / page_size;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	pt32x0xx_info->probed = true;

	return ERROR_OK;
}

static int pt32x0xx_auto_probe(struct flash_bank *bank)
{
	struct pt32x0xx_flash_bank *pt32x0xx_info = bank->driver_priv;
	
	if (pt32x0xx_info->probed)
		return ERROR_OK;
	
	return pt32x0xx_probe(bank);
}

static int leo01_auto_probe(struct flash_bank *bank)
{
	struct pt32x0xx_flash_bank *pt32x0xx_info = bank->driver_priv;
	
	if (pt32x0xx_info->probed)
		return ERROR_OK;
	
	return leo01_probe(bank);
}

/* flash bank pt32x0xx <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(pt32x0xx_flash_bank_command)
{
	struct pt32x0xx_flash_bank *pt32x0xx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pt32x0xx_info = malloc(sizeof(struct pt32x0xx_flash_bank));

	bank->driver_priv = pt32x0xx_info;
	pt32x0xx_info->probed = false;

	/* The flash write must be aligned to a halfword boundary */
	bank->write_start_alignment = bank->write_end_alignment = 4;

	return ERROR_OK;
}

COMMAND_HANDLER(pt32x0xx_handle_lock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	if (pt32x0xx_protect(bank, 1, 0, 0) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(pt32x0xx_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (pt32x0xx_protect(bank, 0, 0, 0) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD, "%s unlocked.\n", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(pt32x0xx_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = pt32x0xx_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "%s mass erase complete", bank->driver->name);
	else
		command_print(CMD, "%s mass erase failed", bank->driver->name);

	return retval;
}

static const struct command_registration pt32x0xx_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = pt32x0xx_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = pt32x0xx_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = pt32x0xx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pt32x0xx_command_handlers[] = {
	{
		.name = "pt32x0xx",
		.mode = COMMAND_ANY,
		.help = "pt32x0xx flash command group",
		.usage = "",
		.chain = pt32x0xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pt32g031x_flash = {
	.name = "pt32g031x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = pt32x0xx_probe,
	.auto_probe = pt32x0xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver pt32x002x_flash = {
	.name = "pt32x002x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = pt32x0xx_probe,
	.auto_probe = pt32x0xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver pt32x012x_flash = {
	.name = "pt32x012x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = pt32x0xx_probe,
	.auto_probe = pt32x0xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver pt32x060x_flash = {
	.name = "pt32x060x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = leo01_probe,
	.auto_probe = leo01_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver pt32x063x_flash = {
	.name = "pt32x063x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = leo01_probe,
	.auto_probe = leo01_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver pt32x066x_flash = {
	.name = "pt32x066x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = leo01_probe,
	.auto_probe = leo01_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};	

const struct flash_driver ptm160x_flash = {
	.name = "ptm160x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = pt32x0xx_probe,
	.auto_probe = pt32x0xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver ptm280x_flash = {
	.name = "ptm280x",
	.commands = pt32x0xx_command_handlers,
	.flash_bank_command = pt32x0xx_flash_bank_command,
	.erase = pt32x0xx_erase,
	.protect = pt32x0xx_protect,
	.write = pt32x0xx_write,
	.read = default_flash_read,
	.probe = pt32x0xx_probe,
	.auto_probe = pt32x0xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pt32x0xx_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};

