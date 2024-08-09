/** @file sr_nvm.c
 *  @brief SR1000 non-volatile memory module.
 *
 *  Functions related to reading and writing the NVM and to its protocol.
 *
 *  @copyright Copyright (C) 2018-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sr_nvm.h"
#include <string.h>
#include "sr_access.h"
#include "sr_api.h"
#include "sr_def.h"

/* TEMP ******************************************************************/
#include "sr_access.h"

/* CONSTANTS ******************************************************************/

#define NVM_KEY_LAYOUT_VER_NAME   "NVM Layout Ver"
#define NVM_KEY_SERIAL_NO_NAME    "Serial No     "
#define NVM_KEY_CALIBRATION_NAME  "Calibration   "
#define NVM_KEY_WIDTH_FREQ_NAME   "Width Freq    "
#define NVM_KEY_FREQ_TICKS_NAME   "Freq Ticks    "
#define NVM_KEY_WAFER_LOT_NAME    "Wafer Lot     "
#define NVM_KEY_TEST_DATE_NAME    "Test Date     "
#define NVM_KEY_TP_VERSION_NAME   "TP Version    "
#define NVM_KEY_REG_CORNER_NAME   "Reg Corner    "
#define NVM_KEY_PRODUCT_ID_NAME   "Product ID    "
#define NVM_KEY_VCRO_SHIFT_NAME   "VCRO Shift    "
#define NVM_KEY_VREF_ADJUST_NAME  "Vref Adjust   "
#define NVM_POST_WRITE_DELAY_MS   150
#define NVM_BINNING_SETUP_MASK    0xFFFF000000000000
#define NVM_BINNING_SETUP_POS     48
#define NVM_CHIP_ID_MASK          0x000000FFFFFFFFFF
#define PULSE_WIDTH_OFFSET_LENGTH 4
#define VREF_TUNE_OFFSET_LENGTH   4

/* MACROS *********************************************************************/
#define NVM_KEY_COUNT        (sizeof(nvm_template) / sizeof(nvm_template[0]))
#define BIT_CHECK(var, nbit) ((var) & (1 << ((nbit) - 1)))

/* PRIVATE GLOBALS ************************************************************/
static nvm_entry_t nvm_template[NVM_KEY_LAST - 1] = {
    /* Currently, keys 1 to 11 are valid */
    {NVM_KEY_LAYOUT_VER, NVM_KEY_LAYOUT_VER_NAME, 1, NULL},   {NVM_KEY_SERIAL_NO, NVM_KEY_SERIAL_NO_NAME, 8, NULL},
    {NVM_KEY_CALIBRATION, NVM_KEY_CALIBRATION_NAME, 1, NULL}, {NVM_KEY_WIDTH_FREQ, NVM_KEY_WIDTH_FREQ_NAME, 1, NULL},
    {NVM_KEY_FREQ_TICKS, NVM_KEY_FREQ_TICKS_NAME, 1, NULL},   {NVM_KEY_WAFER_LOT, NVM_KEY_WAFER_LOT_NAME, 3, NULL},
    {NVM_KEY_TEST_DATE, NVM_KEY_TEST_DATE_NAME, 3, NULL},     {NVM_KEY_TP_VERSION, NVM_KEY_TP_VERSION_NAME, 2, NULL},
    {NVM_KEY_REG_CORNER, NVM_KEY_REG_CORNER_NAME, 1, NULL},   {NVM_KEY_PRODUCT_ID, NVM_KEY_PRODUCT_ID_NAME, 2, NULL},
    {NVM_KEY_VCRO_SHIFT, NVM_KEY_VCRO_SHIFT_NAME, 2, NULL},   {NVM_KEY_VREF_ADJUST, NVM_KEY_VREF_ADJUST_NAME, 1, NULL},
};

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void extract_values(nvm_t *nvm);

/* PUBLIC FUNCTIONS ***********************************************************/
bool sr_nvm_init(radio_t *radio, nvm_t *nvm)
{
    bool nvm_is_populated = false;

    /* Copy template to object */
    for (uint8_t i = 0; i < (NVM_KEY_LAST - 1); ++i) {
        nvm->entry[i] = nvm_template[i];
    }

    sr_nvm_read(radio, nvm->shadow_nvm, NVM_FIRST_ADDRESS, NVM_LAST_ADDRESS);

    nvm_is_populated = (nvm->shadow_nvm[0] != NVM_KEY_TERMINATOR);
    if (nvm_is_populated) {
        extract_values(nvm);
    }

    return nvm_is_populated;
}

uint8_t *sr_nvm_get_value(nvm_entry_t *nvm_entry, uint8_t key)
{
    return ((key == 0) || (key > NVM_KEY_COUNT) ? NULL : nvm_entry[KEY_TO_INDEX(key)].value);
}

uint8_t sr_nvm_get_size(nvm_entry_t *nvm_entry, uint8_t key)
{
    return ((key == 0) || (key > NVM_KEY_COUNT)) ? 0 : nvm_entry[KEY_TO_INDEX(key)].size;
}

char *sr_nvm_get_name(nvm_entry_t *nvm_entry, uint8_t key)
{
    return ((key == 0) || (key > NVM_KEY_COUNT)) ? NULL : nvm_entry[KEY_TO_INDEX(key)].name;
}

int8_t sr_nvm_read(radio_t *radio, uint8_t *buf, uint8_t addr_start, uint8_t addr_end)
{
    uint8_t idx          = 0;
    uint8_t addr_current = addr_start;
    uint8_t *read_reg;

    /* Wake up the radio */
    uwb_write_register_8(radio, REG_ACTIONS, 0);
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_AWAKE));

    sr_nvm_power_up(radio);

    while (addr_current <= addr_end) {
        uwb_write_register_8(radio, REG_NVMADDRESS, BIT_ROMPWRSW | addr_current++);
        read_reg = uwb_read_register_8(radio, REG_NVMVALUE);
        uwb_transfer_blocking(radio);
        buf[idx++] = *read_reg;
    }

    sr_nvm_power_down(radio);

    return 0;
}

uint8_t sr_nvm_get_product_id_model(nvm_t *nvm)
{
    uint8_t *ptr;
    uint16_t product_id;
    uint8_t model;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_PRODUCT_ID);

    if (ptr == NULL) {
        model = 0;
    } else {
        product_id = (ptr[1] << 8) | ptr[0]; /* TODO: Create a util for that */
        model      = MASK2VAL(product_id, BITS_PID_MODEL);
    }

    return model;
}

uint8_t sr_nvm_get_product_id_version(nvm_t *nvm)
{
    uint8_t *ptr;
    uint16_t product_id;
    uint8_t version;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_PRODUCT_ID);

    if (ptr == NULL) {
        version = 0;
    } else {
        product_id = (ptr[1] << 8) | ptr[0]; /* TODO: Create a util for that */
        version    = MASK2VAL(product_id, BITS_PID_VERSION);
    }

    return version;
}

uint8_t sr_nvm_get_product_id_package(nvm_t *nvm)
{
    uint8_t *ptr;
    uint16_t product_id;
    uint8_t package;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_PRODUCT_ID);

    if (ptr == NULL) {
        package = 0;
    } else {
        product_id = (ptr[1] << 8) | ptr[0]; /* TODO: Create a util for that */
        package    = MASK2VAL(product_id, BITS_PID_PACKAGE);
    }

    return package;
}

uint8_t sr_nvm_get_calibration(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t resistune;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_CALIBRATION);

    if (ptr == NULL) {
        resistune = 0;
    } else {
        resistune = *ptr;
    }

    return resistune;
}

uint16_t sr_nvm_get_vcro_shift(nvm_t *nvm)
{
    uint8_t *ptr;
    uint16_t vcro_shift;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_VCRO_SHIFT);

    if (ptr == NULL) { /* Keep backward compatibility with old SR1020 calibration */
        vcro_shift = 0x0000;
    } else {
        vcro_shift = (ptr[0] << 8) | ptr[1]; /* Value is written in big-endian in NVM */
    }

    return vcro_shift;
}

int8_t sr_nvm_get_vref_adjust_pulse_width_offset(nvm_t *nvm)
{
    uint8_t *ptr;
    int8_t pulse_width_offset, pwo_temp;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_VREF_ADJUST);

    if (ptr == NULL) {
        pulse_width_offset = 0x00;
    } else {
        pwo_temp = ptr[0] >> 4;
        if (BIT_CHECK(pwo_temp, PULSE_WIDTH_OFFSET_LENGTH)) {
            pulse_width_offset = 0xF0 | pwo_temp;
        } else {
            pulse_width_offset = pwo_temp;
        }
    }

    return pulse_width_offset;
}

int8_t sr_nvm_get_vref_adjust_vref_tune_offset(nvm_t *nvm)
{
    uint8_t *ptr;
    int8_t vref_tune_offset, vto_temp;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_VREF_ADJUST);

    if (ptr == NULL) {
        vref_tune_offset = 0x00;
    } else {
        vto_temp = ptr[0] & 0x0F;
        if (BIT_CHECK(vto_temp, VREF_TUNE_OFFSET_LENGTH)) {
            vref_tune_offset = 0xF0 | vto_temp;
        } else {
            vref_tune_offset = vto_temp;
        }
    }

    return vref_tune_offset;
}

uint64_t sr_nvm_get_serial_number(nvm_t *nvm)
{
    uint8_t *ptr;
    uint64_t serial_num_lsb;
    uint64_t serial_num_msb;
    uint64_t serial_num;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_SERIAL_NO);
    if (ptr == NULL) {
        serial_num = 0;
    } else {
        serial_num_msb = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
        serial_num_msb &= 0x00000000FFFFFFFF;
        serial_num_lsb = (ptr[4] << 24) | (ptr[5] << 16) | (ptr[6] << 8) | ptr[7];
        serial_num_lsb &= 0x00000000FFFFFFFF;
        serial_num = (serial_num_msb << 32) | serial_num_lsb;
    }

    return serial_num;
}

uint16_t sr_nvm_get_serial_number_binning_setup_code(nvm_t *nvm)
{
    uint64_t serial_number;
    uint16_t binning_setup_code;

    serial_number      = sr_nvm_get_serial_number(nvm);
    binning_setup_code = (serial_number & NVM_BINNING_SETUP_MASK) >> NVM_BINNING_SETUP_POS;

    return binning_setup_code;
}

uint64_t sr_nvm_get_serial_number_chip_id(nvm_t *nvm)
{
    uint64_t serial_number;
    uint64_t chip_id;

    serial_number = sr_nvm_get_serial_number(nvm);
    chip_id       = serial_number & NVM_CHIP_ID_MASK;

    return chip_id;
}

uint8_t sr_nvm_get_layout_version(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t layout_ver;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_LAYOUT_VER);

    if (ptr == NULL) {
        layout_ver = 0;
    } else {
        layout_ver = *ptr;
    }

    return layout_ver;
}

uint8_t sr_nvm_get_width_code(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t width_freq;
    uint8_t width_code;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_WIDTH_FREQ);
    if (ptr == NULL) {
        width_code = 0;
    } else {
        width_freq = *ptr;
        width_code = width_freq & BITS_WIDTH_PULSE_WIDTH;
        width_code = width_code >> 5;
    }

    return width_code;
}

uint8_t sr_nvm_get_width_base_freq(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t width_freq;
    uint8_t base_freq_at_room_temp;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_WIDTH_FREQ);
    if (ptr == NULL) {
        base_freq_at_room_temp = 0;
    } else {
        width_freq             = *ptr;
        base_freq_at_room_temp = width_freq & BITS_WIDTH_BASE_FREQ_ROOM_TEMP;
    }

    return base_freq_at_room_temp;
}

uint8_t sr_nvm_get_freq_tick(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t freq_ticks;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_FREQ_TICKS);
    if (ptr == NULL) {
        freq_ticks = 0;
    } else {
        freq_ticks = *ptr;
    }

    return freq_ticks;
}

uint16_t sr_nvm_get_wafer_lot(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t wafer_value;
    uint16_t wafer_lot;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_WAFER_LOT);
    if (ptr == NULL) {
        wafer_lot = 0;
    } else {
        wafer_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        wafer_value = wafer_value >> 8;
        wafer_value &= BITS_WAFER_LOT;
        wafer_lot = wafer_value;
    }

    return wafer_lot;
}

uint8_t sr_nvm_get_wafer_id(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t wafer_value;
    uint8_t wafer_id;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_WAFER_LOT);
    if (ptr == NULL) {
        wafer_id = 0;
    } else {
        wafer_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        wafer_value = wafer_value & BITS_WAFER_ID;
        wafer_value = wafer_value >> 16;
        wafer_id    = wafer_value;
    }

    return wafer_id;
}

bool sr_nvm_get_test_date_century(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t test_date_value;
    bool century;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_TEST_DATE);
    if (ptr == NULL) {
        century = 0;
    } else {
        test_date_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        /* Year[7..1], Century[0] - Month[7..4] - Data [7..3] */
        century = test_date_value & BIT_TEST_DATE_CENTURY;
    }

    return century;
}

uint8_t sr_nvm_get_test_date_year(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t test_date_value;
    uint8_t year;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_TEST_DATE);
    if (ptr == NULL) {
        year = 0;
    } else {
        test_date_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        /* Year[7..1], Century[0] - Month[7..4] - Data [7..3] */
        year = test_date_value & BITS_TEST_DATE_YEAR;
        /* Year[7..1], Century[0] */
        year = year >> 1;
    }

    return year;
}

uint8_t sr_nvm_get_test_date_month(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t test_date_value;
    uint8_t month;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_TEST_DATE);
    if (ptr == NULL) {
        month = 0;
    } else {
        test_date_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        /* Year[7..1], Century[0] - Month[7..4] - Data [7..3] */
        month = (test_date_value >> 8) & BITS_TEST_DATE_MONTH;
        /* Month[7..4] */
        month = month >> 4;
    }

    return month;
}

uint8_t sr_nvm_get_test_date_date(nvm_t *nvm)
{
    uint8_t *ptr;
    uint32_t test_date_value;
    uint8_t date;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_TEST_DATE);
    if (ptr == NULL) {
        date = 0;
    } else {
        test_date_value = (ptr[2] << 16) | (ptr[1] << 8) | ptr[0];
        /* Year[7..1], Century[0] - Month[7..4] - Data [7..3] */
        date = (test_date_value >> 16) & BITS_TEST_DATE_DATE;
        /* Date[7..3] */
        date = date >> 3;
    }

    return date;
}

uint16_t sr_nvm_get_tp_version(nvm_t *nvm)
{
    uint8_t *ptr;
    uint16_t tp_version;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_TP_VERSION);
    if (ptr == NULL) {
        tp_version = 0;
    } else {
        tp_version = (ptr[1] << 8) | ptr[0];
    }

    return tp_version;
}

uint8_t sr_nvm_get_region(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t region;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_REG_CORNER);
    if (ptr == NULL) {
        region = 0;
    } else {
        region = *ptr;
        region &= BITS_REGION_CORNER_REGION;
        region = region >> 4;
    }

    return region;
}

uint8_t sr_nvm_get_corner(nvm_t *nvm)
{
    uint8_t *ptr;
    uint8_t corner;

    ptr = sr_nvm_get_value(nvm->entry, NVM_KEY_REG_CORNER);
    if (ptr == NULL) {
        corner = 0;
    } else {
        corner = *ptr;
        corner &= BITS_REGION_CORNER_CORNER;
    }

    return corner;
}

void sr_nvm_power_up(radio_t *radio)
{
    
    uint8_t *read_reg;
    read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
    uwb_transfer_blocking(radio);
    /* Wake up the radio */
    uwb_write_register_8(radio, REG_NVMADDRESS, BIT_ROMPWRSW);
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_ROMEN));
    radio->radio_hal.delay_ms(NVM_DELAY_AFTER_ROMEN_SET_MS);
}

void sr_nvm_power_down(radio_t *radio)
{
    uwb_write_register_8(radio, REG_NVMADDRESS, 0x00);
    uwb_transfer_blocking(radio);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Extract usable values from the NVM.
 *
 *  This populates the NVM array value.
 *  @param[in] nvm  NVM structure.
 */
static void extract_values(nvm_t *nvm)
{
    uint8_t key, size, addr;

    addr = NVM_FIRST_ADDRESS;
    do {
        key = nvm->shadow_nvm[addr++]; /* First valid value should be a key */
        if (key == NVM_KEY_INVALID) {
            if (addr > NVM_LAST_ADDRESS) { /* Misburned code has been "erased", check next address */
                break;
            }
            continue;
        } else if (key == NVM_KEY_TERMINATOR) {
            break;
        }
        size = sr_nvm_get_size(nvm->entry, key);
        if (size == 0) {
            /* Unknown key, abort parsing */
            break;
        }
        nvm->entry[KEY_TO_INDEX(key)].value = &(nvm->shadow_nvm[addr]); /* First byte of the value is right after the key */
        addr += size;                                                   /* Skip to next key */
        /* Stop when the terminator or end of NVM is reached */
    } while ((nvm->shadow_nvm[addr] != NVM_KEY_TERMINATOR) && (addr <= NVM_LAST_ADDRESS));
}
