/** @file sr_nvm.h
 *  @brief SR1000 non-volatile memory module.
 *
 *  Functions related to reading and writing the NVM and to its protocol.
 *
 *  @copyright Copyright (C) 2018 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_NVM_H_
#define SR_NVM_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sr_api.h"
#include "sr_api_hal.h"
#include "sr_radio_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define NVM_SIZE_BYTES               128
#define NVM_FIRST_ADDRESS            0
#define NVM_LAST_ADDRESS             127
#define NVM_LAST_BIT_POS             7
#define NVM_DELAY_AFTER_ROMEN_SET_MS 1
#define NVM_OVERWRITE_RETRY_COUNT    3
#define NVM_DEFAULT_LAYOUT           0

/* Product ID fields */
#define BITS_PID_MODEL   BITS16_1(3, 0)
#define BITS_PID_UNUSED  BITS16_1(7, 4)
#define BITS_PID_PACKAGE BITS16_2(3, 0)
#define BITS_PID_VERSION BITS16_2(7, 4)

/* Width freq field */
#define BITS_WIDTH_BASE_FREQ_ROOM_TEMP BITS8(4, 0)
#define BITS_WIDTH_PULSE_WIDTH         BITS8(7, 5)

/* Wafer lot field */
#define BITS_WAFER_ID  BITS24(20, 16)
#define BITS_WAFER_LOT BITS24(15, 0)

/* Test date field */

#define BITS_TEST_DATE_YEAR   BITS8(7, 1)
#define BIT_TEST_DATE_CENTURY BIT(0)
#define BITS_TEST_DATE_MONTH  BITS8(7, 4)
#define BITS_TEST_DATE_DATE   BITS8(7, 3)

/* Region corner field */
#define BITS_REGION_CORNER_CORNER BITS8(3, 0)
#define BITS_REGION_CORNER_REGION BITS8(7, 4)

/* VCRO Shift fields */
#define BITS_FREQ_SHIFT_RX_1 0xf000
#define BITS_FREQ_SHIFT_RX_0 0x0f00
#define BITS_FREQ_SHIFT_TX_1 0x00f0
#define BITS_FREQ_SHIFT_TX_0 0x000f

#define POS_FREQ_SHIFT_RX_1 12
#define POS_FREQ_SHIFT_RX_0 8
#define POS_FREQ_SHIFT_TX_1 4
#define POS_FREQ_SHIFT_TX_0 0

/* Binning setup code definitions */
#define INTERNAL_SR1010_BINNING_SETUP_CODE 0x4143u /* AC */
#define INTERNAL_SR1020_BINNING_SETUP_CODE 0x4243u /* BC */
#define ATE_BINNING_SETUP_CODE             0x4954u /* IT */

/* TYPES **********************************************************************/
typedef enum phy_version {
    PHY_VERSION_8_0 = 0,
    PHY_VERSION_8_1 = 1,
    PHY_VERSION_8_2 = 2,
    PHY_VERSION_8_3 = 3,
} phy_version_t;

/** @brief NVM entry key enumeration
 *
 *  @note Keys start at 1
 */
typedef enum nvm_entry_key {
    NVM_KEY_TERMINATOR  = 0x00, /*!< NVM delimiter */
    NVM_KEY_LAYOUT_VER  = 0x01, /*!< Layout version NVM entry key, 8-bit value.*/
    NVM_KEY_SERIAL_NO   = 0x02, /*!< Serial number NVM entry key, 64-bit unique value.*/
    NVM_KEY_CALIBRATION = 0x03, /*!< Calibration NVM entry key, 8-bit value with field :
                                 *  PLL_RES[6..4] and VREF_TUNE[3..0].
                                 */
    NVM_KEY_WIDTH_FREQ = 0x04,  /*!< Pulse width and base frequency NVM entry key, 8-bit value with field :
                                 *  WIDTH_CODE[7..5] and FREQ_CODE[4..0].
                                 */
    NVM_KEY_FREQ_TICKS = 0x05,  /*!< Frequency ticks NVM entry key, 8-bit value */
    NVM_KEY_WAFER_LOT  = 0x06,  /*!< Wafer lot and ID NVM entry key, 24-bit value with field :
                                 *  WAFER_LOT_0[7..0], WAFER_LOT_1[15..8] and WAFER_ID[20..16].
                                 */
    NVM_KEY_TEST_DATE = 0x07,   /*!< Test date NVM entry key, 24-bit value with field :
                                 *  Date[23..19], month[15..12], year[7..1] and century[0].
                                 */
    NVM_KEY_TP_VERSION = 0x08,  /*!< Test program version NVM entry key, 16-bit value */
    NVM_KEY_REG_CORNER = 0x09,  /*!< Region corner NVM entry key, 8-bit value with field :
                                 *  REGION[7..4] and CORNER[3..0].
                                 */
    NVM_KEY_PRODUCT_ID = 0x0A,  /*!< Product ID NVM entry key, 16-bit value */
    NVM_KEY_VCRO_SHIFT = 0x0B,  /*!< VCRO Shift NVM entry key, 16-bit value with field :
                                 *  FREQ_SHIFT_RX_1[15..12], FREQ_SHIFT_RX_0[11..8],
                                 *  FREQ_SHIFT_TX_1[7..4] and FREQ_SHIFT_TX_0[3..0].
                                 */
    NVM_KEY_VREF_ADJUST = 0x0C, /*!< Offset values for VREF_TUNE and PULSE_WIDTH :
                                 *  PULSE_WIDTH_OFFSET[7..4], VREF_TUNE_OFFSET[3..0].
                                 *  Each 4-bit value is signed(two's complement)
                                 */
    NVM_KEY_LAST,               /*!< NVM delimiter */
    NVM_KEY_INVALID = 0xFF      /*!< Invalid NVM entry key */
} nvm_entry_key_t;

/** @brief NVM entry key structure
 *
 */
typedef struct {
    uint8_t key;    /*!< NVM entry key, equal to index + 1 */
    char *name;     /*!< User name for NVM entry */
    uint8_t size;   /*!< Size of the NVM entry value, in 8-bit count */
    uint8_t *value; /*!< Pointer to value/buffer of the NVM entry */
} nvm_entry_t;

/** @brief NVM structure.
 *
 */
typedef struct {
    nvm_entry_t entry[NVM_KEY_LAST - 1]; /*!< Sorted NVM raw data, by entry key.*/
    uint8_t shadow_nvm[NVM_SIZE_BYTES];  /*!< NVM raw data */
} nvm_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the NVM.
 *
 *  This populates an array in the MCU that mirrors
 *  the content of the NVM.
 *
 *  @param[in] radio  Radio's instance.
 *  @param[in] nvm    NVM structure.
 *  @retval true   NVM is properly initialized.
 *  @retval false  NVM is not properly initialized.
 */
bool sr_nvm_init(radio_t *radio, nvm_t *nvm);

/** @brief Get a value from the NVM.
 *
 *  A call to sr_nvm_init() must have been made
 *  prior to using this function.
 *
 *  @param[in] nvm_entry  NVM entry table.
 *  @param[in] key        The key associated with the value.
 *  @return A pointer to the array of bytes containing the value or NULL not found.
 */
uint8_t *sr_nvm_get_value(nvm_entry_t *nvm_entry, uint8_t key);

/** @brief Get the size of value in NVM entry table.
 *
 *  @param[in] nvm_entry  NVM entry table.
 *  @param[in] key        The key associated with the value.
 *  @return The size of the data or 0 if the key is invalid.
 */
uint8_t sr_nvm_get_size(nvm_entry_t *nvm_entry, uint8_t key);

/** @brief Get the name of a key.
 *
 *  @param[in] nvm_entry  NVM entry table.
 *  @param[in] key        The key (1 to NVM_ENTRY_COUNT) for which the name is required.
 *  @return Pointer to name string.
 */
char *sr_nvm_get_name(nvm_entry_t *nvm_entry, uint8_t key);

/** @brief Read a range of NVM locations.
 *
 *  @param[in]  radio       Radio's instance.
 *  @param[out] buf         Values read from the NVM.
 *  @param[in]  addr_start  Address of the first memory location.
 *  @param[in]  addr_end    Address of the last memory location.
 */
int8_t sr_nvm_read(radio_t *radio, uint8_t *buf, uint8_t addr_start, uint8_t addr_end);

/** @brief Power up the NVM.
 *
 *  @param[in] radio  Radio's instance.
 */
void sr_nvm_power_up(radio_t *radio);

/** @brief Power down the NVM.
 *
 *  @param[in] radio  Radio's instance.
 */
void sr_nvm_power_down(radio_t *radio);

/** @brief Get the product id model from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Product ID model.
 */
uint8_t sr_nvm_get_product_id_model(nvm_t *nvm);

/** @brief Get the product id version from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Product ID version.
 */
uint8_t sr_nvm_get_product_id_version(nvm_t *nvm);

/** @brief Get the product id package from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Product ID package.
 */
uint8_t sr_nvm_get_product_id_package(nvm_t *nvm);

/** @brief Get the calibration value from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Calibration value.
 */
uint8_t sr_nvm_get_calibration(nvm_t *nvm);

/** @brief Get the VCRO shift value from the NVM.
 *
 *  Value in NVM for the VCRO shift are written in
 *  big endian. The order is corrected in the return
 *  value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return VCRO shift value.
 */
uint16_t sr_nvm_get_vcro_shift(nvm_t *nvm);

/** @brief Get the pulse width offset value from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Pulse width offset value.
 */
int8_t sr_nvm_get_vref_adjust_pulse_width_offset(nvm_t *nvm);

/** @brief Get the vref tune offset value from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Vref tune offset value.
 */
int8_t sr_nvm_get_vref_adjust_vref_tune_offset(nvm_t *nvm);

/** @brief Get the layout version from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Layout version value.
 */
uint8_t sr_nvm_get_layout_version(nvm_t *nvm);

/** @brief Get the unique 64 bit serial number from the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Serial number value.
 */
uint64_t sr_nvm_get_serial_number(nvm_t *nvm);

/** @brief Get the 16-bit binning setup code from the NVM.
 *  @Note  The binning setup code is a serial number field.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Binning setup code.
 */
uint16_t sr_nvm_get_serial_number_binning_setup_code(nvm_t *nvm);

/** @brief Get the 40-bit chip ID from the NVM.
 *  @Note  The chip ID is a serial number field.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Chip ID.
 */
uint64_t sr_nvm_get_serial_number_chip_id(nvm_t *nvm);

/** @brief Get the width code from the NVM.
 *
 *  Return the width code field of the WIDTH_FREQ entry
 *  in the NVM, located in bits 7..5 of the 8 bits value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Width code value (3 bits).
 */
uint8_t sr_nvm_get_width_code(nvm_t *nvm);

/** @brief Get the base frequency from the NVM.
 *
 *  Return the code for base frequency at room temperature(25°C) field
 *  of the WIDTH_FREQ entry in the NVM. The field is located in bits 4..0
 *  of the 8 bits value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Base frequency at room temperature code (5 bits).
 */
uint8_t sr_nvm_get_width_base_freq(nvm_t *nvm);

/** @brief Get the frequency tick from the NVM.
 *
 *  Return the frequency in multiples of 40.96 MHz that
 *  correspond to the base frequency at room temperature from
 *  the WIDTH_FREQ entry. The frequency tick is taken from the
 *  FREQ_TICKS entry in NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Frequency ticks value.
 */
uint8_t sr_nvm_get_freq_tick(nvm_t *nvm);

/** @brief Get the wafer lot from the NVM.
 *
 *  Return the wafer lot field from the WAFER_LOT
 *  entry in the NVM. The field is located in bit 15..0 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Wafer lot value (16 bits).
 */
uint16_t sr_nvm_get_wafer_lot(nvm_t *nvm);

/** @brief Get the wafer ID from the NVM.
 *
 *  Return the wafer ID field from the WAFER_LOT
 *  entry in the NVM. The field is located in bit 20..16 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Wafer ID value (5 bits).
 */
uint8_t sr_nvm_get_wafer_id(nvm_t *nvm);

/** @brief Get the century from the NVM.
 *
 *  Return the century field from the TEST_DATE
 *  entry in the NVM. The field is located in bit 0 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @retval 0  2000.
 *  @retval 1  2100.
 */
bool sr_nvm_get_test_date_century(nvm_t *nvm);

/** @brief Get the year from the NVM.
 *
 *  Return the year field from the TEST_DATE
 *  entry in the NVM. The field is located in bit 7..1 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Year value (7 bits).
 */
uint8_t sr_nvm_get_test_date_year(nvm_t *nvm);

/** @brief Get the month from the NVM.
 *
 *  Return the month field from the TEST_DATE
 *  entry in the NVM. The field is located in bit 15..12 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Month value (4 bits).
 */
uint8_t sr_nvm_get_test_date_month(nvm_t *nvm);

/** @brief Get the date from the NVM.
 *
 *  Return the date field from the TEST_DATE
 *  entry in the NVM. The field is located in bit 23..19 of
 *  the 24 bit value.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Date value (5 bits).
 */
uint8_t sr_nvm_get_test_date_date(nvm_t *nvm);

/** @brief Get the tp version from the NVM.
 *
 *  Return the version of the test program used on this part
 *  from the TP_VERSION entry in the NVM.
 *
 *  @param[in] nvm  NVM structure.
 *  @return TP version major and minor value.
 */
uint16_t sr_nvm_get_tp_version(nvm_t *nvm);

/** @brief Get the region from the NVM.
 *
 *  Return the assessment of suitable operating regions
 *  based on test results from the REG_CORNER entry in NVM.
 *  This field is located in bit 7..4 of the entry.
 *
 *  @param[in] nvm  NVM structure .
 *  @return Region value (4 bits).
 */
uint8_t sr_nvm_get_region(nvm_t *nvm);

/** @brief Get the corner from the NVM.
 *
 *  Return the assessment of the corner based on test results
 *  from the REG_CORNER entry in NVM. This field is located in
 *  bit 3..0 of the entry.
 *
 *  @param[in] nvm  NVM structure.
 *  @return Corner value (4 bits).
 */
uint8_t sr_nvm_get_corner(nvm_t *nvm);

#ifdef __cplusplus
}
#endif

#endif /* SR_NVM_H_ */
